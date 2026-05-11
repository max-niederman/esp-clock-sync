//! Pulse-rig firmware for measuring inter-client clock-sync precision.
//!
//! Identical binary on both ESPs. Each board:
//!
//!  * drives 1 ms pulses simultaneously on **D14** (GPIO14, cross-board wire)
//!    and **D32** (GPIO32, optional self-loop wire on the *same* board) every
//!    100 ms of *synced* time, aligned to the next round 100 ms server-time
//!    tick;
//!  * captures rising edges on **D5** (GPIO5, cross-board input) and **D34**
//!    (GPIO34, self-loop input) via GPIO interrupts.
//!
//! Cross-board wiring: ESP-A.D14 → ESP-B.D5 (and optionally B.D14 → A.D5).
//! Self-loop wiring (optional, for noise-floor measurement): jumper this
//! board's D32 → D34. Pulses produce both `pulse_rx port=D5` (cross-board) and
//! `pulse_rx port=D34` (self) events, letting the operator separate sync-
//! engine error from intrinsic rig noise.
//!
//! GPIO14 was chosen over GPIO12 because GPIO12 is the MTDI strapping pin —
//! an external pull-up there switches the SPI-flash voltage at boot and
//! prevents flashing. GPIO34 is input-only on ESP32 (no pull resistors), so
//! the wire from GPIO32 must drive it directly; the resting Low on GPIO32
//! holds GPIO34 Low between pulses.
//! Each board emits log lines over UART:
//!
//!   `pulse_tx mac=ab:cd:ef seq=N synced_ns=…`
//!   `pulse_rx mac=ab:cd:ef synced_ns=… local_us=…`
//!
//! The companion `skew-meter` Linux binary correlates Tx/Rx events from both
//! serial streams and reports the inter-board timing error.

#![no_std]
#![no_main]

extern crate alloc;

mod mcpwm_cap;

use alloc::string::String;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::ram;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println::println;
use esp_radio::Controller;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiMode, WifiStaState,
};
use static_cell::StaticCell;

use clock_sync_client::ClockSyncClient;

esp_bootloader_esp_idf::esp_app_desc!();

/// Pulse cadence in nanoseconds (server time): 100 ms = 100_000_000 ns.
const PULSE_PERIOD_NS: u128 = 100_000_000;

/// Width of each TX pulse on D14, in microseconds.
const PULSE_WIDTH_US: u64 = 1_000;

/// Capture ring buffer between the GPIO ISR and the async logger.
const CAP_RING: usize = 16;

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static CELL: StaticCell<$t> = StaticCell::new();
        CELL.init($val)
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let cfg = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(cfg);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Bump UART0 to 921600 baud — esp_println shares UART0 with the
    // pulser/reporter print spam, and at 115200 a single 100-char line
    // takes ~9 ms, which collides with concurrent prints and corrupts
    // pulse_tx output (observed `pulse_tx seq=N` lines getting their
    // prefix mangled into garbage bytes, leading to a permanent
    // off-by-one in the reporter's pulse_rx pairing). At 921600 baud
    // a 100-char line takes ~1.1 ms — well below per-100ms-pulse budget.
    let _uart0 = Uart::new(
        peripherals.UART0,
        UartConfig::default().with_baudrate(921_600),
    )
    .unwrap()
    .with_tx(peripherals.GPIO1)
    .with_rx(peripherals.GPIO3);

    let mac = esp_hal::efuse::Efuse::mac_address();
    let mac_short = (mac[3], mac[4], mac[5]);
    println!(
        "\n=== clock-sync-test boot, mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ===",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    // Hardcoded WiFi config (UART prompts were flaky after long boots).
    // BSSID lock is critical on mesh networks — without it different clients
    // associate to different mesh nodes and see unrelated TSF clocks.
    let ssid = String::from("Chorus Resident Wi-Fi");
    let password = String::from("15MnG08!");
    // Pin to a single mesh AP — different APs have unrelated TSF clocks,
    // so without this lock multiple clients fall on different mesh nodes
    // and our beacon-derived sync references diverge by tens of ms.
    // 34:20:E3:63:66:B3 is the strongest (signal 95) currently.
    let bssid = parse_bssid("34:20:E3:63:66:B3");
    println!("got ssid={ssid:?} bssid={bssid:?}");

    let esp_radio_ctrl = &*mk_static!(Controller<'static>, esp_radio::init().unwrap());
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();
    wifi_controller.set_mode(WifiMode::Sta).unwrap();

    let wifi_interface = interfaces.sta;
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = ((rng.random() as u64) << 32) | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(embassy_net::StackResources<3>, embassy_net::StackResources::<3>::new()),
        seed,
    );

    spawner
        .spawn(connection(wifi_controller, ssid.clone(), password.clone(), bssid))
        .ok();
    spawner.spawn(net_task(runner)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
    loop {
        if let Some(c) = stack.config_v4() {
            println!("got IP: {}", c.address);
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
    }

    let client = clock_sync_client::install(interfaces.sniffer, spawner)
        .expect("install clock-sync");

    // GPIO output pins.
    let pulse_out = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let pulse_out_self =
        Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default());

    // GPIO inputs: configured for direction (Pull::Down on GPIO5; GPIO34 is
    // input-only with no pulls, relies on the wire from GPIO32 being driven
    // Low between pulses). We deliberately do *not* `.listen()` for an
    // interrupt — the MCPWM CAP submodule does the timestamping in
    // hardware so software ISR latency drops out of the budget entirely.
    let _capture_in = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Down),
    );
    let _capture_self = Input::new(
        peripherals.GPIO34,
        InputConfig::default().with_pull(Pull::None),
    );
    // Initialise MCPWM0 CAP. Routes GPIO5 → PWM0_CAP0 and GPIO34 →
    // PWM0_CAP1 via the GPIO matrix; latches the cap timer in hardware
    // on each rising edge. Calibrate against `Instant::now` and
    // `mac_tsf_us` so the precision paths are available.
    mcpwm_cap::init(
        peripherals.MCPWM0,
        Some(5),
        Some(34),
        Some(clock_sync_client::mac_tsf_us),
    );
    // Register a beacon-time hook that takes a SW capture of the MCPWM
    // CAP timer at exactly the same moment as the beacon anchor is
    // recorded. This lets us convert future `cap_us` event timestamps
    // straight to `ap_tsf_us` via the latest beacon anchor — no
    // `Instant↔TSF` EMA, no `mac_tsf_us` driver-mutex jitter.
    clock_sync_client::set_beacon_hook(mcpwm_cap::sw_capture_ch2_pub);
    println!("mcpwm_cap initialised");

    spawner
        .spawn(pulser(client, pulse_out, pulse_out_self, mac_short))
        .ok();
    spawner.spawn(reporter(client, mac_short)).ok();

    // Idle.
    loop {
        Timer::after(Duration::from_secs(10)).await;
        let q = client.quality();
        let d = mcpwm_cap::debug_dump();
        println!(
            "stats: samples={} drift_ppb={} p95_us={} seen={} matched={} dropped={}",
            q.n_samples,
            q.drift_ppb,
            q.residual_us_p95,
            client.frames_seen(),
            client.frames_matched(),
            client.samples_dropped()
        );
        println!(
            "mcpwm_dump: clk={:#x} cap_timer={:#x} ch0_cfg={:#x} ch1_cfg={:#x} ch0_val={:#x} ch1_val={:#x} fn109={:#x} fn110={:#x} cap_now={:#x} cap_status={:#x} ch0_hits={} ch1_hits={}",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9],
            mcpwm_cap::ch0_hits(),
            mcpwm_cap::ch1_hits(),
        );
    }
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    ssid: String,
    password: String,
    bssid: Option<[u8; 6]>,
) {
    loop {
        if matches!(esp_radio::wifi::sta_state(), WifiStaState::Connected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(2000)).await;
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let mut client_cfg = ClientConfig::default()
                .with_ssid(ssid.as_str().into())
                .with_password(password.as_str().into());
            if let Some(b) = bssid {
                client_cfg = client_cfg.with_bssid(b);
            }
            controller.set_config(&ModeConfig::Client(client_cfg)).unwrap();
            controller.start_async().await.unwrap();
        }
        match controller.connect_async().await {
            Ok(_) => println!("wifi connected"),
            Err(e) => {
                println!("wifi connect failed: {e:?}");
                Timer::after(Duration::from_millis(3000)).await;
            }
        }
    }
}

fn parse_bssid(s: &str) -> Option<[u8; 6]> {
    let mut out = [0u8; 6];
    let parts: alloc::vec::Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        return None;
    }
    for (i, p) in parts.iter().enumerate() {
        out[i] = u8::from_str_radix(p, 16).ok()?;
    }
    Some(out)
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

/// Drives D14 (cross-board wire) and D32 (self-loop wire) high for
/// `PULSE_WIDTH_US` µs at the next 100 ms-aligned synced time tick. The
/// busy-wait is on `Instant` (cheap, monotonic, sub-µs to read), but the
/// reported `synced_ns` goes through the precision path
/// (`synced_ns_at_own_tsf`) — bypassing the noisy `Instant↔TSF` α-β filter
/// for the field that's actually compared cross-board.
#[embassy_executor::task]
async fn pulser(
    client: &'static ClockSyncClient,
    mut pulse_out: Output<'static>,
    mut pulse_out_self: Output<'static>,
    mac: (u8, u8, u8),
) {
    // Wait until the estimator has converged.
    while client.synced_ns_now().is_none() {
        Timer::after(Duration::from_millis(200)).await;
    }
    println!("pulser: estimator converged, beginning pulses");

    let mut seq: u32 = 0;
    loop {
        let Some(now_ns) = client.synced_ns_now() else {
            Timer::after(Duration::from_millis(100)).await;
            continue;
        };
        // Next 100ms-aligned synced tick, at least 5 ms in the future to give
        // us scheduling slack.
        let next_tick = ((now_ns / PULSE_PERIOD_NS) + 1) * PULSE_PERIOD_NS;
        let next_tick = if next_tick - now_ns < 5_000_000 {
            next_tick + PULSE_PERIOD_NS
        } else {
            next_tick
        };
        // Target → local Instant µs (cheap, no MAC TSF call). Using the
        // EMA-tracked Instant↔AP_TSF here is fine for *timing* purposes —
        // we just need the wait to elapse roughly the right amount; the
        // measured `synced_ns` of the edge is read separately below via
        // the precision path.
        let Some(target_local_us) = client.local_us_at(next_tick) else {
            Timer::after(Duration::from_millis(100)).await;
            continue;
        };
        let now_local_us = Instant::now().as_micros();
        if target_local_us > now_local_us {
            let coarse_wait = (target_local_us - now_local_us).saturating_sub(80);
            if coarse_wait > 0 {
                Timer::after(Duration::from_micros(coarse_wait)).await;
            }
            while Instant::now().as_micros() < target_local_us {
                core::hint::spin_loop();
            }
        }
        // Edge!
        pulse_out.set_high();
        pulse_out_self.set_high();
        // Take a SW capture of the cap timer *immediately* after set_high.
        // The cap value reflects the moment of the SW-write, which is
        // ~few CPU cycles after the GPIO bus update. This shares a clock
        // with the rx-side cap reading, so the cross-board diff measured
        // through `synced_ns_at_extra` reflects only beacon-anchor noise +
        // crystal drift since the last beacon.
        let edge_cap = mcpwm_cap::sw_capture_ch2_pub();
        const CAP_TICKS_PER_US_Q32: u64 = 80u64 << 32;
        let actual_synced_ns = client
            .synced_ns_at_extra_filtered(edge_cap, CAP_TICKS_PER_US_Q32)
            .unwrap_or(next_tick);
        let release_at = Instant::now().as_micros() + PULSE_WIDTH_US;
        while Instant::now().as_micros() < release_at {
            core::hint::spin_loop();
        }
        pulse_out.set_low();
        pulse_out_self.set_low();

        println!(
            "pulse_tx mac={:02x}:{:02x}:{:02x} seq={} synced_ns={} target_ns={} cap_us={}",
            mac.0, mac.1, mac.2, seq, actual_synced_ns, next_tick, edge_cap
        );
        seq = seq.wrapping_add(1);
    }
}

/// Drains both capture rings and prints `pulse_rx port=D5|D34` lines. D5 is
/// the cross-board input; D34 is the self-loop input. Converts
/// `cap_ticks → server_ns` via [`ClockSyncClient::synced_ns_at_extra`]
/// using the latest beacon's `(extra_us=cap_at_beacon, ap_tsf_us)`
/// anchor. Both anchor and event timestamps are read from the SAME local
/// CAP timer, so the cross-board jitter from per-beacon MAC RX
/// variations + Instant↔TSF EMA smoothing is gone — the only residual
/// is per-board crystal drift over the time since the last beacon
/// (~3 µs per 100 ms beacon interval at typical 30 ppm crystal mismatch).
#[embassy_executor::task]
async fn reporter(client: &'static ClockSyncClient, mac: (u8, u8, u8)) {
    // Cap timer ticks at PWM_CLK = APB = 80 MHz on ESP32, so 1 µs of
    // ap_tsf elapses every 80 cap ticks → Q32 = 80 << 32.
    const CAP_TICKS_PER_US_Q32: u64 = 80u64 << 32;
    loop {
        let mut emitted = false;
        if let Some(cap_us) = mcpwm_cap::poll_ch0() {
            log_event("D5", client, mac, cap_us, CAP_TICKS_PER_US_Q32);
            emitted = true;
        }
        if let Some(cap_us) = mcpwm_cap::poll_ch1() {
            log_event("D34", client, mac, cap_us, CAP_TICKS_PER_US_Q32);
            emitted = true;
        }
        if !emitted {
            Timer::after(Duration::from_millis(2)).await;
        }
    }
}

fn log_event(
    port: &str,
    client: &'static ClockSyncClient,
    mac: (u8, u8, u8),
    cap_us: u64,
    cap_per_us_q32: u64,
) {
    let synced_ns = client.synced_ns_at_extra_filtered(cap_us, cap_per_us_q32);
    println!(
        "pulse_rx port={} mac={:02x}:{:02x}:{:02x} synced_ns={} cap_us={}",
        port,
        mac.0,
        mac.1,
        mac.2,
        synced_ns.map_or(String::from("NONE"), |ns| alloc::format!("{ns}")),
        cap_us
    );
}

// (Old GPIO-ISR-based capture removed — MCPWM CAP latches edge
// timestamps in hardware, see `mcpwm_cap.rs`.)
