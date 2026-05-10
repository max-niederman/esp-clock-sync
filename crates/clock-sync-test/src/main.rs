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

use alloc::string::String;
use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull};
use esp_hal::handler;
use esp_hal::ram;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx};
use esp_hal::Async;
use esp_println::{print, println};
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

    let mac = esp_hal::efuse::Efuse::mac_address();
    let mac_short = (mac[3], mac[4], mac[5]);
    println!(
        "\n=== clock-sync-test boot, mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ===",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    // Bootstrap WiFi creds over serial (every boot — no persistence).
    let uart0 = Uart::new(peripherals.UART0, UartConfig::default())
        .unwrap()
        .with_tx(peripherals.GPIO1)
        .with_rx(peripherals.GPIO3)
        .into_async();
    let (mut uart_rx, _uart_tx) = uart0.split();

    let ssid = read_line("ssid: ", &mut uart_rx).await;
    let password = read_line("pass: ", &mut uart_rx).await;
    let bssid_str = read_line("bssid: ", &mut uart_rx).await;
    let bssid = parse_bssid(bssid_str.trim());
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

    // GPIO setup.
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(gpio_isr);

    let pulse_out = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let pulse_out_self =
        Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default());
    let mut capture_in = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Down),
    );
    capture_in.listen(Event::RisingEdge);
    let mut capture_self = Input::new(
        peripherals.GPIO34,
        InputConfig::default().with_pull(Pull::None),
    );
    capture_self.listen(Event::RisingEdge);
    critical_section::with(|cs| {
        CAPTURE_PIN.borrow(cs).replace(Some(capture_in));
        CAPTURE_SELF.borrow(cs).replace(Some(capture_self));
    });

    spawner
        .spawn(pulser(client, pulse_out, pulse_out_self, mac_short))
        .ok();
    spawner.spawn(reporter(client, mac_short)).ok();

    // Idle.
    loop {
        Timer::after(Duration::from_secs(60)).await;
        let q = client.quality();
        println!(
            "stats: samples={} drift_ppb={} p95_us={} seen={} matched={} dropped={}",
            q.n_samples,
            q.drift_ppb,
            q.residual_us_p95,
            client.frames_seen(),
            client.frames_matched(),
            client.samples_dropped()
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
/// `PULSE_WIDTH_US` µs at the next 100 ms-aligned synced time tick. Waits
/// ~50 µs short of the target then busy-spins to nail the edge.
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
        // Convert target server time → local Instant for busy-spin (Instant
        // is sub-µs to read; the EMA-tracked Instant↔AP_TSF delta gives us
        // low-jitter conversion).
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
        let actual_synced_ns = client.synced_ns_now().unwrap_or(next_tick);
        let release_at = Instant::now().as_micros() + PULSE_WIDTH_US;
        while Instant::now().as_micros() < release_at {
            core::hint::spin_loop();
        }
        pulse_out.set_low();
        pulse_out_self.set_low();

        println!(
            "pulse_tx mac={:02x}:{:02x}:{:02x} seq={} synced_ns={} target_ns={}",
            mac.0, mac.1, mac.2, seq, actual_synced_ns, next_tick
        );
        seq = seq.wrapping_add(1);
    }
}

/// Drains both capture rings and prints `pulse_rx port=D5|D34` lines. D5 is
/// the cross-board input; D34 is the self-loop input. Either may be unwired
/// on a given board — only events from wired pins fire.
#[embassy_executor::task]
async fn reporter(client: &'static ClockSyncClient, mac: (u8, u8, u8)) {
    loop {
        let mut emitted = false;
        let cross = critical_section::with(|cs| CAP_BUF.borrow_ref_mut(cs).pop());
        if let Some(local_us) = cross {
            log_event("D5", client, mac, local_us);
            emitted = true;
        }
        let self_ev = critical_section::with(|cs| CAP_SELF_BUF.borrow_ref_mut(cs).pop());
        if let Some(local_us) = self_ev {
            log_event("D34", client, mac, local_us);
            emitted = true;
        }
        if !emitted {
            Timer::after(Duration::from_millis(5)).await;
        }
    }
}

fn log_event(port: &str, client: &'static ClockSyncClient, mac: (u8, u8, u8), local_us: u64) {
    let synced_ns = client.synced_ns_at(local_us);
    println!(
        "pulse_rx port={} mac={:02x}:{:02x}:{:02x} synced_ns={} local_us={}",
        port,
        mac.0,
        mac.1,
        mac.2,
        synced_ns.map_or(String::from("NONE"), |ns| alloc::format!("{ns}")),
        local_us
    );
}

// --- GPIO interrupt plumbing ---------------------------------------------

static CAPTURE_PIN: critical_section::Mutex<RefCell<Option<Input<'static>>>> =
    critical_section::Mutex::new(RefCell::new(None));
static CAPTURE_SELF: critical_section::Mutex<RefCell<Option<Input<'static>>>> =
    critical_section::Mutex::new(RefCell::new(None));

static CAP_BUF: critical_section::Mutex<RefCell<CaptureRing>> =
    critical_section::Mutex::new(RefCell::new(CaptureRing::new()));
static CAP_SELF_BUF: critical_section::Mutex<RefCell<CaptureRing>> =
    critical_section::Mutex::new(RefCell::new(CaptureRing::new()));

struct CaptureRing {
    buf: [u64; CAP_RING],
    head: usize,
    tail: usize,
    full: bool,
}

impl CaptureRing {
    const fn new() -> Self {
        Self {
            buf: [0; CAP_RING],
            head: 0,
            tail: 0,
            full: false,
        }
    }
    fn push(&mut self, v: u64) {
        self.buf[self.head] = v;
        self.head = (self.head + 1) % CAP_RING;
        if self.full {
            self.tail = (self.tail + 1) % CAP_RING;
        }
        if self.head == self.tail {
            self.full = true;
        }
    }
    fn pop(&mut self) -> Option<u64> {
        if self.head == self.tail && !self.full {
            return None;
        }
        let v = self.buf[self.tail];
        self.tail = (self.tail + 1) % CAP_RING;
        self.full = false;
        Some(v)
    }
}

#[handler]
#[ram]
fn gpio_isr() {
    // Capture Instant (cheap TIMG register read, ISR-safe). The reporter
    // task will convert this to own_TSF via a `mac_tsf_us` reading taken
    // immediately after popping — Instant↔TSF rate match within a few ppm,
    // so the conversion error over the few ms before the reporter runs is
    // sub-µs. We CANNOT call `mac_tsf_us` from this ISR — it acquires the
    // WiFi driver mutex and deadlocks.
    let local_us = Instant::now().as_micros();
    critical_section::with(|cs| {
        if let Some(pin) = CAPTURE_PIN.borrow_ref_mut(cs).as_mut() {
            if pin.is_interrupt_set() {
                pin.clear_interrupt();
                CAP_BUF.borrow_ref_mut(cs).push(local_us);
            }
        }
        if let Some(pin) = CAPTURE_SELF.borrow_ref_mut(cs).as_mut() {
            if pin.is_interrupt_set() {
                pin.clear_interrupt();
                CAP_SELF_BUF.borrow_ref_mut(cs).push(local_us);
            }
        }
    });
}

/// Print a prompt and read an LF-terminated line from UART0.
async fn read_line(prompt: &str, rx: &mut UartRx<'static, Async>) -> String {
    print!("{prompt}");
    let mut out = String::new();
    let mut buf = [0u8; 1];
    loop {
        match embedded_io_async::Read::read(rx, &mut buf).await {
            Ok(0) => continue,
            Ok(_) => {
                let b = buf[0];
                match b {
                    b'\n' => {
                        println!();
                        return out;
                    }
                    b'\r' => {}
                    0x08 | 0x7f => {
                        out.pop();
                    }
                    b => {
                        let s = [b];
                        if let Ok(s) = core::str::from_utf8(&s) {
                            print!("{s}");
                        }
                        out.push(b as char);
                    }
                }
            }
            Err(e) => {
                println!("uart read error: {e:?}");
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

