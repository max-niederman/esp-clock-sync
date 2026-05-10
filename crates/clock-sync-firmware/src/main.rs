//! Minimal `clock-sync` example firmware for ESP32.
//!
//! Bootstraps WiFi STA from credentials read over the USB-UART, joins the AP,
//! installs `clock-sync-client`, and then runs a heartbeat task that sends the
//! current synced server-time to the time-server every second over a regular
//! embassy-net UDP socket. The heartbeat proves that promiscuous-mode
//! reception (used internally by the sync engine) does not interfere with
//! ordinary embassy-net traffic.
//!
//! Bootstrap protocol on UART0 @ 115200 8N1, every boot:
//!
//!   <- "ssid: "
//!   -> <ssid><CR/LF>
//!   <- "pass: "
//!   -> <password><CR/LF>
//!   <- "server: "
//!   -> <server-ipv4><CR/LF>
//!
//! Empty password ⇒ open network.

#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use core::net::Ipv4Addr;
use core::str::FromStr;

use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
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

esp_bootloader_esp_idf::esp_app_desc!();

const HEARTBEAT_PORT: u16 = 51235;

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static CELL: StaticCell<$t> = StaticCell::new();
        CELL.init($val)
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // MAC address — useful for identifying which physical board this is.
    let mac = esp_hal::efuse::Efuse::mac_address();
    println!(
        "\n=== clock-sync-firmware boot, mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ===",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    // Bring up UART0 RX for the bootstrap prompt. UART0 TX is also used by
    // esp-println for log output; the two coexist because they're independent
    // FIFO directions.
    let uart0 = Uart::new(peripherals.UART0, UartConfig::default())
        .unwrap()
        .with_tx(peripherals.GPIO1)
        .with_rx(peripherals.GPIO3)
        .into_async();
    let (mut uart_rx, _uart_tx) = uart0.split();

    let ssid = read_line("ssid: ", &mut uart_rx).await;
    let password = read_line("pass: ", &mut uart_rx).await;
    let server_str = read_line("server: ", &mut uart_rx).await;
    let server_ip = Ipv4Addr::from_str(server_str.trim()).unwrap_or_else(|_| {
        println!("invalid server IP {server_str:?}, falling back to 0.0.0.0 (heartbeat off)");
        Ipv4Addr::UNSPECIFIED
    });
    println!("got ssid={ssid:?} server={server_ip}");

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
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner
        .spawn(connection(wifi_controller, ssid.clone(), password.clone()))
        .ok();
    spawner.spawn(net_task(runner)).ok();

    // Wait for link and DHCP.
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
    println!("waiting for DHCP...");
    loop {
        if let Some(cfg) = stack.config_v4() {
            println!("got IP: {}", cfg.address);
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
    }

    // Install clock-sync. After this, the sniffer callback will start
    // populating the estimator from broadcast frames.
    let client = clock_sync_client::install(interfaces.sniffer, spawner)
        .expect("install clock-sync");

    // Heartbeat — proves coexistence with ordinary embassy-net traffic.
    if !server_ip.is_unspecified() {
        let mut rx_meta = [PacketMetadata::EMPTY; 4];
        let mut rx_buf = [0u8; 256];
        let mut tx_meta = [PacketMetadata::EMPTY; 4];
        let mut tx_buf = [0u8; 256];
        let mut sock = UdpSocket::new(
            stack,
            &mut rx_meta,
            &mut rx_buf,
            &mut tx_meta,
            &mut tx_buf,
        );
        sock.bind(0).unwrap();
        let dest = IpEndpoint::new(IpAddress::Ipv4(server_ip), HEARTBEAT_PORT);

        loop {
            Timer::after(Duration::from_secs(1)).await;
            let q = client.quality();
            let synced = client.synced_ns_now();
            let line = match synced {
                Some(ns) => alloc::format!(
                    "hb mac={:02x}:{:02x}:{:02x} synced_ns={} samples={} drift_ppb={} p95_us={} dropped={}\n",
                    mac[3], mac[4], mac[5], ns, q.n_samples, q.drift_ppb, q.residual_us_p95, client.samples_dropped()
                ),
                None => alloc::format!(
                    "hb mac={:02x}:{:02x}:{:02x} synced=NONE samples={} seen={} matched={}\n",
                    mac[3], mac[4], mac[5], q.n_samples, client.frames_seen(), client.frames_matched()
                ),
            };
            if let Err(e) = sock.send_to(line.as_bytes(), dest).await {
                log::warn!("hb send error: {e:?}");
            }
            println!("{}", line.trim_end());
        }
    } else {
        loop {
            Timer::after(Duration::from_secs(2)).await;
            let q = client.quality();
            println!(
                "no server IP — sync-only: samples={} drift_ppb={} p95_us={} seen={} matched={}",
                q.n_samples,
                q.drift_ppb,
                q.residual_us_p95,
                client.frames_seen(),
                client.frames_matched()
            );
        }
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>, ssid: String, password: String) {
    println!("starting wifi connection task");
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(2000)).await;
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let cfg = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(ssid.as_str().into())
                    .with_password(password.as_str().into()),
            );
            controller.set_config(&cfg).unwrap();
            controller.start_async().await.unwrap();
            println!("wifi started");
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

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

/// Print a prompt and read an LF-terminated line from UART0 (CR is stripped).
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
                        // Echo newline back so the user sees it in their terminal.
                        println!();
                        return out;
                    }
                    b'\r' => {
                        // Wait for the LF; ignore CR.
                    }
                    0x08 | 0x7f => {
                        // Backspace / DEL
                        out.pop();
                    }
                    b => {
                        // Echo so the user can see what they typed.
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
