//! Linux clock-sync time-server.
//!
//! Broadcasts a [`clock_sync::SyncPacket`] every `--interval-ms` to the chosen
//! UDP destination. Defaults to `255.255.255.255:51234` (subnet broadcast).
//!
//! The send-time is captured as late as possible before the syscall: any kernel
//! delay between `send_unix_ns` and the wire is absorbed as a constant offset
//! by the per-client estimator and does not affect inter-client agreement
//! (which is what we actually care about).

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::Parser;
use clock_sync::{DEFAULT_PORT, SyncPacket};

#[derive(Parser, Debug)]
#[command(version, about = "ESP32 clock-sync time-server (UDP broadcast).")]
struct Cli {
    /// Broadcast interval in milliseconds.
    #[arg(long, default_value_t = 100)]
    interval_ms: u64,

    /// Destination port.
    #[arg(long, default_value_t = DEFAULT_PORT)]
    port: u16,

    /// Destination broadcast address. Use `255.255.255.255` for limited
    /// broadcast, or e.g. `192.168.1.255` for a directed subnet broadcast.
    #[arg(long, default_value = "255.255.255.255")]
    broadcast_addr: Ipv4Addr,

    /// Optional local interface to bind to. If unset, the kernel picks one.
    /// On Linux this sets `SO_BINDTODEVICE`, which is the most reliable way to
    /// force outgoing broadcasts onto a specific NIC.
    #[arg(long)]
    bind_iface: Option<String>,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    let sock =
        UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0)).context("bind UDP socket")?;
    sock.set_broadcast(true).context("set SO_BROADCAST")?;
    if let Some(iface) = cli.bind_iface.as_deref() {
        bind_to_device(&sock, iface).with_context(|| format!("SO_BINDTODEVICE = {iface:?}"))?;
        log::info!("bound to interface {}", iface);
    }

    let dest = SocketAddrV4::new(cli.broadcast_addr, cli.port);
    log::info!(
        "broadcasting clock-sync packets to {} every {} ms",
        dest,
        cli.interval_ms
    );

    let interval = Duration::from_millis(cli.interval_ms);
    let interval_ns: u128 = (cli.interval_ms as u128) * 1_000_000;

    // We embed *deterministic* timestamps in the packets:
    //   send_unix_ns(seq) = T0 + seq * interval_ns
    // T0 is captured once at startup. The actual sendmsg jitter (typically
    // milliseconds, much larger than our 100 µs target) does NOT enter the
    // payload, so each ESP sees identical (server_ns, seq) tables. Only the
    // hardware RX timestamp at each ESP carries the real wire-time jitter,
    // and that's where the regression averages it out.
    let t0: u128 = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    log::info!("T0 = {t0} ns; interval = {interval_ns} ns");

    let start = Instant::now();
    let mut next = start;
    let mut seq: u32 = 0;
    loop {
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        }
        next += interval;

        let send_unix_ns = t0 + (seq as u128) * interval_ns;
        let pkt = SyncPacket {
            magic: clock_sync::MAGIC,
            seq,
            send_unix_ns,
        };
        if let Err(e) = sock.send_to(&pkt.to_bytes(), dest) {
            log::warn!("send_to failed: {e}");
        } else if seq.is_multiple_of(50) {
            log::debug!("sent seq={seq} send_unix_ns={send_unix_ns}");
        }
        seq = seq.wrapping_add(1);
    }
}

#[cfg(target_os = "linux")]
fn bind_to_device(sock: &UdpSocket, iface: &str) -> Result<()> {
    use std::os::fd::AsRawFd;
    let cstr = std::ffi::CString::new(iface).context("interface name has NUL byte")?;
    // SAFETY: setsockopt with SO_BINDTODEVICE expects a NUL-terminated string.
    let ret = unsafe {
        libc::setsockopt(
            sock.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_BINDTODEVICE,
            cstr.as_ptr() as *const _,
            (cstr.as_bytes_with_nul().len()) as libc::socklen_t,
        )
    };
    if ret != 0 {
        Err(std::io::Error::last_os_error()).context("SO_BINDTODEVICE")
    } else {
        Ok(())
    }
}

#[cfg(not(target_os = "linux"))]
fn bind_to_device(_: &UdpSocket, _: &str) -> Result<()> {
    anyhow::bail!("--bind-iface is only supported on Linux")
}
