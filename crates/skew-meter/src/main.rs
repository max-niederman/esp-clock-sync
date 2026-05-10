//! Reads two ESP serial streams (one per board) and reports the inter-board
//! clock-sync error.
//!
//! Each board (running `clock-sync-test`) emits two flavours of log lines:
//!
//!   pulse_tx mac=ab:cd:ef seq=N synced_ns=… target_ns=…
//!   pulse_rx mac=ab:cd:ef synced_ns=… local_us=…
//!
//! The wiring is ESP-A.D12 → ESP-B.D5 (and optionally B.D12 → A.D5). For each
//! `pulse_rx` from board X, we find the closest-in-synced-time `pulse_tx` from
//! board Y (≠ X). Their `synced_ns` should agree to within 100 µs if both
//! boards' estimators agree on server time. We report the rolling distribution
//! of that delta.
//!
//! Usage:
//!
//!   skew-meter /dev/ttyUSB0 /dev/ttyUSB1
//!
//! Exits 0 once a 60-second window has elapsed during which max skew stayed
//! below `--limit-us` (default 100 µs). Exits non-zero on overshoot.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use crossbeam_channel::{Sender, unbounded};

#[derive(Parser, Debug)]
#[command(version, about = "Inter-ESP clock-sync skew meter.")]
struct Cli {
    /// Serial ports of the two ESP boards. Order doesn't matter.
    #[arg(num_args = 2)]
    ports: Vec<PathBuf>,

    /// Baud rate.
    #[arg(long, default_value_t = 115200)]
    baud: u32,

    /// Maximum acceptable inter-board skew in microseconds. Affects exit code.
    #[arg(long, default_value_t = 100)]
    limit_us: u64,

    /// Run duration in seconds.
    #[arg(long, default_value_t = 60)]
    seconds: u64,

    /// Match window: pair Tx/Rx events whose synced_ns differ by no more than
    /// this many milliseconds.
    #[arg(long, default_value_t = 50)]
    match_window_ms: u64,
}

#[derive(Clone, Debug)]
enum Event {
    Tx {
        device: usize,
        #[allow(dead_code)] mac: String,
        #[allow(dead_code)] seq: u32,
        synced_ns: u128,
    },
    Rx {
        device: usize,
        #[allow(dead_code)] mac: String,
        synced_ns: u128,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    if cli.ports.len() != 2 {
        anyhow::bail!("exactly two serial ports required (got {})", cli.ports.len());
    }

    let (tx, rx) = unbounded::<Event>();
    for (i, path) in cli.ports.iter().enumerate() {
        spawn_reader(i, path.clone(), cli.baud, tx.clone())?;
    }
    drop(tx);

    let limit_ns: i128 = (cli.limit_us as i128) * 1000;
    let match_window_ns: u128 = (cli.match_window_ms as u128) * 1_000_000;
    let deadline = Instant::now() + Duration::from_secs(cli.seconds);

    let mut tx_history: Vec<VecDeque<Event>> = vec![VecDeque::new(), VecDeque::new()];
    let mut stats_per_dir: [DirStats; 2] = Default::default();

    let mut last_print = Instant::now();
    while Instant::now() < deadline {
        let timeout = deadline.saturating_duration_since(Instant::now());
        let event = match rx.recv_timeout(timeout.min(Duration::from_millis(500))) {
            Ok(e) => e,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                maybe_print_stats(&mut last_print, &stats_per_dir, cli.limit_us);
                continue;
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
        };
        match event {
            ev @ Event::Tx { device, .. } => {
                let q = &mut tx_history[device];
                q.push_back(ev);
                let cutoff_synced_ns = match q.back() {
                    Some(Event::Tx { synced_ns, .. }) => synced_ns.saturating_sub(match_window_ns * 4),
                    _ => 0,
                };
                while let Some(Event::Tx { synced_ns, .. }) = q.front() {
                    if *synced_ns < cutoff_synced_ns {
                        q.pop_front();
                    } else {
                        break;
                    }
                }
            }
            Event::Rx {
                device: rx_dev,
                synced_ns: rx_sync,
                ..
            } => {
                // Look for nearest Tx on the *other* device.
                let other = 1 - rx_dev;
                let q = &tx_history[other];
                if q.is_empty() {
                    continue;
                }
                // Find min |Δ|.
                let mut best: Option<(u128, i128)> = None; // (tx_synced_ns, delta)
                for ev in q.iter() {
                    if let Event::Tx { synced_ns, .. } = ev {
                        let d = (rx_sync as i128) - (*synced_ns as i128);
                        if d.unsigned_abs() <= match_window_ns {
                            if best.map_or(true, |(_, b)| d.unsigned_abs() < b.unsigned_abs()) {
                                best = Some((*synced_ns, d));
                            }
                        }
                    }
                }
                if let Some((_tx_synced, delta_ns)) = best {
                    let dir = direction_idx(other, rx_dev);
                    stats_per_dir[dir].record(delta_ns, limit_ns);
                }
                maybe_print_stats(&mut last_print, &stats_per_dir, cli.limit_us);
            }
        }
    }

    println!("\n=== final ===");
    let mut overshoot = false;
    for (i, stats) in stats_per_dir.iter().enumerate() {
        let label = match i {
            0 => "device 0 → device 1",
            1 => "device 1 → device 0",
            _ => unreachable!(),
        };
        if stats.n == 0 {
            println!("{label}: no matched events");
            continue;
        }
        let line = stats.format(label);
        println!("{line}");
        if stats.max_abs_us > cli.limit_us {
            overshoot = true;
        }
    }
    if overshoot {
        std::process::exit(2);
    }
    Ok(())
}

fn direction_idx(tx_dev: usize, _rx_dev: usize) -> usize {
    // We just key per-Tx-device; works for the two-device case.
    tx_dev
}

fn maybe_print_stats(last: &mut Instant, stats: &[DirStats; 2], limit_us: u64) {
    if last.elapsed() < Duration::from_secs(1) {
        return;
    }
    *last = Instant::now();
    for (i, s) in stats.iter().enumerate() {
        if s.n == 0 {
            continue;
        }
        let label = match i {
            0 => "0→1",
            1 => "1→0",
            _ => unreachable!(),
        };
        let ok = if s.max_abs_us <= limit_us { "OK " } else { "!! " };
        println!(
            "{ok} {label}  n={:>5}  mean={:+6}µs  stddev={:>5}µs  p99={:>5}µs  max|Δ|={:>5}µs",
            s.n, s.mean_us, s.stddev_us, s.p99_abs_us, s.max_abs_us
        );
    }
}

#[derive(Default, Clone, Debug)]
struct DirStats {
    n: u64,
    sum_us: i64,
    sum_sq_us: i128,
    max_abs_us: u64,
    /// Approximation of p99 via reservoir of recent |Δ| in µs.
    recent_abs_us: VecDeque<u64>,
    p99_abs_us: u64,
    mean_us: i64,
    stddev_us: u64,
}

impl DirStats {
    fn record(&mut self, delta_ns: i128, _limit_ns: i128) {
        let delta_us = (delta_ns / 1_000) as i64;
        self.n += 1;
        self.sum_us += delta_us;
        self.sum_sq_us += (delta_us as i128) * (delta_us as i128);
        let abs = delta_us.unsigned_abs();
        if abs > self.max_abs_us {
            self.max_abs_us = abs;
        }
        self.recent_abs_us.push_back(abs);
        if self.recent_abs_us.len() > 1024 {
            self.recent_abs_us.pop_front();
        }
        let mut sorted: Vec<u64> = self.recent_abs_us.iter().copied().collect();
        sorted.sort_unstable();
        let idx = (sorted.len() as f64 * 0.99) as usize;
        let idx = idx.min(sorted.len().saturating_sub(1));
        self.p99_abs_us = sorted.get(idx).copied().unwrap_or(0);
        self.mean_us = self.sum_us / self.n as i64;
        let var = (self.sum_sq_us / self.n as i128) - (self.mean_us as i128).pow(2);
        let var = var.max(0) as u128;
        self.stddev_us = (var as f64).sqrt() as u64;
    }

    fn format(&self, label: &str) -> String {
        format!(
            "{label}  n={:>5}  mean={:+6}µs  stddev={:>5}µs  p99={:>5}µs  max|Δ|={:>5}µs",
            self.n, self.mean_us, self.stddev_us, self.p99_abs_us, self.max_abs_us
        )
    }
}

fn spawn_reader(device: usize, path: PathBuf, baud: u32, tx: Sender<Event>) -> Result<()> {
    let port = serialport::new(path.to_string_lossy(), baud)
        .timeout(Duration::from_millis(500))
        .open()
        .with_context(|| format!("open {path:?}"))?;
    let reader = BufReader::new(port);
    std::thread::Builder::new()
        .name(format!("reader-{device}"))
        .spawn(move || {
            for line in reader.lines() {
                let line = match line {
                    Ok(l) => l,
                    Err(_) => continue, // timeouts are expected
                };
                if let Some(ev) = parse_line(device, &line) {
                    let _ = tx.send(ev);
                }
            }
        })?;
    Ok(())
}

fn parse_line(device: usize, line: &str) -> Option<Event> {
    if let Some(rest) = line.strip_prefix("pulse_tx ") {
        let mut mac: Option<String> = None;
        let mut seq: Option<u32> = None;
        let mut synced_ns: Option<u128> = None;
        for kv in rest.split_whitespace() {
            let (k, v) = kv.split_once('=')?;
            match k {
                "mac" => mac = Some(v.to_string()),
                "seq" => seq = v.parse().ok(),
                "synced_ns" => synced_ns = v.parse().ok(),
                _ => {}
            }
        }
        Some(Event::Tx {
            device,
            mac: mac?,
            seq: seq?,
            synced_ns: synced_ns?,
        })
    } else if let Some(rest) = line.strip_prefix("pulse_rx ") {
        let mut mac: Option<String> = None;
        let mut synced_ns: Option<u128> = None;
        for kv in rest.split_whitespace() {
            let (k, v) = kv.split_once('=')?;
            match k {
                "mac" => mac = Some(v.to_string()),
                "synced_ns" if v != "NONE" => synced_ns = v.parse().ok(),
                _ => {}
            }
        }
        Some(Event::Rx {
            device,
            mac: mac?,
            synced_ns: synced_ns?,
        })
    } else {
        None
    }
}
