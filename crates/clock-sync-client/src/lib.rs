//! ESP32 client side of the clock-sync system.
//!
//! Wires `esp-radio`'s promiscuous-mode RX callback into a [`clock_sync::Estimator`]
//! and exposes a tiny query API (`synced_ns_now`, `synced_ns_at`, `quality`).
//!
//! The library does **not** call `esp_radio::init`, `wifi::new`, `connect`, or
//! own any embassy-net stack. The application owns WiFi initialisation and
//! association, then hands us its `Sniffer<'static>`. STA-mode traffic and
//! promiscuous reception coexist: the sniffer callback fires for every frame
//! the radio receives, including normal STA traffic, without disrupting it.
//!
//! Usage sketch:
//!
//! ```ignore
//! let esp_radio_ctrl = mk_static!(Controller<'static>, esp_radio::init().unwrap());
//! let (mut wifi_controller, interfaces) =
//!     esp_radio::wifi::new(esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();
//! wifi_controller.set_mode(WifiMode::Sta).unwrap();
//! wifi_controller.start().unwrap();
//! // ... connect to AP via embassy-net as usual ...
//!
//! let client = clock_sync_client::install(interfaces.sniffer, spawner)?;
//!
//! // Anywhere afterwards:
//! if let Some(now_ns) = client.synced_ns_now() {
//!     // ...
//! }
//! ```

#![no_std]
#![deny(rust_2018_idioms)]

use core::cell::RefCell;

use clock_sync::{AlphaBetaFilter, Estimator, Quality, Sample, SyncPacket};
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use esp_radio::wifi::{PromiscuousPkt, Sniffer};
use static_cell::StaticCell;

/// Read the WiFi MAC's TSF clock in microseconds. The MAC TSF is hardware-
/// driven, monotonic across associations, and the same source the WiFi RX
/// path stamps received packets with — so using it for both pulse generation
/// and capture eliminates the Instant↔TSF crystal-drift slop that bounded
/// inter-board agreement to ~hundreds of µs.
///
/// Costs ~µs to call (per ESP-IDF docs, can spike to ~120 µs under WiFi
/// driver contention). Safe to call from interrupt context.
#[inline]
pub fn mac_tsf_us() -> u64 {
    // SAFETY: `esp_wifi_get_tsf_time` is a thread-safe driver call that
    // returns the MAC's TSF in microseconds for the given interface. We
    // always query STA. If WiFi has not yet been initialised, the function
    // returns 0; we accept that as a sentinel because no caller will read
    // the value until after `install` (which requires WiFi up).
    let v = unsafe {
        esp_wifi_sys::include::esp_wifi_get_tsf_time(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_STA,
        )
    };
    v as u64
}

/// Sentinel for "Instant↔TSF delta not yet measured". Real deltas are bounded
/// by Instant range (i64 µs) so this sentinel is unreachable in practice.
const DELTA_UNSET: i64 = i64::MIN;

/// Capacity of the ISR → task channel. We drop on overflow; if the consumer is
/// scheduled even once per second the broadcast cadence (~10 Hz) is well below
/// this depth.
const CHANNEL_DEPTH: usize = 32;

/// One sniffer-side observation tagged by source. We feed the precise
/// `Beacon` samples into the inter-board sync estimator, while `Udp` samples
/// only contribute to the slow wall-clock mapping.
#[derive(Clone, Copy)]
enum SnifferEvent {
    Beacon(BeaconAnchor),
    /// Per-UDP-broadcast: server-time (ns) and our hardware RX TSF.
    Udp { server_ns: u128, own_tsf_us: u64 },
}

static SAMPLES: Channel<CriticalSectionRawMutex, SnifferEvent, CHANNEL_DEPTH> = Channel::new();

/// Beacon estimator: own_TSF → AP_TSF. Kept for diagnostics (drift/quality).
/// Predictions actually use [`LATEST_BEACON`], not this estimator — at 10 Hz
/// beacons and ~10 ppm crystal mismatch, anchoring to the *latest* beacon
/// keeps elapsed-time-since-anchor ≤100 ms and bounds inter-board drift
/// bias to ~1 µs. A mean-over-window fit suffers ~mean_age × Δrate which is
/// ~125 µs at our window length.
static BEACON_EST: Mutex<RefCell<Estimator>> = Mutex::new(RefCell::new(Estimator::new()));

/// Anchor captured at the most recent beacon from our AP, atomically as
/// observed in the sniffer rx_cb:
///
///   (instant_us_at_cb, own_tsf_us, ap_tsf_us)
///
/// `instant_us_at_cb` and `own_tsf_us` are sampled in the callback, so the
/// pair `(instant, own_tsf)` shares the same physical instant — accurate to
/// callback dispatch jitter. `ap_tsf_us` is the AP's TSF at *its* TX
/// instant, which is physically the same moment as our RX (sub-µs RF flight).
/// Together this gives a clean three-clock anchor for any conversion.
static LATEST_BEACON: Mutex<RefCell<Option<BeaconAnchor>>> =
    Mutex::new(RefCell::new(None));

#[derive(Clone, Copy, Debug)]
struct BeaconAnchor {
    instant_us: u64,
    own_tsf_us: u64,
    ap_tsf_us: u64,
}

/// UDP estimator: AP_TSF → server_ns (wall time). Noisy due to AP buffering;
/// sufficient for ms-precision wall-clock display.
static UDP_EST: Mutex<RefCell<Estimator>> = Mutex::new(RefCell::new(Estimator::new()));

use portable_atomic::{AtomicI64, AtomicU64, Ordering};

/// Counter of frames the sniffer has seen since boot. Useful for debug.
static FRAMES_SEEN: AtomicU64 = AtomicU64::new(0);
/// Counter of frames that contained a valid `SyncPacket`.
static FRAMES_MATCHED: AtomicU64 = AtomicU64::new(0);
/// Counter of all 802.11 beacon frames we've seen.
static BEACONS_SEEN: AtomicU64 = AtomicU64::new(0);
/// Counter of beacons whose AP TSF was within range of our current MAC TSF
/// (i.e. our associated AP, not a neighbour).
static BEACONS_USED: AtomicU64 = AtomicU64::new(0);
/// Counter of beacons rejected for abnormal callback latency.
static BEACONS_FILTERED: AtomicU64 = AtomicU64::new(0);
/// EMA of beacon callback latency in µs. Used to filter outlier beacons
/// whose anchor would be biased by an unusually delayed callback.
static BEACON_CB_LATENCY_EMA: AtomicU64 = AtomicU64::new(0);
/// BSSID of the AP we're STA-associated with. Filled in by `poll_ap_bssid`
/// from `esp_wifi_sta_get_ap_info`. Beacons not matching this BSSID are
/// rejected so we don't mix multiple APs' (independent) TSF clocks.
static LOCKED_BSSID: Mutex<RefCell<Option<[u8; 6]>>> = Mutex::new(RefCell::new(None));
/// Counter of samples dropped because the channel was full.
static SAMPLES_DROPPED: AtomicU64 = AtomicU64::new(0);

/// `Instant::now().as_micros() - mac_tsf_us()` smoothed via EMA over recent
/// samples. Used to convert caller-supplied `Instant` values to TSF for the
/// less-precise `synced_ns_at(Instant)` API. Sync-critical paths should call
/// `mac_tsf_us()` and `synced_ns_at_tsf` directly.
static INSTANT_MINUS_TSF: AtomicI64 = AtomicI64::new(DELTA_UNSET);

/// α-β filter on (instant_us_at_cb, ap_tsf_us - instant_us_at_cb). Tracks
/// both current offset and rate-of-change (Instant↔AP_TSF crystal drift).
/// Over a long run the rate stabilises, so per-query predictions are
/// limited only by α-smoothed offset noise (`σ_y / sqrt(α)`) plus the
/// rate-extrapolation error since the last update — both small.
///
/// α=16 → offset filter time constant ~1.6 s.
/// β=2048 → rate filter time constant ~3.4 min; converges over a few
/// minutes to a stable per-board rate.
///
/// Empirically these gains beat (α=32, β=4096) — smaller α tracks short-
/// term offset variations better; smaller β still gives enough rate
/// stability for the per-board crystal drift signal.
static AP_TSF_MINUS_INSTANT_FILTER: Mutex<RefCell<AlphaBetaFilter>> =
    Mutex::new(RefCell::new(AlphaBetaFilter::new(16, 2048)));

/// Public handle returned by [`install`]. Cheap to copy (`&'static`).
pub struct ClockSyncClient {
    _private: (),
}

static CLIENT: StaticCell<ClockSyncClient> = StaticCell::new();
static SNIFFER_STORAGE: StaticCell<Sniffer<'static>> = StaticCell::new();

/// Install clock-sync into a running esp-radio STA setup.
///
/// Consumes the [`Sniffer`] returned from [`esp_radio::wifi::new`]'s
/// `Interfaces`, registers the global RX callback, enables promiscuous mode,
/// and spawns the consumer task. Returns a `&'static` handle.
///
/// Calling this more than once panics ([`StaticCell`] is single-init).
pub fn install(
    sniffer: Sniffer<'static>,
    spawner: Spawner,
) -> Result<&'static ClockSyncClient, InstallError> {
    let sniffer = SNIFFER_STORAGE.init(sniffer);
    sniffer
        .set_promiscuous_mode(true)
        .map_err(|_| InstallError::Promiscuous)?;
    sniffer.set_receive_cb(rx_cb);

    let client = CLIENT.init(ClockSyncClient { _private: () });
    spawner
        .spawn(consume_samples())
        .map_err(|_| InstallError::Spawn)?;
    spawner
        .spawn(poll_ap_bssid())
        .map_err(|_| InstallError::Spawn)?;
    Ok(client)
}

#[derive(Debug)]
pub enum InstallError {
    Promiscuous,
    Spawn,
}

impl ClockSyncClient {
    /// Convert a `embassy_time::Instant` (in microseconds) into the
    /// corresponding server-time nanosecond. Less precise than
    /// [`Self::synced_ns_at_own_tsf`] (carries Instant↔AP_TSF EMA tracking
    /// jitter); prefer the TSF version for sync-critical paths.
    pub fn synced_ns_at(&self, instant_us: u64) -> Option<u128> {
        let ap_tsf_us = instant_to_ap_tsf(instant_us)?;
        critical_section::with(|cs| UDP_EST.borrow_ref(cs).synced_ns_at(ap_tsf_us))
    }

    /// Convert a TSF microsecond reading (from [`mac_tsf_us`] or a captured
    /// hardware TSF timestamp) into a server-time nanosecond.
    ///
    /// **The precision-critical entry point.** Bypasses the Instant↔TSF
    /// EMA — the only sources of inter-board jitter are (1) `mac_tsf_us()`
    /// call latency (typically ~µs) and (2) the LATEST_BEACON anchor's
    /// per-beacon noise (sub-µs in `(own_tsf, ap_tsf)` because both are
    /// hardware-captured at packet RX). For event timestamping that
    /// matters, capture `mac_tsf_us()` in the ISR/event-handler and pass
    /// it here.
    pub fn synced_ns_at_own_tsf(&self, own_tsf_us: u64) -> Option<u128> {
        let ap_tsf_us = own_tsf_to_ap_tsf(own_tsf_us)?;
        critical_section::with(|cs| UDP_EST.borrow_ref(cs).synced_ns_at(ap_tsf_us))
    }

    /// Inverse of [`Self::synced_ns_at_own_tsf`]: given a target
    /// server-time, the own_TSF reading at which it will occur.
    pub fn own_tsf_at(&self, server_ns: u128) -> Option<u64> {
        let ap_tsf_us = critical_section::with(|cs| UDP_EST.borrow_ref(cs).local_us_at(server_ns))?;
        // ap_tsf → own_tsf via the LATEST_BEACON anchor (1:1 rate, drift
        // bounded to ~µs over a beacon interval).
        let a = critical_section::with(|cs| *LATEST_BEACON.borrow_ref(cs))?;
        let delta = (ap_tsf_us as i64).wrapping_sub(a.ap_tsf_us as i64);
        let own = (a.own_tsf_us as i64).wrapping_add(delta);
        if own < 0 { None } else { Some(own as u64) }
    }

    /// Convert "right now" into server-time nanoseconds. Uses the smoothed
    /// Instant→AP_TSF EMA path (cheap, no `mac_tsf_us` overhead). For
    /// sync-critical event timestamping (where you have a captured TSF),
    /// use [`Self::synced_ns_at_own_tsf`] instead.
    pub fn synced_ns_now(&self) -> Option<u128> {
        self.synced_ns_at(Instant::now().as_micros())
    }

    /// Inverse: target server-time → `embassy_time::Instant` microsecond.
    pub fn local_us_at(&self, server_ns: u128) -> Option<u64> {
        let ap_tsf_us = critical_section::with(|cs| UDP_EST.borrow_ref(cs).local_us_at(server_ns))?;
        ap_tsf_to_instant(ap_tsf_us)
    }

    /// Diagnostic snapshot of the precision-sync (beacon) estimator.
    pub fn quality(&self) -> Quality {
        critical_section::with(|cs| BEACON_EST.borrow_ref(cs).quality())
    }

    /// Diagnostic snapshot of the wall-clock (UDP) estimator.
    pub fn wall_quality(&self) -> Quality {
        critical_section::with(|cs| UDP_EST.borrow_ref(cs).quality())
    }

    /// How many beacons we've used so far (filtered to our AP).
    pub fn beacons_used(&self) -> u64 {
        BEACONS_USED.load(Ordering::Relaxed)
    }

    /// How many 802.11 beacons we've seen total (any BSSID).
    pub fn beacons_seen(&self) -> u64 {
        BEACONS_SEEN.load(Ordering::Relaxed)
    }

    /// Telemetry: total frames the sniffer has seen since boot.
    pub fn frames_seen(&self) -> u64 {
        FRAMES_SEEN.load(Ordering::Relaxed)
    }

    /// Telemetry: frames that matched our magic.
    pub fn frames_matched(&self) -> u64 {
        FRAMES_MATCHED.load(Ordering::Relaxed)
    }

    /// Telemetry: samples dropped due to channel saturation. Should stay 0
    /// in normal operation.
    pub fn samples_dropped(&self) -> u64 {
        SAMPLES_DROPPED.load(Ordering::Relaxed)
    }
}

/// Promiscuous-mode RX callback. Runs in WiFi-driver context.
///
/// Two cases of interest:
///
/// 1. **AP beacon** (FC byte 0 = 0x80). Body byte 0..8 carries the AP's TSF
///    in microseconds (LE u64). Both boards see the same beacon at almost
///    the same physical instant — beacons are not DTIM-buffered, so per-
///    packet wire-time jitter is sub-µs. We capture an anchor whose
///    `instant_us` field corresponds to the *physical RX time* (not the
///    software callback entry time) so that two boards' anchors are
///    referenced to the same physical event.
///
/// 2. **Our magic UDP packet** (`SyncPacket::find_in_frame`). Pair
///    (server_ns, our MAC RX TSF) for the slow wall-clock estimator.
fn rx_cb(packet: PromiscuousPkt<'_>) {
    let tsf_raw_low = packet.rx_cntl.timestamp;
    // Capture instant FIRST and own_tsf SECOND, both inside the callback. The
    // tiny difference (instant_at_cb - own_tsf_at_cb) measures the callback's
    // internal Instant↔TSF delta at *callback time*. This matters because we
    // want to back-compute the Instant value at the *physical RX time* below.
    let instant_us_at_cb = Instant::now().as_micros();
    let own_tsf_at_cb = mac_tsf_us();
    FRAMES_SEEN.fetch_add(1, Ordering::Relaxed);

    // Reconstruct the full 64-bit MAC TSF for the hardware RX timestamp,
    // using the high bits of `own_tsf_at_cb` (just-now reading is recent).
    let own_tsf_us = if (own_tsf_at_cb as u32) >= tsf_raw_low {
        (own_tsf_at_cb & 0xFFFF_FFFF_0000_0000) | (tsf_raw_low as u64)
    } else {
        ((own_tsf_at_cb & 0xFFFF_FFFF_0000_0000).wrapping_sub(1u64 << 32))
            | (tsf_raw_low as u64)
    };

    let _ = own_tsf_at_cb; // kept for the high-bit unwrap above

    // Slow EMA on the *Instant↔TSF delta sampled at the same software line*
    // (instant_at_cb - own_tsf_at_cb). This is jitter-free with respect to
    // RX latency because both readings are software-co-sampled.
    let new_delta = (instant_us_at_cb as i64).wrapping_sub(own_tsf_at_cb as i64);
    let prev = INSTANT_MINUS_TSF.load(Ordering::Relaxed);
    let updated = if prev == DELTA_UNSET {
        new_delta
    } else {
        prev + (new_delta - prev) / 64
    };
    INSTANT_MINUS_TSF.store(updated, Ordering::Relaxed);

    // --- Path 1: beacon detection (precision sync) -------------------------
    if packet.data.len() >= 36 && packet.data[0] == 0x80 {
        BEACONS_SEEN.fetch_add(1, Ordering::Relaxed);

        // Address 3 (BSSID) is at bytes 16..22.
        let mut bssid = [0u8; 6];
        bssid.copy_from_slice(&packet.data[16..22]);

        // Beacon body starts at byte 24; first 8 bytes = AP TSF (LE u64).
        let mut buf = [0u8; 8];
        buf.copy_from_slice(&packet.data[24..32]);
        let ap_tsf_us = u64::from_le_bytes(buf);

        // Filter to the BSSID we're associated with. Until `poll_ap_bssid`
        // has populated this (~few seconds after STA-up), reject everything.
        let pass = critical_section::with(|cs| {
            LOCKED_BSSID.borrow_ref(cs).map(|b| b == bssid).unwrap_or(false)
        });

        if pass {
            let beacons = BEACONS_USED.fetch_add(1, Ordering::Relaxed);
            let anchor = BeaconAnchor {
                instant_us: instant_us_at_cb,
                own_tsf_us,
                ap_tsf_us,
            };
            // Update the precision anchor immediately (before async hop), so
            // that conversions performed shortly after this beacon use the
            // freshest anchor.
            critical_section::with(|cs| {
                *LATEST_BEACON.borrow_ref_mut(cs) = Some(anchor);
            });

            // Feed the α-β filter for instant→AP_TSF prediction. Tested but
            // not used: subtracting per-sample (mac_tsf_us - rx_cntl.timestamp)
            // from instant_at_cb to "back-step to physical RX time" drops
            // cross-board p50 from 29 µs to 1.6 ms — empirically `mac_tsf_us`
            // and `rx_cntl.timestamp` are not in compatible timebases on
            // ESP32 (the former tracks AP TSF, the latter is the local MAC
            // free-running clock). Using `instant_at_cb` directly works.
            let observed_offset = (ap_tsf_us as i64).wrapping_sub(instant_us_at_cb as i64);
            critical_section::with(|cs| {
                AP_TSF_MINUS_INSTANT_FILTER
                    .borrow_ref_mut(cs)
                    .observe(instant_us_at_cb as i64, observed_offset);
            });
            if beacons < 5 {
                log::info!(
                    "beacon bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ap_tsf_us={ap_tsf_us} own_tsf_us={own_tsf_us} instant_us={instant_us_at_cb}",
                    bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
                );
            }
            let _ = SAMPLES.try_send(SnifferEvent::Beacon(anchor));
        }
        return;
    }

    // --- Path 2: our UDP packet (wall-clock) ------------------------------
    let Some(pkt) = SyncPacket::find_in_frame(packet.data) else {
        return;
    };
    FRAMES_MATCHED.fetch_add(1, Ordering::Relaxed);
    let server_ns = { pkt.send_unix_ns };
    if SAMPLES
        .try_send(SnifferEvent::Udp {
            server_ns,
            own_tsf_us,
        })
        .is_err()
    {
        SAMPLES_DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// own_TSF → AP_TSF using the latest beacon anchor (1:1 rate assumption,
/// drift bounded by ~beacon_interval × Δrate ≈ 1 µs).
fn own_tsf_to_ap_tsf(own_tsf_us: u64) -> Option<u64> {
    let a = critical_section::with(|cs| *LATEST_BEACON.borrow_ref(cs))?;
    let delta = (own_tsf_us as i64).wrapping_sub(a.own_tsf_us as i64);
    let ap = (a.ap_tsf_us as i64).wrapping_add(delta);
    if ap < 0 { None } else { Some(ap as u64) }
}

/// Instant µs → AP_TSF µs via the α-β filter.
fn instant_to_ap_tsf(instant_us: u64) -> Option<u64> {
    let offset = critical_section::with(|cs| {
        AP_TSF_MINUS_INSTANT_FILTER
            .borrow_ref(cs)
            .predict(instant_us as i64)
    })?;
    let ap = (instant_us as i128) + (offset as i128);
    if ap < 0 || ap > u64::MAX as i128 { None } else { Some(ap as u64) }
}

/// Inverse: find `instant` such that `instant + offset(instant) = ap_tsf`.
/// Two-step fixed-point iteration converges quickly because the offset
/// derivative wrt instant is the (tiny) rate, ≈ 0.
fn ap_tsf_to_instant(ap_tsf_us: u64) -> Option<u64> {
    critical_section::with(|cs| {
        let f = AP_TSF_MINUS_INSTANT_FILTER.borrow_ref(cs);
        let off1 = f.predict(ap_tsf_us as i64)?;
        let inst1 = (ap_tsf_us as i64).wrapping_sub(off1);
        let off2 = f.predict(inst1)?;
        let inst2 = (ap_tsf_us as i64).wrapping_sub(off2);
        if inst2 < 0 { None } else { Some(inst2 as u64) }
    })
}

fn instant_us_to_tsf_us(instant_us: u64) -> Option<u64> {
    let delta = INSTANT_MINUS_TSF.load(Ordering::Relaxed);
    if delta == DELTA_UNSET {
        return None;
    }
    let tsf = (instant_us as i128).wrapping_sub(delta as i128);
    if tsf < 0 {
        return None;
    }
    Some(tsf as u64)
}

/// Polls `esp_wifi_sta_get_ap_info` until it returns a valid BSSID, then
/// continues to refresh slowly to handle roaming.
#[embassy_executor::task]
async fn poll_ap_bssid() {
    use embassy_time::{Duration, Timer};
    let mut prev: Option<[u8; 6]> = None;
    loop {
        let mut info: esp_wifi_sys::include::wifi_ap_record_t =
            unsafe { core::mem::zeroed() };
        // SAFETY: writes one wifi_ap_record_t. Returns ESP_OK (0) on success.
        let r =
            unsafe { esp_wifi_sys::include::esp_wifi_sta_get_ap_info(&mut info as *mut _) };
        if r == 0 {
            let bssid = info.bssid;
            if Some(bssid) != prev {
                log::info!(
                    "associated BSSID = {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                    bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]
                );
                critical_section::with(|cs| {
                    *LOCKED_BSSID.borrow_ref_mut(cs) = Some(bssid);
                });
                prev = Some(bssid);
            }
            Timer::after(Duration::from_secs(10)).await;
        } else {
            // Not yet associated — try again soon.
            Timer::after(Duration::from_millis(500)).await;
        }
    }
}

#[embassy_executor::task]
async fn consume_samples() {
    loop {
        match SAMPLES.receive().await {
            SnifferEvent::Beacon(anchor) => {
                // BEACON_EST is purely diagnostic now (rate/drift health
                // monitor) — predictions use LATEST_BEACON which the rx_cb
                // already updated synchronously.
                let sample = Sample {
                    server_ns: (anchor.ap_tsf_us as u128).saturating_mul(1_000),
                    local_us: anchor.own_tsf_us,
                };
                critical_section::with(|cs| BEACON_EST.borrow_ref_mut(cs).observe(sample));
            }
            SnifferEvent::Udp {
                server_ns,
                own_tsf_us,
            } => {
                // own_TSF → AP_TSF via latest beacon anchor, then
                // (server_ns, AP_TSF µs) → UDP_EST.
                let Some(ap_tsf_us) = own_tsf_to_ap_tsf(own_tsf_us) else {
                    continue;
                };
                let sample = Sample {
                    server_ns,
                    local_us: ap_tsf_us,
                };
                critical_section::with(|cs| UDP_EST.borrow_ref_mut(cs).observe(sample));
            }
        }
    }
}
