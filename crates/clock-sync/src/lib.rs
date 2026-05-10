//! Shared types for the ESP32 clock-sync system.
//!
//! Two halves:
//!
//! * [`SyncPacket`] — the on-the-wire UDP payload broadcast by the time-server
//!   and observed by every client.
//! * [`Estimator`] — a fixed-size, no_std, no-alloc sliding-window weighted
//!   linear regression that maps each client's local microsecond clock onto the
//!   server's nanosecond wall clock. The same `Estimator` is fed by every
//!   client; because every client observes the *same* broadcast frame at very
//!   nearly the same physical instant, every client's estimator agrees on the
//!   server time at any local moment, which is what gives us inter-client
//!   agreement well below 100 µs.
//!
//! The library compiles with `default-features = false` for `no_std`
//! environments (clock-sync-client, the on-device firmware) and with the
//! default `std` feature for the host server / skew-meter.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(rust_2018_idioms)]

use core::mem::size_of;

/// Magic bytes identifying a clock-sync packet on the wire. Doubles as the
/// search key for promiscuous-mode RX callbacks.
pub const MAGIC: [u8; 4] = *b"CSY1";

/// Default UDP port for clock-sync broadcasts.
pub const DEFAULT_PORT: u16 = 51234;

/// Default IPv4 subnet broadcast address ("everywhere on this LAN").
pub const DEFAULT_BROADCAST_ADDR: [u8; 4] = [255, 255, 255, 255];

/// Wire format. 24 bytes, little-endian. Packed so it fits exactly into a
/// minimum UDP payload.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct SyncPacket {
    /// Always [`MAGIC`].
    pub magic: [u8; 4],
    /// Wrapping sequence counter. Useful for debugging / dedup.
    pub seq: u32,
    /// Server `CLOCK_REALTIME` at the moment of `sendmsg`, in nanoseconds.
    pub send_unix_ns: u128,
}

const _: () = assert!(size_of::<SyncPacket>() == 24);

impl SyncPacket {
    pub const SIZE: usize = 24;

    /// View as raw bytes, suitable for `UdpSocket::send_to`.
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0..4].copy_from_slice(&self.magic);
        out[4..8].copy_from_slice(&self.seq.to_le_bytes());
        out[8..24].copy_from_slice(&self.send_unix_ns.to_le_bytes());
        out
    }

    /// Parse from a raw byte slice. Returns `None` unless `buf` is at least
    /// 24 bytes and starts with [`MAGIC`].
    pub fn parse(buf: &[u8]) -> Option<SyncPacket> {
        if buf.len() < Self::SIZE || buf[0..4] != MAGIC {
            return None;
        }
        let seq = u32::from_le_bytes(buf[4..8].try_into().ok()?);
        let send_unix_ns = u128::from_le_bytes(buf[8..24].try_into().ok()?);
        Some(SyncPacket {
            magic: MAGIC,
            seq,
            send_unix_ns,
        })
    }

    /// Find the first occurrence of [`MAGIC`] within `buf` and try to parse a
    /// `SyncPacket` starting there. Useful in promiscuous-mode callbacks where
    /// the packet is wrapped in 802.11 + LLC + IPv4 + UDP headers of varying
    /// length.
    pub fn find_in_frame(buf: &[u8]) -> Option<SyncPacket> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let last = buf.len() - Self::SIZE;
        let mut i = 0usize;
        while i <= last {
            if buf[i] == MAGIC[0]
                && buf[i + 1] == MAGIC[1]
                && buf[i + 2] == MAGIC[2]
                && buf[i + 3] == MAGIC[3]
            {
                if let Some(pkt) = Self::parse(&buf[i..]) {
                    return Some(pkt);
                }
            }
            i += 1;
        }
        None
    }
}

/// One observation: server time `server_ns` corresponded to the local
/// microsecond clock reading `local_us` at the same physical instant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub server_ns: u128,
    pub local_us: u64,
}

/// Diagnostic output from [`Estimator::quality`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Quality {
    /// Number of inlier samples currently in the window (0..=N).
    pub n_samples: u16,
    /// 95th-percentile residual (|server_predicted - server_observed|) in
    /// microseconds. Approximated as `2 * stddev`.
    pub residual_us_p95: u32,
    /// Estimated frequency offset between local clock and server clock,
    /// expressed in parts per billion. Positive => local clock runs fast.
    pub drift_ppb: i32,
    /// Local microsecond timestamp of the most recent inlier observation.
    /// Zero if no observation has been made.
    pub last_update_us: u64,
}

/// Maximum number of samples kept in the regression window.
///
/// Sized for the WiFi-broadcast scenario, where AP DTIM buffering inflates
/// per-sample wire-time jitter to ~10s of milliseconds. Averaging over 256
/// samples (~25 s at 10 Hz) brings the per-board mean-offset uncertainty into
/// the millisecond range and the inter-board agreement into hundreds of µs.
pub const WINDOW: usize = 256;

/// Minimum number of inliers before [`Estimator::synced_ns_at`] returns
/// `Some`.
pub const MIN_SAMPLES: usize = 8;

/// Sliding-window weighted linear regression mapping local µs → server ns.
///
/// Internally we store the window in a ring buffer, then on every `observe`
/// recompute affine `(offset, slope)` parameters such that
/// `server_ns ≈ offset + slope * local_us`. `slope` is stored as `1 + drift`
/// where `drift` is in parts per billion. Math is done entirely in fixed-point
/// integer arithmetic so this works on chips without an FPU.
#[derive(Debug)]
pub struct Estimator {
    samples: [Option<Sample>; WINDOW],
    head: usize,
    n: usize,
    /// Last fit: `server_ns ≈ offset_ns + (local_us - anchor_us) * 1000 +
    ///          (local_us - anchor_us) * drift_ppb / 1_000_000` (in ns).
    anchor_us: u64,
    offset_ns: i128,
    drift_ppb: i64,
    residual_us_p95: u32,
    last_update_us: u64,
}

impl Default for Estimator {
    fn default() -> Self {
        Self::new()
    }
}

impl Estimator {
    pub const fn new() -> Self {
        Self {
            samples: [None; WINDOW],
            head: 0,
            n: 0,
            anchor_us: 0,
            offset_ns: 0,
            drift_ppb: 0,
            residual_us_p95: u32::MAX,
            last_update_us: 0,
        }
    }

    /// Add an observation, updating the affine fit.
    ///
    /// No outlier rejection — when the time-server uses deterministic
    /// timestamps, every sample's `server_ns` is correct, and "outlying"
    /// samples are real reception-time jitter we want averaged in. Rejecting
    /// them based on the current fit creates a feedback loop that locks the
    /// regression onto whichever direction noise pulled it first.
    pub fn observe(&mut self, sample: Sample) {
        self.samples[self.head] = Some(sample);
        self.head = (self.head + 1) % WINDOW;
        if self.n < WINDOW {
            self.n += 1;
        }
        self.last_update_us = sample.local_us;
        self.refit();
    }

    /// Return the server time corresponding to the given *local* microsecond
    /// reading, or `None` if we don't yet have enough samples.
    ///
    /// Uses the simple model `server_ns = offset + 1000 * local_us` (slope is
    /// nominal — see [`Self::refit`]).
    pub fn synced_ns_at(&self, local_us: u64) -> Option<u128> {
        if self.n < MIN_SAMPLES {
            return None;
        }
        let predicted = self
            .offset_ns
            .saturating_add((local_us as i128).saturating_mul(1_000));
        if predicted < 0 {
            return None;
        }
        Some(predicted as u128)
    }

    /// Inverse: given a target server-time, return the local µs reading at
    /// which it will occur.
    pub fn local_us_at(&self, server_ns: u128) -> Option<u64> {
        if self.n < MIN_SAMPLES {
            return None;
        }
        let num = (server_ns as i128).saturating_sub(self.offset_ns);
        if num < 0 {
            return None;
        }
        Some((num / 1_000) as u64)
    }

    pub fn quality(&self) -> Quality {
        Quality {
            n_samples: self.n as u16,
            residual_us_p95: if self.n >= MIN_SAMPLES {
                self.residual_us_p95
            } else {
                u32::MAX
            },
            drift_ppb: self.drift_ppb.clamp(i32::MIN as i64, i32::MAX as i64) as i32,
            last_update_us: self.last_update_us,
        }
    }

    fn refit(&mut self) {
        // Strategy: assume nominal slope of 1000 ns/µs (true crystal drift is
        // bounded by ±50 ppm and dominated by per-sample noise on this
        // timescale). Compute only the OFFSET as a robust statistic — the
        // mean over the window of `(server_ns - 1000 * local_us)`.
        //
        // Per-sample noise from AP DTIM buffering on `local_us` is on the
        // order of tens of ms, but this noise is *shared* across ESPs (they
        // receive the same broadcast at the same wire-time), so both
        // estimators converge to the same mean offset and inter-board
        // agreement bottoms out at ~hardware-RX-jitter / sqrt(N) ≈ µs.
        let mut k: u32 = 0;
        let mut sum_offset_ns: i128 = 0;
        for s in self.samples.iter().flatten() {
            let offset = (s.server_ns as i128) - (s.local_us as i128) * 1_000;
            sum_offset_ns += offset;
            k += 1;
        }
        if (k as usize) < MIN_SAMPLES {
            return;
        }
        let mean_offset_ns = sum_offset_ns / k as i128;

        // Slope-as-drift: keep the field for diagnostic purposes, recompute
        // it cheaply with the simple two-point fit between the oldest and
        // newest in-window sample. This is just a hint for callers; we don't
        // use it in `synced_ns_at`.
        let mut oldest: Option<&Sample> = None;
        let mut newest: Option<&Sample> = None;
        for s in self.samples.iter().flatten() {
            match (oldest, newest) {
                (None, None) => {
                    oldest = Some(s);
                    newest = Some(s);
                }
                (Some(o), Some(n)) => {
                    if s.local_us < o.local_us {
                        oldest = Some(s);
                    }
                    if s.local_us > n.local_us {
                        newest = Some(s);
                    }
                }
                _ => unreachable!(),
            }
        }
        let drift_ppb = match (oldest, newest) {
            (Some(o), Some(n)) if n.local_us > o.local_us + 1_000_000 => {
                let dx_us = (n.local_us - o.local_us) as i128;
                let dy_ns = (n.server_ns as i128) - (o.server_ns as i128);
                // slope_ns_per_us = dy / dx, drift_ppb = (slope - 1000) * 1e6
                let slope_q = dy_ns.saturating_mul(1_000_000) / dx_us;
                let drift = slope_q - 1_000_000_000;
                drift.clamp(-1_000_000, 1_000_000) as i64
            }
            _ => 0,
        };

        // Anchor at 0 µs so synced_ns_at(local_us) is just
        //   offset_ns + 1000 * local_us
        // (drift is intentionally unused here; see top of fn).
        self.anchor_us = 0;
        self.offset_ns = mean_offset_ns;
        self.drift_ppb = drift_ppb;

        // Residual = |observed_server_ns - (offset + 1000*local_us)|
        // = |observed_server_ns - sample-by-sample offset estimate|
        // = std of per-sample offsets.
        let mut sum_sq_us: u128 = 0;
        for s in self.samples.iter().flatten() {
            let per_sample_offset = (s.server_ns as i128) - (s.local_us as i128) * 1_000;
            let resid_ns = (per_sample_offset - mean_offset_ns).unsigned_abs();
            let resid_us = (resid_ns / 1_000) as u128;
            sum_sq_us = sum_sq_us.saturating_add(resid_us * resid_us);
        }
        let var_us = sum_sq_us / k as u128;
        let std_us = isqrt_u128(var_us) as u64;
        self.residual_us_p95 = (std_us.saturating_mul(2)).min(u32::MAX as u64) as u32;
    }
}

/// Sliding-window linear regression of `y` against `x`, both `i64` (typically
/// in microseconds). Tracks **both** intercept *and* slope, so once enough
/// samples have accumulated, the slope absorbs slow drift between the two
/// clocks and per-sample callback jitter no longer leaks into the prediction.
///
/// Predictions are `O(1)`; updates are `O(N)`.
///
/// Use this for the Instant↔AP_TSF mapping in clock-sync-client: with a long
/// window, the slope (≈ relative crystal drift, ~10s of ppm) becomes well
/// determined, leaving short-term predictions limited only by `σ_y / sqrt(N)`
/// rather than `σ_y` per sample.
pub struct LinearTracker {
    samples: [Option<(i64, i64)>; 256],
    head: usize,
    n: u16,
    /// Most-recently-inserted sample x. Tracked so predictions can know
    /// "how far past the window are we extrapolating".
    last_x: i64,
    /// Window mean of x (the anchor for the fitted line — not the latest
    /// sample, so per-sample noise on the latest sample doesn't bias
    /// predictions).
    mean_x: i64,
    /// Window mean of y, the y-coordinate of the line's anchor point.
    mean_y: i64,
    /// Fitted slope `dy/dx` scaled by 2^32 to preserve sub-ppm precision in
    /// integer arithmetic.
    slope_q32: i64,
}

impl Default for LinearTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl LinearTracker {
    pub const N: usize = 256;
    /// Predictions return `Some` once at least this many samples have been
    /// observed.
    pub const MIN_SAMPLES: usize = 8;

    pub const fn new() -> Self {
        Self {
            samples: [None; Self::N],
            head: 0,
            n: 0,
            last_x: 0,
            mean_x: 0,
            mean_y: 0,
            slope_q32: 0, // 0 by default — y is constant
        }
    }

    pub fn n_samples(&self) -> u16 {
        self.n
    }

    /// Slope in parts per billion (signed).
    pub fn slope_ppb(&self) -> i64 {
        ((self.slope_q32 as i128 * 1_000_000_000) >> 32) as i64
    }

    pub fn observe(&mut self, x: i64, y: i64) {
        self.samples[self.head] = Some((x, y));
        self.head = (self.head + 1) % Self::N;
        if (self.n as usize) < Self::N {
            self.n += 1;
        }
        self.last_x = x;
        self.refit();
    }

    /// Predict `y` at the given `x`. Returns `None` until at least
    /// `MIN_SAMPLES` observations have been made.
    pub fn predict(&self, x: i64) -> Option<i64> {
        if (self.n as usize) < Self::MIN_SAMPLES {
            return None;
        }
        let dx = (x as i128) - (self.mean_x as i128);
        let slope_part = (dx * (self.slope_q32 as i128)) >> 32;
        let y = (self.mean_y as i128) + slope_part;
        if y < i64::MIN as i128 || y > i64::MAX as i128 {
            None
        } else {
            Some(y as i64)
        }
    }

    /// Inverse of [`Self::predict`].
    pub fn predict_inverse(&self, y: i64) -> Option<i64> {
        if (self.n as usize) < Self::MIN_SAMPLES {
            return None;
        }
        if self.slope_q32 == 0 {
            // Flat fit — no unique inverse. Return the mean x.
            return Some(self.mean_x);
        }
        let dy = (y as i128) - (self.mean_y as i128);
        // dx = dy / slope. Multiply by 2^32 then divide by slope_q32.
        let dx = (dy << 32) / (self.slope_q32 as i128);
        let x = (self.mean_x as i128) + dx;
        if x < i64::MIN as i128 || x > i64::MAX as i128 {
            None
        } else {
            Some(x as i64)
        }
    }

    fn refit(&mut self) {
        // Anchor at the most recent x for numerical conditioning, then
        // recompute mean_x / mean_y in absolute coordinates so predictions
        // pass through the centroid of the window.
        let anchor_x = self.last_x as i128;
        let mut sum_dx: i128 = 0;
        let mut sum_y: i128 = 0;
        let mut k: i128 = 0;
        for s in self.samples.iter().flatten() {
            sum_dx += (s.0 as i128) - anchor_x;
            sum_y += s.1 as i128;
            k += 1;
        }
        if k < Self::MIN_SAMPLES as i128 {
            return;
        }
        let mean_dx = sum_dx / k;
        let mean_y = sum_y / k;

        let mut num: i128 = 0;
        let mut den: i128 = 0;
        for s in self.samples.iter().flatten() {
            let dx = ((s.0 as i128) - anchor_x) - mean_dx;
            let dy = (s.1 as i128) - mean_y;
            num += dx * dy;
            den += dx * dx;
        }
        if den == 0 {
            return;
        }
        // Clamp slope to ±0.5 (way more than any realistic rate): protects
        // against numerical pathologies on degenerate data.
        let raw_q32 = (num << 32) / den;
        let max_q32 = 1i128 << 31;
        let clamped = raw_q32.clamp(-max_q32, max_q32);
        self.slope_q32 = clamped as i64;
        self.mean_x = (anchor_x + mean_dx) as i64;
        self.mean_y = mean_y as i64;
    }
}

/// Two-state α-β filter for tracking a slowly-drifting offset.
///
/// State: `(offset, rate)` where offset is in µs and rate is in PPB
/// (offset_µs per second of x). At each observation `(x, observed_offset)`:
///
/// 1. Predict: `offset_predicted = offset + rate * (x - last_x) * 1e-9`
/// 2. Innovation: `y = observed_offset - offset_predicted`
/// 3. Update offset: `offset = offset_predicted + y / α`
/// 4. Update rate:   `rate = rate + (y / dx_seconds) / β`
///
/// `α` filters per-sample noise on the offset (small α = fast tracking,
/// big α = heavy smoothing). `β` filters slope estimation noise (big β =
/// stable rate at the cost of slower learning). Choosing α≈16 and β≈512
/// gives a fast-tracking offset with a rate that converges over ~1 minute
/// of beacons.
///
/// All-integer; safe for `no_std`.
#[derive(Debug)]
pub struct AlphaBetaFilter {
    initialized: bool,
    /// Most recent x (in µs).
    last_x_us: i64,
    /// Tracked offset at `last_x_us`, scaled by `Q_OFFSET` for sub-µs precision.
    offset_q: i128,
    /// Tracked rate in (offset_µs * Q_RATE) per (x_µs).
    /// Equivalently: rate_ppm = rate_q / Q_RATE * 1e6 (since slope is dimensionless).
    rate_q: i128,
    /// Filter gains (α, β). Use higher values for more smoothing.
    alpha: u32,
    beta: u32,
}

impl AlphaBetaFilter {
    /// Q-format scale factors. offset_q = offset_us * 2^20; rate_q =
    /// rate_dimensionless * 2^32. So `(rate_q * dx_us) >> 12` lifts a rate
    /// into offset_q.
    const OFFSET_Q_SHIFT: u32 = 20;
    const RATE_Q_SHIFT: u32 = 32;
    const RATE_TO_OFFSET_SHIFT: u32 = Self::RATE_Q_SHIFT - Self::OFFSET_Q_SHIFT; // = 12

    pub const fn new(alpha: u32, beta: u32) -> Self {
        Self {
            initialized: false,
            last_x_us: 0,
            offset_q: 0,
            rate_q: 0,
            alpha,
            beta,
        }
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Tracked rate as parts-per-billion (signed).
    pub fn rate_ppb(&self) -> i64 {
        ((self.rate_q * 1_000_000_000) >> Self::RATE_Q_SHIFT) as i64
    }

    /// Tracked offset in µs at `last_x_us`.
    pub fn offset_us(&self) -> i64 {
        (self.offset_q >> Self::OFFSET_Q_SHIFT) as i64
    }

    pub fn observe(&mut self, x_us: i64, observed_offset_us: i64) {
        if !self.initialized {
            self.last_x_us = x_us;
            self.offset_q = (observed_offset_us as i128) << Self::OFFSET_Q_SHIFT;
            self.rate_q = 0;
            self.initialized = true;
            return;
        }
        let dx_us = (x_us as i128) - (self.last_x_us as i128);
        if dx_us <= 0 {
            // Out-of-order or duplicate; skip.
            return;
        }
        // predicted_offset_q = offset_q + (rate_q * dx_us) >> (RATE - OFFSET)
        let predicted_offset_q = self
            .offset_q
            .saturating_add((self.rate_q.saturating_mul(dx_us)) >> Self::RATE_TO_OFFSET_SHIFT);
        let observed_q = (observed_offset_us as i128) << Self::OFFSET_Q_SHIFT;
        let innovation_q = observed_q - predicted_offset_q;

        // offset += innovation / α
        self.offset_q = predicted_offset_q + innovation_q / (self.alpha as i128);
        // observed_rate_q = (innovation_q << RATE_TO_OFFSET) / dx_us
        // (so that observed_rate_q * dx_us >> RATE_TO_OFFSET ≈ innovation_q)
        // rate += observed_rate / β
        let observed_rate_q = (innovation_q << Self::RATE_TO_OFFSET_SHIFT) / dx_us;
        self.rate_q = self.rate_q.saturating_add(observed_rate_q / (self.beta as i128));

        self.last_x_us = x_us;
    }

    /// Predict offset at the given x. Returns `None` if the filter hasn't
    /// been seeded yet.
    pub fn predict(&self, x_us: i64) -> Option<i64> {
        if !self.initialized {
            return None;
        }
        let dx_us = (x_us as i128) - (self.last_x_us as i128);
        let predicted_q = self
            .offset_q
            .saturating_add((self.rate_q.saturating_mul(dx_us)) >> Self::RATE_TO_OFFSET_SHIFT);
        Some((predicted_q >> Self::OFFSET_Q_SHIFT) as i64)
    }
}

#[cfg(test)]
mod alpha_beta_tests {
    use super::*;

    #[test]
    fn ab_recovers_constant_offset() {
        let mut f = AlphaBetaFilter::new(16, 512);
        for i in 0..100i64 {
            f.observe(i * 100_000, 5_000); // y = 5 ms constant
        }
        let p = f.predict(100 * 100_000).unwrap();
        assert!((p - 5_000).abs() < 5, "p={}", p);
        assert!(f.rate_ppb().abs() < 100, "rate_ppb={}", f.rate_ppb());
    }

    #[test]
    fn ab_recovers_linear_drift() {
        // y = 5_000 + 10 ppm * x (x in µs)
        let mut f = AlphaBetaFilter::new(16, 256);
        for i in 0..2000i64 {
            let x = i * 100_000;
            let y = 5_000 + (10 * x) / 1_000_000;
            f.observe(x, y);
        }
        let predicted = f.predict(2001 * 100_000).unwrap();
        let expected = 5_000 + (10 * 2001 * 100_000) / 1_000_000;
        assert!(
            (predicted - expected).abs() < 50,
            "predicted={} expected={}",
            predicted,
            expected
        );
        // rate_ppb should be near 10_000 (10 ppm)
        let r = f.rate_ppb();
        assert!((r - 10_000).abs() < 1_000, "rate_ppb={}", r);
    }

    #[test]
    fn ab_filters_per_sample_noise() {
        // y = 0 + 200 µs noise
        let mut f = AlphaBetaFilter::new(32, 1024);
        let noise: [i64; 16] = [
            120, -180, 90, -200, 30, 150, -80, 110, -50, 200, -100, 60, -150, 90, -30, 80,
        ];
        for i in 0..1000i64 {
            f.observe(i * 100_000, noise[(i as usize) & 15]);
        }
        let p = f.predict(1000 * 100_000).unwrap();
        assert!(p.abs() < 30, "p={}", p);
    }
}

#[cfg(test)]
mod tracker_tests {
    use super::*;

    #[test]
    fn linear_tracker_recovers_constant_offset() {
        let mut t = LinearTracker::new();
        for i in 0..32 {
            t.observe(i * 100, 5_000); // x in µs, y = 5 ms constant
        }
        let p = t.predict(3_200).unwrap();
        assert!((p - 5_000).abs() < 10, "p={}", p);
    }

    #[test]
    fn linear_tracker_recovers_linear_drift() {
        // Simulate y = 5_000 + 10 ppm × x (x in µs → 10e-6 × x)
        let mut t = LinearTracker::new();
        for i in 0..256i64 {
            let x = i * 100_000; // 100 ms steps in µs
            let y = 5_000 + (10 * x) / 1_000_000; // 10 ppm
            t.observe(x, y);
        }
        let last_x = 255 * 100_000;
        let p = t.predict(last_x + 100_000).unwrap();
        let expected = 5_000 + (10 * (last_x + 100_000)) / 1_000_000;
        assert!(
            (p - expected).abs() < 5,
            "p={} expected={}",
            p,
            expected
        );
    }

    #[test]
    fn linear_tracker_filters_per_sample_noise() {
        // y = 0 + 0 ppm × x + 200 µs noise
        let mut t = LinearTracker::new();
        let noise: [i64; 16] = [
            120, -180, 90, -200, 30, 150, -80, 110, -50, 200, -100, 60, -150, 90, -30, 80,
        ];
        // Need 256 samples to fill window
        for i in 0..256i64 {
            let x = i * 100_000;
            let y = noise[(i as usize) & 15];
            t.observe(x, y);
        }
        let p = t.predict(255 * 100_000).unwrap();
        assert!(p.abs() < 50, "p={}", p);
    }
}

/// Integer square root for u128 (Newton iteration). Used for residual stddev.
fn isqrt_u128(n: u128) -> u128 {
    if n < 2 {
        return n;
    }
    let mut x = n;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let p = SyncPacket {
            magic: MAGIC,
            seq: 0xDEAD_BEEF,
            send_unix_ns: 1_700_000_000_123_456_789u128,
        };
        let bytes = p.to_bytes();
        let p2 = SyncPacket::parse(&bytes).unwrap();
        let (seq2, ns2) = ({ p2.seq }, { p2.send_unix_ns });
        let (seq1, ns1) = ({ p.seq }, { p.send_unix_ns });
        assert_eq!(seq2, seq1);
        assert_eq!(ns2, ns1);
    }

    #[test]
    fn find_in_frame_with_headers() {
        let p = SyncPacket {
            magic: MAGIC,
            seq: 7,
            send_unix_ns: 42,
        };
        let mut frame = vec![0u8; 60]; // simulated 802.11/IP/UDP headers
        frame.extend_from_slice(&p.to_bytes());
        let parsed = SyncPacket::find_in_frame(&frame).unwrap();
        let (seq, ns) = ({ parsed.seq }, { parsed.send_unix_ns });
        assert_eq!(seq, 7);
        assert_eq!(ns, 42);
    }

    #[test]
    fn find_in_frame_returns_none_when_absent() {
        let frame = [0u8; 100];
        assert!(SyncPacket::find_in_frame(&frame).is_none());
    }

    #[test]
    fn estimator_recovers_zero_drift() {
        // Server clock ticks 1000 ns per local µs (perfect match), with a
        // constant offset of 1_000_000 ns.
        let mut est = Estimator::new();
        for i in 0..16u64 {
            est.observe(Sample {
                server_ns: 1_000_000 + (i * 100_000) as u128 * 1_000,
                local_us: i * 100_000,
            });
        }
        let q = est.quality();
        assert!(q.n_samples >= MIN_SAMPLES as u16);
        assert!(q.drift_ppb.abs() < 1000, "drift_ppb = {}", q.drift_ppb);
        // Predict at a known local time.
        let predicted = est.synced_ns_at(500_000).unwrap();
        assert!((predicted as i128 - (1_000_000 + 500_000_000) as i128).abs() < 10_000);
    }

    #[test]
    fn estimator_recovers_positive_drift() {
        // Server clock runs 100ppm faster than local (drift_ppb = +100_000).
        let mut est = Estimator::new();
        for i in 0..16u64 {
            let local_us = i * 100_000;
            // server_ns = local_us * (1000 + 100_000/1_000_000) ns/µs
            //           = local_us * 1000.1 ns
            let server_ns = (local_us as u128) * 1_000 + (local_us as u128) / 10;
            est.observe(Sample {
                server_ns,
                local_us,
            });
        }
        let q = est.quality();
        assert!(
            (q.drift_ppb - 100_000).abs() < 5_000,
            "drift_ppb = {}",
            q.drift_ppb
        );
    }

    #[test]
    fn estimator_inverse_is_consistent() {
        let mut est = Estimator::new();
        for i in 0..8u64 {
            let local_us = i * 100_000;
            let server_ns = (local_us as u128) * 1_000 + 5_000_000;
            est.observe(Sample {
                server_ns,
                local_us,
            });
        }
        let target_server_ns = 250_000_000u128 + 5_000_000;
        let local_us = est.local_us_at(target_server_ns).unwrap();
        let predicted = est.synced_ns_at(local_us).unwrap();
        let err = (predicted as i128 - target_server_ns as i128).abs();
        assert!(err < 1_000, "round-trip error {} ns", err);
    }
}
