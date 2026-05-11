//! ESP32 MCPWM-CAP based hardware GPIO timestamping.
//!
//! esp-hal 1.0 doesn't expose a high-level wrapper for the MCPWM Capture
//! sub-module yet, so we drive it directly via the esp32 PAC + GPIO-matrix
//! register writes. The CAP timer is a 32-bit free-running counter clocked
//! at 1 MHz (APB / 80) — captures latch the timer in hardware on a GPIO
//! edge, so jitter from software ISR dispatch (~µs to ms) is removed
//! entirely.
//!
//! Conversion to `embassy_time::Instant` is done via a one-time calibration
//! at startup: read both clocks at the same software moment, store the
//! constant offset, then `instant_at_edge_us = cap_ticks_at_edge +
//! cap_to_instant_offset`. Both clocks tick at the same nominal rate (1 µs)
//! because they share the chip's crystal.

use portable_atomic::{AtomicI64, AtomicU64, Ordering};

use embassy_time::Instant;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use esp_println::println;
use static_cell::StaticCell;

/// MCPWM0 base address (ESP32 only). Source: ESP32 TRM "Memory Map".
const MCPWM0_BASE: usize = 0x3FF5_E000;
/// GPIO matrix input-select base, indexed by *peripheral input signal*.
/// Each entry routes a peripheral input from a GPIO. Source: ESP32 TRM
/// "IO_MUX and GPIO Matrix".
const GPIO_FUNC_IN_SEL_CFG_BASE: usize = 0x3FF4_4130;

/// Peripheral input signal indices (ESP32). Source: ESP32 TRM "I/O signal
/// mapping". esp-metadata-generated knows these but the enum is `doc(hidden)`.
const PWM0_CAP0: u32 = 109;
const PWM0_CAP1: u32 = 110;

/// MCPWM0 register offsets (subset). Source: esp32-0.40.2 PAC layout —
/// the CAP block sits well past the operator/generator clusters at 0xE8+,
/// **not** at the 0x6C/0x74/… offsets used on later esp32sX variants.
const REG_CLK_CFG: usize = 0x000;
const REG_CAP_TIMER_CFG: usize = 0x0E8;
const REG_CAP_CH0_CFG: usize = 0x0F0;
const REG_CAP_CH1_CFG: usize = 0x0F4;
const REG_CAP_CH2_CFG: usize = 0x0F8;
const REG_CAP_CH0: usize = 0x0FC;
const REG_CAP_CH1: usize = 0x100;
const REG_CAP_CH2: usize = 0x104;
const REG_CAP_STATUS: usize = 0x108;

/// Anchors for the cap-tick → `embassy_time::Instant` and cap-tick →
/// `mac_tsf_us` conversions. The CAP timer ticks faster than 1 µs
/// (empirically ~80 MHz on ESP32 — the docs claim PWM_CLK is divisor-
/// controlled but the CAP submodule appears to bypass it). `init`
/// measures two SW captures spaced ~4 s apart and stores both slopes as
/// Q32 fixed-point, so subsequent conversions are exact integer math.
///
/// `instant_us(t)  ≈ ((cap(t) - cap_anchor) * us_per_tick_instant_q32) >> 32 + instant_anchor_us`
/// `mac_tsf_us(t)  ≈ ((cap(t) - cap_anchor) * us_per_tick_tsf_q32)     >> 32 + tsf_anchor_us`
///
/// Sentinel `i64::MIN` in `*_ANCHOR_US` means "not yet calibrated".
/// The TSF anchor is separate because `mac_tsf_us()` and `Instant::now()`
/// are independent counters with their own zero points, even though both
/// are crystal-locked.
static CAP_ANCHOR_TICKS: AtomicU64 = AtomicU64::new(0);
static INSTANT_ANCHOR_US: AtomicI64 = AtomicI64::new(i64::MIN);
static TSF_ANCHOR_US: AtomicI64 = AtomicI64::new(i64::MIN);
/// Microseconds per CAP tick, in Q32 fixed-point. e.g. for an 80 MHz CAP
/// timer this is `(1.0 / 80) * 2^32` ≈ 53_687_091.
static US_PER_TICK_INSTANT_Q32: AtomicU64 = AtomicU64::new(0);
static US_PER_TICK_TSF_Q32: AtomicU64 = AtomicU64::new(0);

/// Captured cap-tick value of the last cross-board RX edge on D5.
/// `u64::MAX` ⇒ no capture yet. The hardware register is 32-bit; we extend
/// to u64 by tracking wraparound via [`maybe_unwrap`].
static LAST_CAP_CH0: AtomicU64 = AtomicU64::new(u64::MAX);
/// Last raw 32-bit value seen on CH0 (for wrap detection).
static LAST_CAP_CH0_RAW: AtomicU64 = AtomicU64::new(0);
/// Same for CH1 / D34 self-loop input.
static LAST_CAP_CH1: AtomicU64 = AtomicU64::new(u64::MAX);
static LAST_CAP_CH1_RAW: AtomicU64 = AtomicU64::new(0);

/// Counter of fresh CH0 captures.
static CAP_CH0_HITS: AtomicU64 = AtomicU64::new(0);
/// Counter of fresh CH1 captures.
static CAP_CH1_HITS: AtomicU64 = AtomicU64::new(0);

#[inline]
fn read_reg(off: usize) -> u32 {
    unsafe { core::ptr::read_volatile((MCPWM0_BASE + off) as *const u32) }
}
#[inline]
fn write_reg(off: usize, v: u32) {
    unsafe { core::ptr::write_volatile((MCPWM0_BASE + off) as *mut u32, v) }
}

/// Route peripheral input signal `signal` to come from `gpio_num` via the
/// GPIO matrix. Equivalent to `gpio_matrix_in()` in ESP-IDF.
unsafe fn route_gpio_to_input_signal(gpio_num: u32, signal: u32) {
    let addr = GPIO_FUNC_IN_SEL_CFG_BASE + (signal as usize) * 4;
    // Bits: [5:0] source GPIO, [6] invert, [7] route via matrix.
    let val: u32 = (gpio_num & 0x3F) | (1 << 7);
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

static MCPWM_HOLD: StaticCell<McPwm<'static, MCPWM0<'static>>> = StaticCell::new();

/// Initialise MCPWM0's CAP submodule.
///
/// `gpio_d5` and `gpio_d34` are the GPIO numbers for the cross-board input
/// (CH0) and self-loop input (CH1) respectively. Pass `None` to skip.
///
/// `read_mac_tsf_us` is an optional callback to read the WiFi MAC TSF — if
/// provided, we also calibrate `cap_ticks → mac_tsf_us` so callers can use
/// the high-precision `synced_ns_at_own_tsf` path that bypasses the noisy
/// `Instant↔TSF` EMA filter. Pass `None` to skip TSF calibration (the
/// `cap_ticks_to_instant_us` path still works).
///
/// We instantiate `McPwm` only to make esp-hal enable the peripheral
/// clock + lift reset for us (PeripheralGuard isn't `pub`). The instance
/// is then leaked into a `StaticCell` so it never `Drop`s. After that we
/// drive the CAP-specific registers via raw MMIO since esp-hal 1.0
/// doesn't yet wrap the CAP submodule.
pub fn init(
    mcpwm0: MCPWM0<'static>,
    gpio_d5: Option<u32>,
    gpio_d34: Option<u32>,
    read_mac_tsf_us: Option<fn() -> u64>,
) {
    // We instantiate `McPwm` purely to enable the peripheral clock + lift
    // its reset (esp-hal's `PeripheralGuard` isn't `pub`). Whatever
    // CLK_PRESCALE we pick here is irrelevant — empirically the CAP timer
    // bypasses CLK_PRESCALE on ESP32 and runs at the source PWM clock
    // (~80 MHz). We measure the actual rate at runtime via two timed SW
    // captures below.
    let clk_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(1)).unwrap();
    let mcpwm = McPwm::new(mcpwm0, clk_cfg);
    MCPWM_HOLD.init(mcpwm);

    // Enable CAP timer; no sync.
    write_reg(REG_CAP_TIMER_CFG, 1);

    // Configure CAP_CH0: en=1 (bit 0), mode=10 (neg edge, bits 2:1),
    // prescale=0 (bits 10:3), in_invert=0, sw=0.
    //
    // NOTE on mode encoding: empirically `mode=01` triggers on the
    // **falling** edge on ESP32 (1 ms-wide TX pulses produced 1 ms-late
    // captures with mode=01), so we use `mode=10` to actually capture
    // the rising edge. The ESP32 TRM section 17.3.11.2 docs say `01 = pos
    // edge` but the PAC follows a different convention.
    if gpio_d5.is_some() {
        write_reg(REG_CAP_CH0_CFG, 0b0_0001 | (0b10 << 1));
    }
    if gpio_d34.is_some() {
        write_reg(REG_CAP_CH1_CFG, 0b0_0001 | (0b10 << 1));
    }

    // Route GPIOs to PWM CAP signals via the GPIO matrix.
    if let Some(g) = gpio_d5 {
        unsafe { route_gpio_to_input_signal(g, PWM0_CAP0) };
    }
    if let Some(g) = gpio_d34 {
        unsafe { route_gpio_to_input_signal(g, PWM0_CAP1) };
    }

    // Enable CH2 (used only for SW captures during calibration + debug
    // dumps). The CAP-CH SW bit is edge-sensitive — keeping `en=1` set
    // means each `write(sw=1)` reliably latches a fresh timer value.
    write_reg(REG_CAP_CH2_CFG, 0b0_0001 | (0b01 << 1));

    // Calibrate `cap_ticks ↔ instant_us` empirically. Take two paired
    // (cap, instant) SW captures separated by a 4 s `Instant`-busy-wait
    // and use the slope (Δinstant / Δcap) as µs-per-tick. The pairing in
    // `sw_capture_ch2_paired` keeps the two samples within a few CPU
    // cycles of each other so the slope reflects the true clock ratio
    // rather than measurement skew.
    //
    // Window length matters: each `Instant::now()` quantises to 1 µs, so
    // the slope error is roughly `2 / window_us`. 4 s window → ~0.5 ppm
    // slope error → ~30 µs accumulated error after 60 s of operation.
    // Shorter windows blow that up rapidly (1 s → ~200 µs at 60 s,
    // 10 ms → ~17 ms).
    //
    // The CAP timer wraps every ~53 s (32-bit at 80 MHz) — we need both
    // samples in the same wrap window, so longer than ~50 s won't work
    // without per-call unwrap logic.
    let (cap0, instant0, tsf0) = sw_capture_ch2_with_clocks(read_mac_tsf_us);
    let target = instant0 + 4_000_000;
    while Instant::now().as_micros() < target {
        core::hint::spin_loop();
    }
    let (cap1, instant1, _tsf1) = sw_capture_ch2_with_clocks(read_mac_tsf_us);

    let d_instant = instant1.wrapping_sub(instant0);
    let d_cap = cap1.wrapping_sub(cap0);
    // Q32 µs-per-tick for the Instant clock — measured empirically because
    // we don't know its exact rate ratio to APB (the embassy timer source
    // is configurable).
    let us_per_tick_instant_q32: u64 = if d_cap == 0 {
        1u64 << 32
    } else {
        (((d_instant as u128) << 32) / (d_cap as u128)) as u64
    };
    CAP_ANCHOR_TICKS.store(cap0, Ordering::Relaxed);
    INSTANT_ANCHOR_US.store(instant0 as i64, Ordering::Relaxed);
    US_PER_TICK_INSTANT_Q32.store(us_per_tick_instant_q32, Ordering::Relaxed);

    println!(
        "mcpwm_cap: calibrated cap0={} instant0={} d_cap={} d_instant={} us_per_tick_instant_q32={}",
        cap0, instant0, d_cap, d_instant, us_per_tick_instant_q32,
    );

    // For the cap → MAC-TSF mapping, hard-code the rate at exactly 80:1
    // (CAP runs at PWM_CLK = APB = 80 MHz, MAC TSF at 1 MHz, both
    // crystal-locked). Empirically measuring the slope is too noisy:
    // `mac_tsf_us()` has ~µs–120µs jitter per call, so a 4 s window's
    // slope is uncertain by ~30 ppm → ~1.5 ms drift after 50 s of
    // operation. Crystal mismatch is bounded by chip spec at <30 ppm,
    // and any constant offset gets absorbed into `tsf_anchor`. The cap
    // tick rate could in principle be other than APB on some CPU clock
    // configs, but our `Config::default().with_cpu_clock(CpuClock::max)`
    // path locks APB to 80 MHz on ESP32.
    if let Some(t0) = tsf0 {
        TSF_ANCHOR_US.store(t0 as i64, Ordering::Relaxed);
        // 1 µs / 80 ticks * 2^32 ≈ 53_687_091.2  (rounds to 53_687_091).
        let q32: u64 = ((1u128 << 32) / 80) as u64;
        US_PER_TICK_TSF_Q32.store(q32, Ordering::Relaxed);
        println!("mcpwm_cap: tsf0={} us_per_tick_tsf_q32={} (hard-coded 80:1)", t0, q32);
    }
}

/// Packed `(high u32 << 32) | (last raw cap u32)`. Updated by every SW
/// capture (frequent: ~20 Hz from beacons + pulses). Edge captures
/// reconcile their raw 32-bit reads against this packed state to recover
/// the matching 64-bit unwrapped value.
static LAST_SW_CAP_PACKED: AtomicU64 = AtomicU64::new(0);

/// Reconcile a raw 32-bit cap-register read against the *expected* cap
/// value derived from `Instant::now()` and the calibrated cap rate.
///
/// Why use Instant rather than the most recent SW-capture state:
///   - SW captures pause if WiFi callbacks pause (e.g. during a
///     reconnection storm). With LAST_SW_CAP_PACKED stale, edges can be
///     misclassified into the wrong wrap window.
///   - Instant is 1 µs and never wraps in our run lengths (i64 µs).
///   - Both Instant and CAP are crystal-derived, so the calibration
///     never drifts beyond ~µs over a run.
///
/// Algorithm: predict the full 64-bit cap value for "now", then pick the
/// high-bits that make `((high << 32) | raw)` closest to the prediction.
fn unwrap_cap_raw(raw: u32) -> u64 {
    let inst_anchor = INSTANT_ANCHOR_US.load(Ordering::Relaxed);
    let cap_anchor = CAP_ANCHOR_TICKS.load(Ordering::Relaxed);
    let q32 = US_PER_TICK_INSTANT_Q32.load(Ordering::Relaxed);
    if inst_anchor == i64::MIN || q32 == 0 {
        // Calibration not done yet — fall back to "same window" guess.
        return raw as u64;
    }
    let now_us = Instant::now().as_micros() as i128;
    // predicted_cap_ticks = cap_anchor + (now_us - inst_anchor) * (1 / us_per_tick)
    // = cap_anchor + (now_us - inst_anchor) << 32 / us_per_tick_q32
    let d_us = now_us - inst_anchor as i128;
    let predicted_64 = (cap_anchor as i128) + (d_us << 32) / (q32 as i128);
    if predicted_64 < 0 {
        return raw as u64;
    }
    // Closest high s.t. ((high << 32) | raw) ≈ predicted_64.
    let predicted = predicted_64 as u64;
    let high_pred = (predicted >> 32) as u32;
    // Three candidates: high_pred - 1, high_pred, high_pred + 1.
    let candidates = [
        ((high_pred.wrapping_sub(1) as u64) << 32) | (raw as u64),
        ((high_pred as u64) << 32) | (raw as u64),
        ((high_pred.wrapping_add(1) as u64) << 32) | (raw as u64),
    ];
    let mut best = candidates[0];
    let mut best_dist = predicted.abs_diff(candidates[0]);
    for &c in &candidates[1..] {
        let d = predicted.abs_diff(c);
        if d < best_dist {
            best = c;
            best_dist = d;
        }
    }
    best
}

/// Trigger a SW capture on CH2 and return the unwrapped 64-bit tick
/// value. Updates [`LAST_SW_CAP_PACKED`] so subsequent edge captures can
/// reconcile against this anchor.
fn sw_capture_ch2_unwrapped() -> u64 {
    write_reg(REG_CAP_CH2_CFG, 0b0_0001 | (0b01 << 1) | (1 << 12));
    let raw = read_reg(REG_CAP_CH2) as u32;
    // CAS loop: beacon hook + pulser SW captures + reporter polls run
    // from different async contexts.
    loop {
        let packed = LAST_SW_CAP_PACKED.load(Ordering::Relaxed);
        let prev_raw = packed as u32;
        let prev_high = (packed >> 32) as u32;
        let new_high = if raw < prev_raw {
            prev_high.wrapping_add(1)
        } else {
            prev_high
        };
        let new_packed = ((new_high as u64) << 32) | (raw as u64);
        if LAST_SW_CAP_PACKED
            .compare_exchange(packed, new_packed, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            return new_packed;
        }
    }
}

/// Trigger a software capture on CH2 and sample `(cap, instant, [tsf])` all
/// referring to the same physical moment as closely as possible. The cap-
/// timer value is latched in hardware on the SW=1 write; the `Instant::now`
/// and optional `mac_tsf_us` reads happen immediately after so they
/// reference the same instant within ~a few CPU cycles + the mac_tsf
/// call's typical ~µs latency.
///
/// If `read_mac_tsf_us` is `None`, returns `(cap, instant, None)`.
/// Assumes CH2 was enabled (en=1) by `init`.
fn sw_capture_ch2_with_clocks(read_mac_tsf_us: Option<fn() -> u64>) -> (u64, u64, Option<u64>) {
    let cap = sw_capture_ch2_unwrapped();
    let inst = Instant::now().as_micros();
    let tsf = read_mac_tsf_us.map(|f| f());
    (cap, inst, tsf)
}

/// Trigger a software capture and return only the cap-tick value (used by
/// debug dumps where the matched Instant isn't needed).
fn sw_capture_ch2() -> u64 {
    sw_capture_ch2_unwrapped()
}

/// Public wrapper around [`sw_capture_ch2`] for use as a `clock_sync_client`
/// beacon hook. Must match the `fn() -> u64` signature exactly.
pub fn sw_capture_ch2_pub() -> u64 {
    sw_capture_ch2()
}

#[inline]
fn maybe_unwrap(prev_raw: u32, new_raw: u32, prev_high: u64) -> u64 {
    let high = if new_raw < prev_raw {
        prev_high.wrapping_add(1u64 << 32)
    } else {
        prev_high
    };
    high | (new_raw as u64)
}

/// Poll CH0 for a newly captured edge. Returns the unwrapped 64-bit
/// cap-tick value if a new capture has occurred since last poll, or
/// `None`. Wrap reconciliation goes through [`unwrap_cap_raw`] using
/// the SW-capture-maintained packed state — *not* a per-channel
/// independent high-bits counter — so anchor & event readings always
/// agree on the high bits even after wraps.
pub fn poll_ch0() -> Option<u64> {
    let raw = read_reg(REG_CAP_CH0);
    let prev_raw = LAST_CAP_CH0_RAW.load(Ordering::Relaxed) as u32;
    if raw == prev_raw {
        return None;
    }
    let unwrapped = unwrap_cap_raw(raw);
    LAST_CAP_CH0.store(unwrapped, Ordering::Relaxed);
    LAST_CAP_CH0_RAW.store(raw as u64, Ordering::Relaxed);
    CAP_CH0_HITS.fetch_add(1, Ordering::Relaxed);
    Some(unwrapped)
}

/// Same for CH1.
pub fn poll_ch1() -> Option<u64> {
    let raw = read_reg(REG_CAP_CH1);
    let prev_raw = LAST_CAP_CH1_RAW.load(Ordering::Relaxed) as u32;
    if raw == prev_raw {
        return None;
    }
    let unwrapped = unwrap_cap_raw(raw);
    LAST_CAP_CH1.store(unwrapped, Ordering::Relaxed);
    LAST_CAP_CH1_RAW.store(raw as u64, Ordering::Relaxed);
    CAP_CH1_HITS.fetch_add(1, Ordering::Relaxed);
    Some(unwrapped)
}

/// Convert a captured CAP-timer tick value to an `embassy_time::Instant`
/// microsecond reading. Uses the empirically-calibrated tick rate
/// (typically ~80 MHz on ESP32). `None` if calibration hasn't been done.
pub fn cap_ticks_to_instant_us(cap_ticks: u64) -> Option<u64> {
    let inst_anchor = INSTANT_ANCHOR_US.load(Ordering::Relaxed);
    if inst_anchor == i64::MIN {
        return None;
    }
    let cap_anchor = CAP_ANCHOR_TICKS.load(Ordering::Relaxed);
    let q32 = US_PER_TICK_INSTANT_Q32.load(Ordering::Relaxed);
    let d_ticks = cap_ticks.wrapping_sub(cap_anchor) as i128;
    let d_us = (d_ticks * q32 as i128) >> 32;
    let inst = inst_anchor as i128 + d_us;
    if inst < 0 || inst > u64::MAX as i128 {
        None
    } else {
        Some(inst as u64)
    }
}

/// Convert a captured CAP-timer tick value to a WiFi MAC TSF microsecond
/// reading. Goes through the same calibration path as
/// [`cap_ticks_to_instant_us`] but using the TSF anchor + slope. `None`
/// if `init` was called without `read_mac_tsf_us`.
///
/// **Use this in preference to `cap_ticks_to_instant_us` for sync-critical
/// timestamps:** the resulting TSF value can be passed to
/// `ClockSyncClient::synced_ns_at_own_tsf`, which bypasses the noisy
/// `Instant↔TSF` α-β filter and gives much tighter cross-board agreement.
pub fn cap_ticks_to_mac_tsf_us(cap_ticks: u64) -> Option<u64> {
    let tsf_anchor = TSF_ANCHOR_US.load(Ordering::Relaxed);
    if tsf_anchor == i64::MIN {
        return None;
    }
    let cap_anchor = CAP_ANCHOR_TICKS.load(Ordering::Relaxed);
    let q32 = US_PER_TICK_TSF_Q32.load(Ordering::Relaxed);
    let d_ticks = cap_ticks.wrapping_sub(cap_anchor) as i128;
    let d_us = (d_ticks * q32 as i128) >> 32;
    let tsf = tsf_anchor as i128 + d_us;
    if tsf < 0 || tsf > u64::MAX as i128 {
        None
    } else {
        Some(tsf as u64)
    }
}

pub fn ch0_hits() -> u64 {
    CAP_CH0_HITS.load(Ordering::Relaxed)
}
pub fn ch1_hits() -> u64 {
    CAP_CH1_HITS.load(Ordering::Relaxed)
}

/// Diagnostic snapshot of MCPWM CAP-related registers + GPIO-matrix entries.
pub fn debug_dump() -> [u32; 10] {
    let cap_now = sw_capture_ch2();
    [
        read_reg(REG_CLK_CFG),
        read_reg(REG_CAP_TIMER_CFG),
        read_reg(REG_CAP_CH0_CFG),
        read_reg(REG_CAP_CH1_CFG),
        read_reg(REG_CAP_CH0),
        read_reg(REG_CAP_CH1),
        unsafe { core::ptr::read_volatile((GPIO_FUNC_IN_SEL_CFG_BASE + 109 * 4) as *const u32) },
        unsafe { core::ptr::read_volatile((GPIO_FUNC_IN_SEL_CFG_BASE + 110 * 4) as *const u32) },
        cap_now as u32,
        read_reg(REG_CAP_STATUS),
    ]
}
