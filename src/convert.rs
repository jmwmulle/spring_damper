// Ported from orangeduck/Spring-It-On common.h at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
/// Default regularization used by the upstream spring functions.
pub const DEFAULT_EPS: f32 = 1e-5;
/// Rational approximation of exp(-x), for nonnegative x.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn fast_negexp(x: f32) -> f32 {
    1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
}
/// Convert a settling halflife into a damping coefficient.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn halflife_to_damping(halflife: f32, eps: f32) -> f32 {
    (4.0 * 0.69314718056) / (halflife + eps)
}
/// Convert a damping coefficient into a settling halflife.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damping_to_halflife(damping: f32, eps: f32) -> f32 {
    (4.0 * 0.69314718056) / (damping + eps)
}
/// Convert cycles per second to spring stiffness.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn frequency_to_stiffness(frequency: f32) -> f32 {
    squaref((2.0 * std::f64::consts::PI * f64::from(frequency)) as f32)
}
/// Convert spring stiffness to cycles per second.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn stiffness_to_frequency(stiffness: f32) -> f32 {
    (f64::from(stiffness.sqrt()) / (2.0 * std::f64::consts::PI)) as f32
}
/// Halflife at the critical damping boundary for a frequency.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn critical_halflife(frequency: f32) -> f32 {
    damping_to_halflife(
        (frequency_to_stiffness(frequency) * 4.0).sqrt(),
        DEFAULT_EPS,
    )
}
/// Frequency at the critical damping boundary for a halflife.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn critical_frequency(halflife: f32) -> f32 {
    stiffness_to_frequency(squaref(halflife_to_damping(halflife, DEFAULT_EPS)) / 4.0)
}
/// Stiffness corresponding to a damping ratio and coefficient.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damping_ratio_to_stiffness(ratio: f32, damping: f32) -> f32 {
    squaref(damping / (ratio * 2.0))
}
/// Damping corresponding to a ratio and stiffness.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damping_ratio_to_damping(ratio: f32, stiffness: f32) -> f32 {
    ratio * 2.0 * stiffness.sqrt()
}
/// Convert halflife to the upstream lag estimate.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn halflife_to_lag(halflife: f32) -> f32 {
    halflife / 0.69314718056
}
/// Convert lag to halflife.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn lag_to_halflife(lag: f32) -> f32 {
    lag * 0.69314718056
}
pub(crate) fn squaref(x: f32) -> f32 {
    x * x
}
pub(crate) fn lerp(x: f32, y: f32, a: f32) -> f32 {
    (1.0 - a) * x + a * y
}
pub(crate) fn fast_atan(x: f32) -> f32 {
    let z = x.abs();
    let w = if z > 1.0 { 1.0 / z } else { z };
    let y = ((std::f64::consts::PI / 4.0) * f64::from(w)
        - f64::from(w * (w - 1.0) * (0.2447 + 0.0663 * w))) as f32;
    (if z > 1.0 {
        (std::f64::consts::PI / 2.0 - f64::from(y)) as f32
    } else {
        y
    })
    .copysign(x)
}
