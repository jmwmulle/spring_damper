// Ported from orangeduck/Spring-It-On common.h at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::convert::{fast_negexp, lerp};
/// Dampen a value toward a goal. Nonpositive dt freezes the value.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damper_exact(x: f32, g: f32, halflife: f32, dt: f32, eps: f32) -> f32 {
    if dt <= 0.0 {
        return x;
    }
    lerp(
        x,
        g,
        1.0 - fast_negexp((0.69314718056 * dt) / (halflife + eps)),
    )
}
/// Dampen a value toward zero.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damper_decay_exact(x: f32, halflife: f32, dt: f32, eps: f32) -> f32 {
    if dt <= 0.0 {
        return x;
    }
    x * fast_negexp((0.69314718056 * dt) / (halflife + eps))
}
