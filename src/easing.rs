// Ported from orangeduck/Spring-It-On inertialeasing.c, cubiceasing.c, interpolation.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::convert::lerp;
/// Scaled cubic smoothstep, retaining the upstream negative-time section.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn smoothstep(t: f32, s: f32) -> f32 {
    s * if t > 1.0 {
        1.0
    } else {
        3.0 * t * t - 2.0 * t * t * t
    }
}
/// Derivative of scaled smoothstep.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn smoothstep_dt(t: f32, s: f32) -> f32 {
    s * if t > 1.0 { 0.0 } else { 6.0 * t - 6.0 * t * t }
}
/// Fit smoothstep time and scale to a target displacement and velocity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn smoothstep_solve(t: &mut f32, s: &mut f32, x: f32, v: f32, overshoot: f32, eps: f32) {
    if v.abs() < 1e-8 {
        *t = 0.0;
        *s = x;
        return;
    }
    if x < 0.0 {
        smoothstep_solve(t, s, -x, -v, 0.05, 1e-8);
        *s = -*s;
        return;
    }
    let rad = ((6.0 * x - v) * (6.0 * x - v) + 8.0 * v * v)
        .max(eps)
        .sqrt();
    let t0 = (v - 6.0 * x + rad) / (4.0 * v);
    let t1 = (v - 6.0 * x - rad) / (4.0 * v);
    *t = if -0.5 > t1 && t1 > -(0.5 + overshoot) {
        t1
    } else {
        t0
    };
    let vt = smoothstep_dt(*t, 1.0);
    *s = if vt.abs() < eps { 0.0 } else { v / vt };
}
/// Cubic trajectory from zero to a goal with a starting velocity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn cubic(t: f32, v: f32, g: f32) -> f32 {
    if t > 1.0 {
        g
    } else {
        (3.0 * t * t - 2.0 * t * t * t) * g + (t * t * t - 2.0 * t * t + t) * v
    }
}
/// Derivative of the cubic trajectory.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn cubic_dt(t: f32, v: f32, g: f32) -> f32 {
    if t > 1.0 {
        0.0
    } else {
        (6.0 * t - 6.0 * t * t) * g + (3.0 * t * t - 4.0 * t + 1.0) * v
    }
}
/// Interpolate control points using normalized time. Retains upstream velocity convention.
/// Panics if the control-point slice is empty. Negative time is clamped to zero for safe indexing.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn piecewise_interpolation(x: &mut f32, v: &mut f32, t: f32, pnts: &[f32]) {
    assert!(!pnts.is_empty());
    let t = t.max(0.0) * (pnts.len() - 1) as f32;
    let i0 = (t.floor() as usize).min(pnts.len() - 1);
    let i1 = (i0 + 1).min(pnts.len() - 1);
    *x = lerp(pnts[i0], pnts[i1], t % 1.0);
    *v = (pnts[i0] - pnts[i1]) / pnts.len() as f32;
}
