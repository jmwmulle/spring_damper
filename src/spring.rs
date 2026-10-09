// Ported from orangeduck/Spring-It-On common.h at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::convert::*;
/// Advance a spring using stiffness and damping coefficients.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn spring_damper_exact_stiffness_damping(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    stiffness: f32,
    damping: f32,
    dt: f32,
    eps: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let g = x_goal;
    let q = v_goal;
    let s = stiffness;
    let d = damping;
    let c = g + (d * q) / (s + eps);
    let y = d / 2.0;
    if (s - (d * d) / 4.0).abs() < eps {
        let j0 = *x - c;
        let j1 = *v + j0 * y;
        let eydt = fast_negexp(y * dt);
        *x = j0 * eydt + dt * j1 * eydt + c;
        *v = -y * j0 * eydt - y * dt * j1 * eydt + j1 * eydt;
    } else if s - (d * d) / 4.0 > 0.0 {
        let w = (s - (d * d) / 4.0).sqrt();
        let mut j = (squaref(*v + y * (*x - c)) / (w * w + eps) + squaref(*x - c)).sqrt();
        let p = fast_atan((*v + (*x - c) * y) / (-(*x - c) * w + eps));
        j = if *x - c > 0.0 { j } else { -j };
        let eydt = fast_negexp(y * dt);
        *x = j * eydt * (w * dt + p).cos() + c;
        *v = -y * j * eydt * (w * dt + p).cos() - w * j * eydt * (w * dt + p).sin();
    } else if s - (d * d) / 4.0 < 0.0 {
        let y0 = (d + (d * d - 4.0 * s).sqrt()) / 2.0;
        let y1 = (d - (d * d - 4.0 * s).sqrt()) / 2.0;
        let j1 = (c * y0 - *x * y0 - *v) / (y1 - y0);
        let j0 = *x - j1 - c;
        let ey0dt = fast_negexp(y0 * dt);
        let ey1dt = fast_negexp(y1 * dt);
        *x = j0 * ey0dt + j1 * ey1dt + c;
        *v = -y0 * j0 * ey0dt - y1 * j1 * ey1dt;
    }
}
/// Advance a spring using frequency and halflife.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn spring_damper_exact(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    frequency: f32,
    halflife: f32,
    dt: f32,
    eps: f32,
) {
    spring_damper_exact_stiffness_damping(
        x,
        v,
        x_goal,
        v_goal,
        frequency_to_stiffness(frequency),
        halflife_to_damping(halflife, DEFAULT_EPS),
        dt,
        eps,
    );
}
/// Advance a spring using damping ratio and halflife.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn spring_damper_exact_ratio(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    damping_ratio: f32,
    halflife: f32,
    dt: f32,
    eps: f32,
) {
    let d = halflife_to_damping(halflife, DEFAULT_EPS);
    spring_damper_exact_stiffness_damping(
        x,
        v,
        x_goal,
        v_goal,
        damping_ratio_to_stiffness(damping_ratio, d),
        d,
        dt,
        eps,
    );
}
/// Advance a critically damped spring toward a moving goal.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn critical_spring_damper_exact(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let d = halflife_to_damping(halflife, DEFAULT_EPS);
    let c = x_goal + (d * v_goal) / ((d * d) / 4.0);
    let y = d / 2.0;
    let j0 = *x - c;
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = eydt * (j0 + j1 * dt) + c;
    *v = eydt * (*v - j1 * y * dt);
}
/// Advance a critically damped spring toward a fixed goal.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn simple_spring_damper_exact(x: &mut f32, v: &mut f32, x_goal: f32, halflife: f32, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j0 = *x - x_goal;
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = eydt * (j0 + j1 * dt) + x_goal;
    *v = eydt * (*v - j1 * y * dt);
}
/// Advance a critically damped spring toward zero.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn decay_spring_damper_exact(x: &mut f32, v: &mut f32, halflife: f32, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j1 = *v + *x * y;
    let eydt = fast_negexp(y * dt);
    *x = eydt * (*x + j1 * dt);
    *v = eydt * (*v - j1 * y * dt);
}
