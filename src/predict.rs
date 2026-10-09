// Ported from orangeduck/Spring-It-On controller.c, extrapolation.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::convert::*;
/// Extrapolate position while exponentially damping velocity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn extrapolate(x: &mut f32, v: &mut f32, dt: f32, halflife: f32, eps: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = 0.69314718056 / (halflife + eps);
    *x += (*v / (y + eps)) * (1.0 - fast_negexp(y * dt));
    *v *= fast_negexp(y * dt);
}
/// Integrate a position whose velocity springs toward a target.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
#[allow(
    clippy::assign_op_pattern,
    reason = "Preserve upstream floating-point grouping"
)]
pub fn spring_character_update(
    x: &mut f32,
    v: &mut f32,
    a: &mut f32,
    v_goal: f32,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j0 = *v - v_goal;
    let j1 = *a + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = eydt * ((-j1 / (y * y)) + ((-j0 - j1 * dt) / y))
        + (j1 / (y * y))
        + j0 / y
        + v_goal * dt
        + *x;
    *v = eydt * (j0 + j1 * dt) + v_goal;
    *a = eydt * (*a - j1 * y * dt);
}
/// Sample the initial state at i*dt, including the initial state at index zero.
/// Panics if the output slices differ in length. This follows the direct upstream predictor, not repeated incremental integration.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn spring_character_predict(
    px: &mut [f32],
    pv: &mut [f32],
    pa: &mut [f32],
    x: f32,
    v: f32,
    a: f32,
    v_goal: f32,
    halflife: f32,
    dt: f32,
) {
    assert_eq!(px.len(), pv.len());
    assert_eq!(px.len(), pa.len());
    for i in 0..px.len() {
        px[i] = x;
        pv[i] = v;
        pa[i] = a;
        spring_character_update(
            &mut px[i],
            &mut pv[i],
            &mut pa[i],
            v_goal,
            halflife,
            i as f32 * dt,
        );
    }
}
