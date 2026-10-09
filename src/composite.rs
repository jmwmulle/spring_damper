// Ported from orangeduck/Spring-It-On doublespring.c, timedspring.c, velocityspring.c, resonance.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::{convert::*, spring::*};
/// Cascade two critically damped springs through an intermediate state.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn double_spring_damper_exact(
    x: &mut f32,
    v: &mut f32,
    xi: &mut f32,
    vi: &mut f32,
    x_goal: f32,
    halflife: f32,
    dt: f32,
) {
    simple_spring_damper_exact(xi, vi, x_goal, 0.5 * halflife, dt);
    simple_spring_damper_exact(x, v, *xi, 0.5 * halflife, dt);
}
/// Follow a goal using an intermediate state with an arrival time.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn timed_spring_damper_exact(
    x: &mut f32,
    v: &mut f32,
    xi: &mut f32,
    x_goal: f32,
    t_goal: f32,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let min_time = if t_goal > dt { t_goal } else { dt };
    let v_goal = (x_goal - *xi) / min_time;
    let t_goal_future = halflife_to_lag(halflife);
    let x_goal_future = if t_goal_future < t_goal {
        *xi + v_goal * t_goal_future
    } else {
        x_goal
    };
    simple_spring_damper_exact(x, v, x_goal_future, halflife, dt);
    *xi += v_goal * dt;
}
/// Follow an intermediate goal travelling at a prescribed speed. eps is retained from upstream, which does not use it.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn velocity_spring_damper_exact(
    x: &mut f32,
    v: &mut f32,
    xi: &mut f32,
    x_goal: f32,
    v_goal: f32,
    halflife: f32,
    dt: f32,
    eps: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let _ = eps;
    let x_diff = (if x_goal - *xi > 0.0 { 1.0 } else { -1.0 }) * v_goal;
    let t_goal_future = halflife_to_lag(halflife);
    let x_goal_future = if (x_goal - *xi).abs() > t_goal_future * v_goal {
        *xi + x_diff * t_goal_future
    } else {
        x_goal
    };
    simple_spring_damper_exact(x, v, x_goal_future, halflife, dt);
    *xi = if (x_goal - *xi).abs() > dt * v_goal {
        *xi + x_diff * dt
    } else {
        x_goal
    };
}
/// Kinetic plus potential energy of a scalar spring.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn spring_energy(x: f32, v: f32, frequency: f32, x_rest: f32, v_rest: f32, scale: f32) -> f32 {
    let s = frequency_to_stiffness(frequency);
    (squaref(scale * (v - v_rest)) + s * squaref(scale * (x - x_rest))) / 2.0
}
/// Resonant frequency. Returns NaN when the upstream radicand is negative.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn resonant_frequency(goal_frequency: f32, halflife: f32) -> f32 {
    let d = halflife_to_damping(halflife, DEFAULT_EPS);
    stiffness_to_frequency(frequency_to_stiffness(goal_frequency) - (d * d) / 4.0)
}
