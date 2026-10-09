// Ported from orangeduck/Spring-It-On tracking.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::{convert::*, damper::*, spring::*};
/// Tracking spring update.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    a_goal: f32,
    x_gain: f32,
    v_gain: f32,
    a_gain: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = lerp(*v, *v + a_goal * dt, a_gain);
    *v = lerp(*v, v_goal, v_gain);
    *v = lerp(*v, (x_goal - *x) / dt, x_gain);
    *x += dt * *v;
}
/// Tracking spring update no acceleration.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_acceleration(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    x_gain: f32,
    v_gain: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = lerp(*v, v_goal, v_gain);
    *v = lerp(*v, (x_goal - *x) / dt, x_gain);
    *x += dt * *v;
}
/// Tracking spring update no velocity acceleration.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_velocity_acceleration(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    x_gain: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = lerp(*v, (x_goal - *x) / dt, x_gain);
    *x += dt * *v;
}
/// Tracking spring update improved.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_improved(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    a_goal: f32,
    x_halflife: f32,
    v_halflife: f32,
    a_halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = damper_exact(*v, *v + a_goal * dt, a_halflife, dt, DEFAULT_EPS);
    *v = damper_exact(*v, v_goal, v_halflife, dt, DEFAULT_EPS);
    *v = damper_exact(*v, (x_goal - *x) / dt, x_halflife, dt, DEFAULT_EPS);
    *x += dt * *v;
}
/// Tracking spring update no acceleration improved.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_acceleration_improved(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    x_halflife: f32,
    v_halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = damper_exact(*v, v_goal, v_halflife, dt, DEFAULT_EPS);
    *v = damper_exact(*v, (x_goal - *x) / dt, x_halflife, dt, DEFAULT_EPS);
    *x += dt * *v;
}
/// Tracking spring update no velocity acceleration improved.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_velocity_acceleration_improved(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    x_halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    *v = damper_exact(*v, (x_goal - *x) / dt, x_halflife, dt, DEFAULT_EPS);
    *x += dt * *v;
}
/// Tracking spring update exact.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_exact(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    a_goal: f32,
    x_gain: f32,
    v_gain: f32,
    a_gain: f32,
    dt: f32,
    gain_dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    let t0 = (1.0 - v_gain) * (1.0 - x_gain);
    let t1 = a_gain * (1.0 - v_gain) * (1.0 - x_gain);
    let t2 = (v_gain * (1.0 - x_gain)) / gain_dt;
    let t3 = x_gain / (gain_dt * gain_dt);

    let stiffness = t3;
    let damping = (1.0 - t0) / gain_dt;
    let spring_x_goal = x_goal;
    let spring_v_goal = (t2 * v_goal + t1 * a_goal) / ((1.0 - t0) / gain_dt);

    spring_damper_exact_stiffness_damping(
        x,
        v,
        spring_x_goal,
        spring_v_goal,
        stiffness,
        damping,
        dt,
        DEFAULT_EPS,
    );
}
/// Tracking spring update no acceleration exact.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_acceleration_exact(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    v_goal: f32,
    x_gain: f32,
    v_gain: f32,
    dt: f32,
    gain_dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    let t0 = (1.0 - v_gain) * (1.0 - x_gain);
    let t2 = (v_gain * (1.0 - x_gain)) / gain_dt;
    let t3 = x_gain / (gain_dt * gain_dt);

    let stiffness = t3;
    let damping = (1.0 - t0) / gain_dt;
    let spring_x_goal = x_goal;
    let spring_v_goal = t2 * v_goal / ((1.0 - t0) / gain_dt);

    spring_damper_exact_stiffness_damping(
        x,
        v,
        spring_x_goal,
        spring_v_goal,
        stiffness,
        damping,
        dt,
        DEFAULT_EPS,
    );
}
/// Tracking spring update no velocity acceleration exact.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_spring_update_no_velocity_acceleration_exact(
    x: &mut f32,
    v: &mut f32,
    x_goal: f32,
    x_gain: f32,
    dt: f32,
    gain_dt: f32,
) {
    if dt <= 0.0 {
        return;
    }

    let t0 = 1.0 - x_gain;
    let t3 = x_gain / (gain_dt * gain_dt);

    let stiffness = t3;
    let damping = (1.0 - t0) / gain_dt;
    let spring_x_goal = x_goal;
    let spring_v_goal = 0.0;

    spring_damper_exact_stiffness_damping(
        x,
        v,
        spring_x_goal,
        spring_v_goal,
        stiffness,
        damping,
        dt,
        DEFAULT_EPS,
    );
}
/// Tracking target acceleration.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_target_acceleration(x_next: f32, x_curr: f32, x_prev: f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return 0.0;
    }

    (((x_next - x_curr) / dt) - ((x_curr - x_prev) / dt)) / dt
}
/// Tracking target velocity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn tracking_target_velocity(x_next: f32, x_curr: f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return 0.0;
    }

    (x_next - x_curr) / dt
}
