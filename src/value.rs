// Ported from orangeduck/Motion-Matching spring.h at commit 57b7250e0d34a4e456a34d47e24c2f05fdcc711e.
// Also ported from orangeduck/Spring-It-On common.h, extrapolation.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under MIT; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; glam storage and generic vector arithmetic.
use crate::convert::*;
use glam::{Vec2, Vec3, Vec3A, Vec4};
use std::ops::{Add, Mul, Sub};
/// A scalar or vector supporting componentwise spring arithmetic. Default must be zero.
pub trait SpringValue:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<f32, Output = Self> + Default
{
    /// Divide each component with scalar rounding. The default uses reciprocal multiplication.
    fn div_scalar(self, rhs: f32) -> Self {
        self * (1.0 / rhs)
    }
}
impl SpringValue for f32 {
    fn div_scalar(self, rhs: f32) -> Self {
        self / rhs
    }
}
impl SpringValue for Vec2 {
    fn div_scalar(self, rhs: f32) -> Self {
        Self::from_array(self.to_array().map(|x| x / rhs))
    }
}
impl SpringValue for Vec3 {
    fn div_scalar(self, rhs: f32) -> Self {
        Self::from_array(self.to_array().map(|x| x / rhs))
    }
}
impl SpringValue for Vec3A {
    fn div_scalar(self, rhs: f32) -> Self {
        Self::from_array(self.to_array().map(|x| x / rhs))
    }
}
impl SpringValue for Vec4 {
    fn div_scalar(self, rhs: f32) -> Self {
        Self::from_array(self.to_array().map(|x| x / rhs))
    }
}
/// Dampen a scalar or vector toward a goal.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damper_exact_vec<T: SpringValue>(x: T, g: T, halflife: f32, dt: f32, eps: f32) -> T {
    if dt <= 0.0 {
        return x;
    }
    let a = 1.0 - fast_negexp((0.69314718056 * dt) / (halflife + eps));
    x * (1.0 - a) + g * a
}
/// Critically damp a scalar or vector toward a fixed goal.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn simple_spring_damper_exact_vec<T: SpringValue>(
    x: &mut T,
    v: &mut T,
    x_goal: T,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j0 = *x - x_goal;
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = (j0 + j1 * dt) * eydt + x_goal;
    *v = (*v - j1 * y * dt) * eydt;
}
/// Critically damp a scalar or vector toward a moving goal.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn critical_spring_damper_exact_vec<T: SpringValue>(
    x: &mut T,
    v: &mut T,
    x_goal: T,
    v_goal: T,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let d = halflife_to_damping(halflife, DEFAULT_EPS);
    let c = x_goal + (v_goal * d).div_scalar((d * d) / 4.0);
    let y = d / 2.0;
    let j0 = *x - c;
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = (j0 + j1 * dt) * eydt + c;
    *v = (*v - j1 * y * dt) * eydt;
}
/// Critically damp a scalar or vector toward zero.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn decay_spring_damper_exact_vec<T: SpringValue>(x: &mut T, v: &mut T, halflife: f32, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j1 = *v + *x * y;
    let eydt = fast_negexp(y * dt);
    *x = (*x + j1 * dt) * eydt;
    *v = (*v - j1 * y * dt) * eydt;
}
/// Transfer offsets across a source change.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_transition_vec<T: SpringValue>(
    off_x: &mut T,
    off_v: &mut T,
    src_x: T,
    src_v: T,
    dst_x: T,
    dst_v: T,
) {
    *off_x = (src_x + *off_x) - dst_x;
    *off_v = (src_v + *off_v) - dst_v;
}
/// Decay and compose motion offsets.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_update_vec<T: SpringValue>(
    out_x: &mut T,
    out_v: &mut T,
    off_x: &mut T,
    off_v: &mut T,
    in_x: T,
    in_v: T,
    halflife: f32,
    dt: f32,
) {
    decay_spring_damper_exact_vec(off_x, off_v, halflife, dt);
    *out_x = in_x + *off_x;
    *out_v = in_v + *off_v;
}
/// Extrapolate a scalar or vector while damping velocity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn extrapolate_vec<T: SpringValue>(x: &mut T, v: &mut T, dt: f32, halflife: f32, eps: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = 0.69314718056 / (halflife + eps);
    *x = *x + (*v).div_scalar(y + eps) * (1.0 - fast_negexp(y * dt));
    *v = *v * fast_negexp(y * dt);
}
