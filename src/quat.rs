// Ported from orangeduck/Motion-Matching spring.h, quat.h at commit 57b7250e0d34a4e456a34d47e24c2f05fdcc711e.
// Copyright (c) 2021 Daniel Holden
// Licensed under MIT; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; glam storage and generic vector arithmetic.
use crate::convert::*;
fn mul(q: Quat, p: Quat) -> Quat {
    Quat::from_xyzw(
        p.w * q.x + p.x * q.w - p.y * q.z + p.z * q.y,
        p.w * q.y + p.x * q.z + p.y * q.w - p.z * q.x,
        p.w * q.z - p.x * q.y + p.y * q.x + p.z * q.w,
        p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
    )
}
fn divide(q: Quat, s: f32) -> Quat {
    Quat::from_xyzw(q.x / s, q.y / s, q.z / s, q.w / s)
}

use glam::{Quat, Vec3};
/// Select the representative quaternion with nonnegative w.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn quat_abs(x: Quat) -> Quat {
    if x.w < 0.0 { -x } else { x }
}
/// Convert a unit quaternion to its rotation vector. Sign is preserved as upstream.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn quat_to_scaled_angle_axis(q: Quat, eps: f32) -> Vec3 {
    let xyz = Vec3::new(q.x, q.y, q.z);
    let length = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if length < eps {
        xyz * 2.0
    } else {
        Vec3::new(xyz.x / length, xyz.y / length, xyz.z / length)
            * q.w.clamp(-1.0, 1.0).acos()
            * 2.0
    }
}
/// Construct a quaternion from a rotation vector.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn quat_from_scaled_angle_axis(v: Vec3, eps: f32) -> Quat {
    let v = v / 2.0;
    let halfangle = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    if halfangle < eps {
        let q = Quat::from_xyzw(v.x, v.y, v.z, 1.0);
        divide(q, (1.0 + v.x * v.x + v.y * v.y + v.z * v.z).sqrt() + eps)
    } else {
        let s = halfangle.sin() / halfangle;
        Quat::from_xyzw(s * v.x, s * v.y, s * v.z, halfangle.cos())
    }
}
fn shortest_approx(q: Quat, mut p: Quat, alpha: f32) -> Quat {
    let ca = q.w * p.w + q.x * p.x + q.y * p.y + q.z * p.z;
    if ca < 0.0 {
        p = -p;
    }
    let d = ca.abs();
    let a = 1.0904 + d * (-3.2452 + d * (3.55645 - d * 1.43519));
    let b = 0.848013 + d * (-1.06021 + d * 0.215638);
    let k = a * (alpha - 0.5) * (alpha - 0.5) + b;
    let oalpha = alpha + alpha * (alpha - 0.5) * (alpha - 1.0) * k;
    let r = q * (1.0 - oalpha) + p * oalpha;
    divide(
        r,
        (r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z).sqrt() + 1e-8,
    )
}
/// Dampen a rotation using the upstream shortest-path interpolation approximation.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damper_exact_quat(x: Quat, g: Quat, halflife: f32, dt: f32, eps: f32) -> Quat {
    if dt <= 0.0 {
        return x;
    }
    shortest_approx(
        x,
        g,
        1.0 - fast_negexp((0.69314718056 * dt) / (halflife + eps)),
    )
}
/// Apply a fraction of a rotation adjustment to the identity quaternion.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn damp_adjustment_exact_quat(g: Quat, halflife: f32, dt: f32, eps: f32) -> Quat {
    damper_exact_quat(Quat::IDENTITY, g, halflife, dt, eps)
}
/// Critically damp a rotation; angular velocity is expressed in world axes.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn simple_spring_damper_exact_quat(
    x: &mut Quat,
    v: &mut Vec3,
    x_goal: Quat,
    halflife: f32,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j0 = quat_to_scaled_angle_axis(quat_abs(mul(*x, -x_goal.conjugate())), 1e-8);
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = mul(
        quat_from_scaled_angle_axis((j0 + j1 * dt) * eydt, 1e-8),
        x_goal,
    );
    *v = (*v - j1 * y * dt) * eydt;
}
/// Critically damp a rotation toward identity.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn decay_spring_damper_exact_quat(x: &mut Quat, v: &mut Vec3, halflife: f32, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    let y = halflife_to_damping(halflife, DEFAULT_EPS) / 2.0;
    let j0 = quat_to_scaled_angle_axis(*x, 1e-8);
    let j1 = *v + j0 * y;
    let eydt = fast_negexp(y * dt);
    *x = quat_from_scaled_angle_axis((j0 + j1 * dt) * eydt, 1e-8);
    *v = (*v - j1 * y * dt) * eydt;
}
/// Transfer rotation and angular-velocity offsets across a source change.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_transition_quat(
    off_x: &mut Quat,
    off_v: &mut Vec3,
    src_x: Quat,
    src_v: Vec3,
    dst_x: Quat,
    dst_v: Vec3,
) {
    *off_x = quat_abs(mul(mul(*off_x, src_x), -dst_x.conjugate()));
    *off_v = (*off_v + src_v) - dst_v;
}
/// Decay and compose rotation offsets, rotating input angular velocity through the offset.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_update_quat(
    out_x: &mut Quat,
    out_v: &mut Vec3,
    off_x: &mut Quat,
    off_v: &mut Vec3,
    in_x: Quat,
    in_v: Vec3,
    halflife: f32,
    dt: f32,
) {
    decay_spring_damper_exact_quat(off_x, off_v, halflife, dt);
    *out_x = mul(*off_x, in_x);
    *out_v = *off_v + *off_x * in_v;
}
