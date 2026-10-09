// Ported from orangeduck/Spring-It-On deadblending.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::{convert::lerp, damper::*};
/// Capture the outgoing motion at a transition.
/// [Original background](https://theorangeduck.com/page/dead-blending).
pub fn dead_blending_transition(
    ext_x: &mut f32,
    ext_v: &mut f32,
    ext_t: &mut f32,
    src_x: f32,
    src_v: f32,
) {
    *ext_x = src_x;
    *ext_v = src_v;
    *ext_t = 0.0;
}
fn blend_alpha(t: f32) -> f32 {
    let x = t.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}
/// Extrapolate the source and blend toward the incoming motion.
/// [Original background](https://theorangeduck.com/page/dead-blending).
pub fn dead_blending_update(
    out_x: &mut f32,
    out_v: &mut f32,
    ext_x: &mut f32,
    ext_v: &mut f32,
    ext_t: &mut f32,
    in_x: f32,
    in_v: f32,
    blendtime: f32,
    dt: f32,
    eps: f32,
) {
    if *ext_t < blendtime {
        if dt > 0.0 {
            *ext_x += *ext_v * dt;
            *ext_t += dt;
        }
        let alpha = blend_alpha(*ext_t / blendtime.max(eps));
        *out_x = lerp(*ext_x, in_x, alpha);
        *out_v = lerp(*ext_v, in_v, alpha);
    } else {
        *out_x = in_x;
        *out_v = in_v;
        if dt > 0.0 {
            *ext_t = f32::MAX;
        }
    }
}
/// Dampen the outgoing velocity while extrapolating and blending.
/// [Original background](https://theorangeduck.com/page/dead-blending).
pub fn dead_blending_update_decay(
    out_x: &mut f32,
    out_v: &mut f32,
    ext_x: &mut f32,
    ext_v: &mut f32,
    ext_t: &mut f32,
    in_x: f32,
    in_v: f32,
    blendtime: f32,
    decay_halflife: f32,
    dt: f32,
    eps: f32,
) {
    if *ext_t < blendtime {
        *ext_v = damper_decay_exact(*ext_v, decay_halflife, dt, 1e-5);
    }
    dead_blending_update(
        out_x, out_v, ext_x, ext_v, ext_t, in_x, in_v, blendtime, dt, eps,
    );
}
/// State of a scalar dead blend.
#[derive(Default, Debug, Clone, Copy)]
pub struct DeadBlend {
    /// Extrapolated position.
    pub ext_x: f32,
    /// Extrapolated velocity.
    pub ext_v: f32,
    /// Elapsed transition time.
    pub ext_t: f32,
}
impl DeadBlend {
    /// Capture an outgoing motion.
    /// [Original background](https://theorangeduck.com/page/dead-blending).
    pub fn transition(&mut self, src_x: f32, src_v: f32) {
        dead_blending_transition(
            &mut self.ext_x,
            &mut self.ext_v,
            &mut self.ext_t,
            src_x,
            src_v,
        );
    }
    /// Return the blended position and velocity; uses the upstream epsilon of 1e-8.
    /// [Original background](https://theorangeduck.com/page/dead-blending).
    pub fn update(&mut self, in_x: f32, in_v: f32, blendtime: f32, dt: f32) -> (f32, f32) {
        let (mut x, mut v) = (0.0, 0.0);
        dead_blending_update(
            &mut x,
            &mut v,
            &mut self.ext_x,
            &mut self.ext_v,
            &mut self.ext_t,
            in_x,
            in_v,
            blendtime,
            dt,
            1e-8,
        );
        (x, v)
    }
}
