// Ported from orangeduck/Spring-It-On inertialization.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; explicit defaults and mutable references.
use crate::spring::*;
/// Transfer the current offsets across a change of source motion.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_transition(
    off_x: &mut f32,
    off_v: &mut f32,
    src_x: f32,
    src_v: f32,
    dst_x: f32,
    dst_v: f32,
) {
    *off_x = (src_x + *off_x) - dst_x;
    *off_v = (src_v + *off_v) - dst_v;
}
/// Decay offsets and compose the current input motion with them.
/// [Original background](https://theorangeduck.com/page/spring-roll-call).
pub fn inertialize_update(
    out_x: &mut f32,
    out_v: &mut f32,
    off_x: &mut f32,
    off_v: &mut f32,
    in_x: f32,
    in_v: f32,
    halflife: f32,
    dt: f32,
) {
    decay_spring_damper_exact(off_x, off_v, halflife, dt);
    *out_x = in_x + *off_x;
    *out_v = in_v + *off_v;
}
/// Persistent offsets for scalar inertialization.
#[derive(Default, Debug, Clone, Copy)]
pub struct Inertializer {
    /// Position offset.
    pub off_x: f32,
    /// Velocity offset.
    pub off_v: f32,
}
impl Inertializer {
    /// Change the source motion while preserving the accumulated offsets.
    /// [Original background](https://theorangeduck.com/page/spring-roll-call).
    pub fn transition(&mut self, src_x: f32, src_v: f32, dst_x: f32, dst_v: f32) {
        inertialize_transition(&mut self.off_x, &mut self.off_v, src_x, src_v, dst_x, dst_v);
    }
    /// Return the current output position and velocity.
    /// [Original background](https://theorangeduck.com/page/spring-roll-call).
    pub fn update(&mut self, in_x: f32, in_v: f32, halflife: f32, dt: f32) -> (f32, f32) {
        let (mut x, mut v) = (0.0, 0.0);
        inertialize_update(
            &mut x,
            &mut v,
            &mut self.off_x,
            &mut self.off_v,
            in_x,
            in_v,
            halflife,
            dt,
        );
        (x, v)
    }
}
