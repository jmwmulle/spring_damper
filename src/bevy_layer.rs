// Native Bevy adapter for the MIT-derived spring functions; no upstream code in this adapter.
use crate::{SpringValue, simple_spring_damper_exact_quat, simple_spring_damper_exact_vec};
use bevy_app::{AnimationSystems, App, Plugin, PostUpdate};
use bevy_ecs::prelude::*;
use bevy_reflect::Reflect;
use bevy_time::{Time, Virtual};
use bevy_transform::{TransformSystems, components::Transform};
use glam::{Quat, Vec2, Vec3, Vec3A, Vec4};
/// Spring update clock; defaults to Bevy virtual time.
#[derive(Resource, Reflect, Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpringClock {
    /// Read Bevy's virtual clock.
    #[default]
    Virtual,
    /// Read the application's explicit delta.
    Manual,
}
/// Application-supplied seconds per update when the clock is Manual.
#[derive(Resource, Reflect, Default, Debug, Clone, Copy)]
pub struct SpringDelta(pub f32);
/// Ordered spring update set, after animation and before transform propagation.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SpringSystems;
/// A translation goal and persistent velocity in the transform's parent space.
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct TranslationSpring {
    /// Target translation.
    pub goal: Vec3,
    /// Persistent velocity.
    pub velocity: Vec3,
    /// Settling halflife in seconds.
    pub halflife: f32,
}
/// A rotation goal and persistent angular velocity in parent axes.
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct RotationSpring {
    /// Target rotation.
    pub goal: Quat,
    /// Persistent angular velocity.
    pub angular_velocity: Vec3,
    /// Settling halflife in seconds.
    pub halflife: f32,
}
/// A scale goal and persistent scale velocity.
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct ScaleSpring {
    /// Target scale.
    pub goal: Vec3,
    /// Persistent velocity.
    pub velocity: Vec3,
    /// Settling halflife in seconds.
    pub halflife: f32,
}
/// A spring value for application-defined uses. Built-in scalar and glam vectors are updated by the plugin.
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct Spring<T: SpringValue + Send + Sync + 'static> {
    /// Current value.
    pub value: T,
    /// Persistent velocity.
    pub velocity: T,
    /// Target value.
    pub goal: T,
    /// Settling halflife in seconds.
    pub halflife: f32,
}
/// Install transform springs and generic springs for f32 and glam vectors.
pub struct SpringPlugin;
impl Plugin for SpringPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SpringClock>()
            .init_resource::<SpringDelta>()
            .register_type::<SpringClock>()
            .register_type::<SpringDelta>()
            .register_type::<TranslationSpring>()
            .register_type::<RotationSpring>()
            .register_type::<ScaleSpring>()
            .register_type::<Spring<f32>>()
            .register_type::<Spring<Vec2>>()
            .register_type::<Spring<Vec3>>()
            .register_type::<Spring<Vec3A>>()
            .register_type::<Spring<Vec4>>()
            .configure_sets(
                PostUpdate,
                SpringSystems
                    .after(AnimationSystems)
                    .before(TransformSystems::Propagate),
            )
            .add_systems(
                PostUpdate,
                (
                    update_transforms,
                    update_value::<f32>,
                    update_value::<Vec2>,
                    update_value::<Vec3>,
                    update_value::<Vec3A>,
                    update_value::<Vec4>,
                )
                    .in_set(SpringSystems),
            );
    }
}
/// Register updates for an additional application-defined SpringValue type.
/// Its reflection registration, if needed, is left to the application.
pub fn add_spring_type<T: SpringValue + Send + Sync + 'static>(app: &mut App) {
    app.add_systems(PostUpdate, update_value::<T>.in_set(SpringSystems));
}
fn delta(clock: &SpringClock, manual: &SpringDelta, time: Option<&Time<Virtual>>) -> f32 {
    let dt = match clock {
        SpringClock::Manual => manual.0,
        SpringClock::Virtual => time.map_or(0.0, Time::delta_secs),
    };
    if dt.is_finite() { dt.max(0.0) } else { 0.0 }
}
type TransformSprings<'w, 's> = Query<
    'w,
    's,
    (
        &'static mut Transform,
        Option<&'static mut TranslationSpring>,
        Option<&'static mut RotationSpring>,
        Option<&'static mut ScaleSpring>,
    ),
    Or<(
        With<TranslationSpring>,
        With<RotationSpring>,
        With<ScaleSpring>,
    )>,
>;
fn update_transforms(
    clock: Res<SpringClock>,
    manual: Res<SpringDelta>,
    time: Option<Res<Time<Virtual>>>,
    mut query: TransformSprings,
) {
    let dt = delta(&clock, &manual, time.as_deref());
    if dt <= 0.0 {
        return;
    }
    for (mut transform, translation, rotation, scale) in &mut query {
        if let Some(mut s) = translation {
            let goal = s.goal;
            let h = s.halflife;
            simple_spring_damper_exact_vec(
                &mut transform.translation,
                &mut s.velocity,
                goal,
                h,
                dt,
            );
        }
        if let Some(mut s) = rotation {
            let goal = s.goal;
            let h = s.halflife;
            simple_spring_damper_exact_quat(
                &mut transform.rotation,
                &mut s.angular_velocity,
                goal,
                h,
                dt,
            );
        }
        if let Some(mut s) = scale {
            let goal = s.goal;
            let h = s.halflife;
            simple_spring_damper_exact_vec(&mut transform.scale, &mut s.velocity, goal, h, dt);
        }
    }
}
fn update_value<T: SpringValue + Send + Sync + 'static>(
    clock: Res<SpringClock>,
    manual: Res<SpringDelta>,
    time: Option<Res<Time<Virtual>>>,
    mut query: Query<&mut Spring<T>>,
) {
    let dt = delta(&clock, &manual, time.as_deref());
    if dt <= 0.0 {
        return;
    }
    for mut spring in &mut query {
        let s = &mut *spring;
        simple_spring_damper_exact_vec(&mut s.value, &mut s.velocity, s.goal, s.halflife, dt);
    }
}
