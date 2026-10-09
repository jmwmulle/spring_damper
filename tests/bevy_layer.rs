#![cfg(feature = "bevy")]
use bevy_app::App;
use bevy_transform::{
    TransformPlugin,
    components::{GlobalTransform, Transform},
};
use glam::Vec3;
use spring_damper::*;
fn app() -> App {
    let mut a = App::new();
    a.add_plugins((TransformPlugin, SpringPlugin));
    a.insert_resource(SpringClock::Manual);
    a.insert_resource(SpringDelta(1.0 / 60.0));
    a
}
#[test]
fn translation_spring_reaches_goal() {
    let mut a = app();
    let e = a
        .world_mut()
        .spawn((
            Transform::default(),
            TranslationSpring {
                goal: Vec3::ONE,
                velocity: Vec3::ZERO,
                halflife: 0.3,
            },
        ))
        .id();
    for _ in 0..360 {
        a.update();
    }
    assert!((a.world().get::<Transform>(e).unwrap().translation - Vec3::ONE).length() < 1e-3);
}
#[test]
fn manual_clock_with_zero_delta_freezes() {
    let mut a = app();
    a.insert_resource(SpringDelta(0.0));
    let e = a
        .world_mut()
        .spawn((
            Transform::default(),
            TranslationSpring {
                goal: Vec3::ONE,
                velocity: Vec3::ONE,
                halflife: 0.3,
            },
        ))
        .id();
    for _ in 0..100 {
        a.update();
    }
    assert_eq!(
        a.world().get::<Transform>(e).unwrap().translation,
        Vec3::ZERO
    );
    assert_eq!(
        a.world().get::<TranslationSpring>(e).unwrap().velocity,
        Vec3::ONE
    );
}
#[test]
fn springs_run_before_propagation() {
    let mut a = app();
    let e = a
        .world_mut()
        .spawn((
            Transform::default(),
            TranslationSpring {
                goal: Vec3::ONE,
                velocity: Vec3::ZERO,
                halflife: 0.3,
            },
        ))
        .id();
    a.update();
    let t = a.world().get::<Transform>(e).unwrap().translation;
    assert!(t.x > 0.0);
    assert_eq!(
        a.world().get::<GlobalTransform>(e).unwrap().translation(),
        t
    );
}
#[test]
fn generic_scalar_and_rotation_scale_are_updated() {
    let mut a = app();
    let e = a
        .world_mut()
        .spawn((
            Transform::default(),
            RotationSpring {
                goal: glam::Quat::from_rotation_y(1.0),
                angular_velocity: Vec3::ZERO,
                halflife: 0.3,
            },
            ScaleSpring {
                goal: Vec3::splat(2.0),
                velocity: Vec3::ZERO,
                halflife: 0.3,
            },
            Spring {
                value: 0.0f32,
                velocity: 0.0,
                goal: 1.0,
                halflife: 0.3,
            },
        ))
        .id();
    a.update();
    assert!(a.world().get::<Spring<f32>>(e).unwrap().value > 0.0);
    let t = a.world().get::<Transform>(e).unwrap();
    assert!(t.scale.x > 1.0);
    assert!(t.rotation.y > 0.0);
}
