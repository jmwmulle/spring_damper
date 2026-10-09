//! Wireframe cubes chase a moving goal. Space changes halflife; Escape closes.
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use spring_damper::*;
#[derive(Resource, Default)]
struct State {
    elapsed: f32,
    slow: bool,
    captured: bool,
}
fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.015, 0.02, 0.03)))
        .init_resource::<State>()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Transform springs — Space changes halflife".into(),
                    resolution: (1100, 800).into(),
                    ..default()
                }),
                ..default()
            }),
            SpringPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, animate)
        .run();
}
fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Transform::from_xyz(0.0, 7.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    for i in 0..5 {
        commands.spawn((
            Transform::from_xyz(-3.0 + i as f32 * 1.5, 0.0, 0.0),
            TranslationSpring {
                goal: Vec3::ZERO,
                velocity: Vec3::ZERO,
                halflife: 0.1 + i as f32 * 0.15,
            },
            RotationSpring {
                goal: Quat::IDENTITY,
                angular_velocity: Vec3::ZERO,
                halflife: 0.3,
            },
            ScaleSpring {
                goal: Vec3::ONE,
                velocity: Vec3::ZERO,
                halflife: 0.3,
            },
        ));
    }
}
fn animate(
    mut commands: Commands,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<State>,
    mut q: Query<(
        &Transform,
        &mut TranslationSpring,
        &mut RotationSpring,
        &mut ScaleSpring,
    )>,
    mut gizmos: Gizmos,
    mut exit: MessageWriter<AppExit>,
) {
    state.elapsed += time.delta_secs();
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
    if keys.just_pressed(KeyCode::Space) {
        state.slow = !state.slow;
    }
    let t = state.elapsed;
    let goal = Vec3::new(t.sin() * 3.0, (t * 0.7).cos(), 0.0);
    gizmos.sphere(
        Isometry3d::from_translation(goal),
        0.2,
        Color::srgb(1.0, 0.7, 0.3),
    );
    for (i, (transform, mut translation, mut rotation, mut scale)) in q.iter_mut().enumerate() {
        translation.goal = goal + Vec3::Z * i as f32 * 0.8;
        translation.halflife = if state.slow {
            0.7
        } else {
            0.1 + i as f32 * 0.1
        };
        rotation.goal = Quat::from_rotation_y(t) * Quat::from_rotation_z(0.3);
        scale.goal = Vec3::splat(0.6 + 0.15 * (t * 1.5).sin());
        gizmos.cube(*transform, Color::srgb(0.3, 0.7 + i as f32 * 0.05, 1.0));
    }
    if t > 3.0
        && !state.captured
        && let Ok(path) = std::env::var("SPRING_SCREENSHOT")
    {
        state.captured = true;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path))
            .observe(
                |_: On<bevy::render::view::screenshot::ScreenshotCaptured>,
                 mut exit: MessageWriter<AppExit>| {
                    exit.write(AppExit::Success);
                },
            );
    }
    if t > 15.0 && std::env::var_os("SPRING_SCREENSHOT").is_some() {
        exit.write(AppExit::Success);
    }
}
