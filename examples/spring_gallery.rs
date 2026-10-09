//! Six spring trajectories rendered as colored plots. Space restarts; Escape closes.
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use spring_damper::*;
#[derive(Resource, Default)]
struct Gallery {
    rows: Vec<Vec<Vec3>>,
    elapsed: f32,
    captured: bool,
}
fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.015, 0.02, 0.03)))
        .init_resource::<Gallery>()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Spring gallery — Space restarts".into(),
                resolution: (1100, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(Update, (draw, capture))
        .run();
}
fn setup(mut commands: Commands, mut gallery: ResMut<Gallery>) {
    commands.spawn(Camera2d);
    for kind in 0..6 {
        let (mut x, mut v, mut xi, mut vi) = (0.0, 0.0, 0.0, 0.0);
        let mut points = vec![];
        for i in 0..300 {
            let dt = 1.0 / 60.0;
            let goal = if i < 100 {
                1.0
            } else if i < 200 {
                -0.6
            } else {
                0.4
            };
            match kind {
                0 => spring_damper_exact(&mut x, &mut v, goal, 0.0, 2.0, 0.4, dt, DEFAULT_EPS),
                1 => simple_spring_damper_exact(&mut x, &mut v, goal, 0.4, dt),
                2 => double_spring_damper_exact(&mut x, &mut v, &mut xi, &mut vi, goal, 0.4, dt),
                3 => timed_spring_damper_exact(&mut x, &mut v, &mut xi, goal, 0.8, 0.4, dt),
                4 => velocity_spring_damper_exact(
                    &mut x,
                    &mut v,
                    &mut xi,
                    goal,
                    1.0,
                    0.4,
                    dt,
                    DEFAULT_EPS,
                ),
                _ => {
                    x = damper_exact(x, goal, 0.4, dt, DEFAULT_EPS);
                }
            }
            points.push(Vec3::new(
                i as f32 * 3.2 - 480.0,
                x * 35.0 + 290.0 - kind as f32 * 115.0,
                0.0,
            ));
        }
        gallery.rows.push(points);
    }
}
fn draw(
    mut gizmos: Gizmos,
    mut gallery: ResMut<Gallery>,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut exit: MessageWriter<AppExit>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
    if keys.just_pressed(KeyCode::Space) {
        gallery.elapsed = 0.0;
    }
    gallery.elapsed += time.delta_secs();
    let colors = [
        Color::srgb(0.3, 0.8, 1.0),
        Color::srgb(0.4, 1.0, 0.7),
        Color::srgb(1.0, 0.7, 0.3),
        Color::srgb(0.8, 0.5, 1.0),
        Color::srgb(1.0, 0.4, 0.6),
        Color::srgb(0.8, 0.9, 0.5),
    ];
    for (i, row) in gallery.rows.iter().enumerate() {
        gizmos.line_2d(
            Vec2::new(-480.0, 290.0 - i as f32 * 115.0),
            Vec2::new(480.0, 290.0 - i as f32 * 115.0),
            Color::srgb(0.12, 0.17, 0.22),
        );
        for pair in row[..((gallery.elapsed * 60.0) as usize + 1).min(row.len())].windows(2) {
            gizmos.line_2d(pair[0].truncate(), pair[1].truncate(), colors[i]);
        }
    }
}
fn capture(mut commands: Commands, mut gallery: ResMut<Gallery>, mut exit: MessageWriter<AppExit>) {
    if gallery.elapsed > 6.0
        && !gallery.captured
        && let Ok(path) = std::env::var("SPRING_SCREENSHOT")
    {
        gallery.captured = true;
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
    if gallery.elapsed > 15.0 && std::env::var_os("SPRING_SCREENSHOT").is_some() {
        exit.write(AppExit::Success);
    }
}
