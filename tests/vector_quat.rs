#![cfg(feature = "glam")]
use glam::{Quat, Vec3};
use spring_damper::*;
fn q(v: &serde_json::Value) -> Quat {
    let a: Vec<f32> = v
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_f64().unwrap() as f32)
        .collect();
    Quat::from_xyzw(a[0], a[1], a[2], a[3])
}
fn v(v: &serde_json::Value) -> Vec3 {
    let a: Vec<f32> = v
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_f64().unwrap() as f32)
        .collect();
    Vec3::new(a[0], a[1], a[2])
}
fn checkq(a: Quat, b: Quat) {
    let a = a.normalize();
    let mut b = b.normalize();
    if a.dot(b) < 0.0 {
        b = -b;
    }
    let chord = (a - b).length();
    assert!(
        chord * 2.0 <= 1e-5,
        "{a:?} vs {b:?}, angular error ~{}",
        chord * 2.0
    );
}
#[test]
fn quat_forms_match_real_upstream_goldens() {
    let rows: serde_json::Value = serde_json::from_reader(flate2::read::GzDecoder::new(
        &include_bytes!("goldens/quat.json.gz")[..],
    ))
    .unwrap();
    for row in rows.as_array().unwrap() {
        let family = row[0].as_u64().unwrap();
        let dt = row[1].as_f64().unwrap() as f32;
        let mut x = q(&row[2]);
        let mut vel = v(&row[3]);
        let goal = q(&row[4]);
        let (mut out, mut ov) = (x, vel);
        let gv = Vec3::new(0.3, -0.1, 0.2);
        match family {
            0 => {
                out = damper_exact_quat(x, goal, 0.3, dt, DEFAULT_EPS);
                x = out;
            }
            1 => out = damp_adjustment_exact_quat(goal, 0.3, dt, DEFAULT_EPS),
            2 => {
                simple_spring_damper_exact_quat(&mut x, &mut vel, goal, 0.3, dt);
                out = x;
                ov = vel;
            }
            3 => {
                decay_spring_damper_exact_quat(&mut x, &mut vel, 0.3, dt);
                out = x;
                ov = vel;
            }
            4 => {
                inertialize_transition_quat(&mut x, &mut vel, goal, gv, Quat::IDENTITY, Vec3::ZERO);
                out = x;
                ov = vel;
            }
            5 => inertialize_update_quat(&mut out, &mut ov, &mut x, &mut vel, goal, gv, 0.3, dt),
            6 => {
                out = quat_from_scaled_angle_axis(quat_to_scaled_angle_axis(goal, 1e-8), 1e-8);
                ov = quat_to_scaled_angle_axis(goal, 1e-8);
            }
            _ => unreachable!(),
        }
        checkq(out, q(&row[5]));
        assert!((ov - v(&row[6])).length() < 1e-5, "family {family}");
        if family == 5 {
            checkq(x, q(&row[7]));
            assert!((vel - v(&row[8])).length() < 1e-5, "family {family}");
        }
    }
}
#[test]
fn vector_forms_equal_componentwise_scalar() {
    let (mut x, mut v) = (Vec3::new(0.1, -0.7, 0.4), Vec3::new(0.2, 0.3, -0.8));
    let (mut sx, mut sv) = (x.to_array(), v.to_array());
    for i in 0..300 {
        let g = Vec3::new(i as f32 * 0.01, -0.4, 0.7);
        simple_spring_damper_exact_vec(&mut x, &mut v, g, 0.3, 1.0 / 60.0);
        for j in 0..3 {
            simple_spring_damper_exact(&mut sx[j], &mut sv[j], g[j], 0.3, 1.0 / 60.0);
        }
        assert_eq!(x.to_array(), sx);
        assert_eq!(v.to_array(), sv);
    }
}
#[test]
fn quat_goal_of_negated_current_takes_short_path() {
    let (mut x, mut v) = (Quat::from_rotation_y(0.7), Vec3::ZERO);
    let start = x;
    simple_spring_damper_exact_quat(&mut x, &mut v, -start, 0.3, 1.0 / 60.0);
    checkq(x, start);
    assert_eq!(v, Vec3::ZERO);
}
#[test]
fn quat_outputs_stay_normalized() {
    let (mut x, mut v) = (Quat::IDENTITY, Vec3::ZERO);
    for i in 0..10000 {
        let g = Quat::from_rotation_y(i as f32 * 0.007) * Quat::from_rotation_x(0.3);
        simple_spring_damper_exact_quat(&mut x, &mut v, g, 0.3, 1.0 / 60.0);
        assert!((x.length() - 1.0).abs() <= 1e-5);
    }
}
#[test]
fn every_generic_math_form_matches_componentwise_scalar() {
    for kind in 0..5 {
        let (mut x, mut vel) = (Vec3::new(0.1, -0.4, 0.9), Vec3::new(0.3, -0.2, 0.7));
        let (mut sx, mut sv) = (x.to_array(), vel.to_array());
        let goal = Vec3::new(1.0, -0.7, 0.2);
        let goal_v = Vec3::new(0.2, 0.3, -0.1);
        for _ in 0..300 {
            let dt = 1.0 / 60.0;
            match kind {
                0 => x = damper_exact_vec(x, goal, 0.3, dt, DEFAULT_EPS),
                1 => critical_spring_damper_exact_vec(&mut x, &mut vel, goal, goal_v, 0.3, dt),
                2 => decay_spring_damper_exact_vec(&mut x, &mut vel, 0.3, dt),
                3 => extrapolate_vec(&mut x, &mut vel, dt, 0.3, DEFAULT_EPS),
                _ => inertialize_transition_vec(
                    &mut x,
                    &mut vel,
                    goal,
                    goal_v,
                    goal * 0.5,
                    goal_v * 0.5,
                ),
            }
            for j in 0..3 {
                match kind {
                    0 => sx[j] = damper_exact(sx[j], goal[j], 0.3, dt, DEFAULT_EPS),
                    1 => critical_spring_damper_exact(
                        &mut sx[j], &mut sv[j], goal[j], goal_v[j], 0.3, dt,
                    ),
                    2 => decay_spring_damper_exact(&mut sx[j], &mut sv[j], 0.3, dt),
                    3 => extrapolate(&mut sx[j], &mut sv[j], dt, 0.3, DEFAULT_EPS),
                    _ => inertialize_transition(
                        &mut sx[j],
                        &mut sv[j],
                        goal[j],
                        goal_v[j],
                        goal[j] * 0.5,
                        goal_v[j] * 0.5,
                    ),
                }
            }
            assert_eq!(x.to_array(), sx, "form {kind}");
            assert_eq!(vel.to_array(), sv, "form {kind}");
        }
    }
    let (mut x, mut vel) = (Vec3::new(0.1, -0.4, 0.9), Vec3::new(0.3, -0.2, 0.7));
    let (mut sx, mut sv) = (x.to_array(), vel.to_array());
    for _ in 0..300 {
        let (mut out, mut ov) = (Vec3::ZERO, Vec3::ZERO);
        inertialize_update_vec(
            &mut out,
            &mut ov,
            &mut x,
            &mut vel,
            Vec3::ONE,
            Vec3::splat(0.2),
            0.3,
            1.0 / 60.0,
        );
        for j in 0..3 {
            let (mut ox, mut vx) = (0.0, 0.0);
            inertialize_update(
                &mut ox,
                &mut vx,
                &mut sx[j],
                &mut sv[j],
                1.0,
                0.2,
                0.3,
                1.0 / 60.0,
            );
            assert_eq!(out[j], ox);
            assert_eq!(ov[j], vx);
        }
        assert_eq!(x.to_array(), sx);
        assert_eq!(vel.to_array(), sv);
    }
}
