use spring_damper::*;
#[test]
fn dt_zero_leaves_state_unchanged() {
    for dt in [0.0, -0.1] {
        let (mut x, mut v, mut xi, mut vi) = (0.7, -0.3, 0.2, 0.4);
        spring_damper_exact(&mut x, &mut v, 1.0, 0.2, 2.0, 0.3, dt, DEFAULT_EPS);
        assert_eq!((x, v), (0.7, -0.3));
        double_spring_damper_exact(&mut x, &mut v, &mut xi, &mut vi, 1.0, 0.3, dt);
        assert_eq!((x, v, xi, vi), (0.7, -0.3, 0.2, 0.4));
        timed_spring_damper_exact(&mut x, &mut v, &mut xi, 1.0, 0.0, 0.3, dt);
        tracking_spring_update(&mut x, &mut v, 1.0, 0.0, 0.0, 0.1, 0.1, 0.1, dt);
        assert_eq!((x, v, xi), (0.7, -0.3, 0.2));
        assert_eq!(damper_exact(x, 1.0, 0.3, dt, DEFAULT_EPS), x);
    }
}
#[test]
fn large_and_negative_dt_and_tiny_halflife_stay_finite() {
    for dt in [0.0, 0.5, 10.0, -0.1] {
        for halflife in [0.0, 1e-9, 0.3] {
            for ratio in [0.25, 1.0, 2.0] {
                let (mut x, mut v) = (0.7, -0.3);
                spring_damper_exact_ratio(
                    &mut x,
                    &mut v,
                    1.0,
                    0.0,
                    ratio,
                    halflife,
                    dt,
                    DEFAULT_EPS,
                );
                assert!(x.is_finite() && v.is_finite());
            }
            let (mut x, mut v, mut a) = (0.7, -0.3, 0.1);
            critical_spring_damper_exact(&mut x, &mut v, 1.0, 0.0, halflife, dt);
            assert!(x.is_finite() && v.is_finite());
            spring_character_update(&mut x, &mut v, &mut a, 1.0, halflife, dt);
            assert!(x.is_finite() && v.is_finite() && a.is_finite());
            assert!(damper_exact(0.7, 1.0, halflife, dt, DEFAULT_EPS).is_finite());
        }
    }
}
#[test]
fn prediction_samples_direct_upstream_solution() {
    let (mut px, mut pv, mut pa) = ([0.0; 30], [0.0; 30], [0.0; 30]);
    spring_character_predict(
        &mut px,
        &mut pv,
        &mut pa,
        0.4,
        0.2,
        0.1,
        1.0,
        0.3,
        1.0 / 60.0,
    );
    for i in 0..30 {
        let (mut x, mut v, mut a) = (0.4, 0.2, 0.1);
        spring_character_update(&mut x, &mut v, &mut a, 1.0, 0.3, i as f32 / 60.0);
        assert!((x - px[i]).abs() < 1e-6 && (v - pv[i]).abs() < 1e-6 && (a - pa[i]).abs() < 1e-6);
    }
    assert_eq!((px[0], pv[0], pa[0]), (0.4, 0.2, 0.1));
}
#[test]
#[should_panic]
fn predict_rejects_mismatched_slices() {
    spring_character_predict(
        &mut [0.0; 3],
        &mut [0.0; 2],
        &mut [0.0; 3],
        0.0,
        0.0,
        0.0,
        1.0,
        0.3,
        1.0 / 60.0,
    );
}
#[test]
fn inertialize_output_is_continuous_at_transition() {
    let mut state = Inertializer {
        off_x: 0.2,
        off_v: -0.1,
    };
    let before = (0.7 + state.off_x, 0.3 + state.off_v);
    state.transition(0.7, 0.3, -1.2, 0.9);
    let after = state.update(-1.2, 0.9, 0.3, 0.0);
    assert!((before.0 - after.0).abs() < 1e-6 && (before.1 - after.1).abs() < 1e-6);
}
#[test]
fn dead_blend_converges_to_input() {
    let mut state = DeadBlend::default();
    state.transition(-1.0, 0.2);
    for _ in 0..100 {
        state.update(0.7, -0.1, 0.5, 1.0 / 60.0);
    }
    assert_eq!(state.update(0.7, -0.1, 0.5, 1.0 / 60.0), (0.7, -0.1));
}
#[test]
fn stiffness_damping_exercises_all_regimes() {
    for damping in [4.0, 10.0, 20.0] {
        let (mut x, mut v) = (0.0, 0.0);
        for _ in 0..600 {
            spring_damper_exact_stiffness_damping(
                &mut x,
                &mut v,
                1.0,
                0.0,
                25.0,
                damping,
                1.0 / 60.0,
                DEFAULT_EPS,
            );
        }
        assert!((x - 1.0).abs() < 1e-3, "damping={damping}, x={x}");
    }
}
#[test]
fn interpolation_negative_time_is_safe() {
    let (mut x, mut v) = (0.0, 0.0);
    piecewise_interpolation(&mut x, &mut v, -0.4, &[0.2, 0.8]);
    assert_eq!(x, 0.2);
    assert!(v.is_finite());
}
#[test]
fn predictor_matches_real_upstream() {
    let golden: serde_json::Value =
        serde_json::from_str(include_str!("goldens/predict.json")).unwrap();
    let (mut px, mut pv, mut pa) = ([0.0; 30], [0.0; 30], [0.0; 30]);
    spring_character_predict(
        &mut px,
        &mut pv,
        &mut pa,
        0.4,
        0.2,
        0.1,
        1.0,
        0.3,
        1.0 / 60.0,
    );
    for i in 0..30 {
        for (j, value) in [px[i], pv[i], pa[i]].iter().enumerate() {
            assert!((value - golden[i][j].as_f64().unwrap() as f32).abs() < 1e-5);
        }
    }
}
