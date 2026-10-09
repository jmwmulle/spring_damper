use spring_damper::*;

#[test]
fn every_scalar_family_matches_real_upstream() {
    let fixture: serde_json::Value = serde_json::from_reader(flate2::read::GzDecoder::new(
        &include_bytes!("goldens/scalar.json.gz")[..],
    ))
    .unwrap();
    for (name, rows) in fixture.as_object().unwrap() {
        for row in rows["rows"].as_array().unwrap() {
            let mut inputs: Vec<f32> = rows["template"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            for (column, value) in rows["columns"]
                .as_array()
                .unwrap()
                .iter()
                .zip(row[0].as_array().unwrap())
            {
                inputs[column.as_u64().unwrap() as usize] = value.as_f64().unwrap() as f32;
            }
            let expected: Vec<f32> = row[1]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            let actual = evaluate(name, &inputs);
            for (i, (actual, expected)) in actual.iter().zip(expected.iter()).enumerate() {
                assert!(
                    (actual - expected).abs() <= 1e-5,
                    "{name} output {i}: {actual} vs {expected}; inputs {inputs:?}"
                );
            }
        }
    }
}

fn evaluate(name: &str, inputs: &[f32]) -> Vec<f32> {
    match name {
        "fast_negexp" => {
            vec![fast_negexp(inputs[0])]
        }
        "damper_exact" => {
            vec![damper_exact(
                inputs[0], inputs[1], inputs[2], inputs[3], inputs[4],
            )]
        }
        "damper_decay_exact" => {
            vec![damper_decay_exact(
                inputs[0], inputs[1], inputs[2], inputs[3],
            )]
        }
        "spring_damper_exact_stiffness_damping" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            spring_damper_exact_stiffness_damping(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
            );
            vec![x, v]
        }
        "halflife_to_damping" => {
            vec![halflife_to_damping(inputs[0], inputs[1])]
        }
        "damping_to_halflife" => {
            vec![damping_to_halflife(inputs[0], inputs[1])]
        }
        "frequency_to_stiffness" => {
            vec![frequency_to_stiffness(inputs[0])]
        }
        "stiffness_to_frequency" => {
            vec![stiffness_to_frequency(inputs[0])]
        }
        "critical_halflife" => {
            vec![critical_halflife(inputs[0])]
        }
        "critical_frequency" => {
            vec![critical_frequency(inputs[0])]
        }
        "spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            spring_damper_exact(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
            );
            vec![x, v]
        }
        "damping_ratio_to_stiffness" => {
            vec![damping_ratio_to_stiffness(inputs[0], inputs[1])]
        }
        "damping_ratio_to_damping" => {
            vec![damping_ratio_to_damping(inputs[0], inputs[1])]
        }
        "spring_damper_exact_ratio" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            spring_damper_exact_ratio(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
            );
            vec![x, v]
        }
        "critical_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            critical_spring_damper_exact(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5],
            );
            vec![x, v]
        }
        "simple_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            simple_spring_damper_exact(&mut x, &mut v, inputs[2], inputs[3], inputs[4]);
            vec![x, v]
        }
        "decay_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            decay_spring_damper_exact(&mut x, &mut v, inputs[2], inputs[3]);
            vec![x, v]
        }
        "halflife_to_lag" => {
            vec![halflife_to_lag(inputs[0])]
        }
        "lag_to_halflife" => {
            vec![lag_to_halflife(inputs[0])]
        }
        "double_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            let mut xi = inputs[2];
            let mut vi = inputs[3];
            double_spring_damper_exact(
                &mut x, &mut v, &mut xi, &mut vi, inputs[4], inputs[5], inputs[6],
            );
            vec![x, v, xi, vi]
        }
        "spring_character_update" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            let mut a = inputs[2];
            spring_character_update(&mut x, &mut v, &mut a, inputs[3], inputs[4], inputs[5]);
            vec![x, v, a]
        }
        "tracking_spring_update" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
                inputs[8],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_acceleration" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_acceleration(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_velocity_acceleration" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_velocity_acceleration(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4],
            );
            vec![x, v]
        }
        "tracking_spring_update_improved" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_improved(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
                inputs[8],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_acceleration_improved" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_acceleration_improved(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_velocity_acceleration_improved" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_velocity_acceleration_improved(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4],
            );
            vec![x, v]
        }
        "tracking_spring_update_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_exact(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
                inputs[8], inputs[9],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_acceleration_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_acceleration_exact(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
            );
            vec![x, v]
        }
        "tracking_spring_update_no_velocity_acceleration_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            tracking_spring_update_no_velocity_acceleration_exact(
                &mut x, &mut v, inputs[2], inputs[3], inputs[4], inputs[5],
            );
            vec![x, v]
        }
        "tracking_target_acceleration" => {
            vec![tracking_target_acceleration(
                inputs[0], inputs[1], inputs[2], inputs[3],
            )]
        }
        "tracking_target_velocity" => {
            vec![tracking_target_velocity(inputs[0], inputs[1], inputs[2])]
        }
        "dead_blending_transition" => {
            let mut ext_x = inputs[0];
            let mut ext_v = inputs[1];
            let mut ext_t = inputs[2];
            dead_blending_transition(&mut ext_x, &mut ext_v, &mut ext_t, inputs[3], inputs[4]);
            vec![ext_x, ext_v, ext_t]
        }
        "smoothstep" => {
            vec![smoothstep(inputs[0], inputs[1])]
        }
        "dead_blending_update" => {
            let mut out_x = inputs[0];
            let mut out_v = inputs[1];
            let mut ext_x = inputs[2];
            let mut ext_v = inputs[3];
            let mut ext_t = inputs[4];
            dead_blending_update(
                &mut out_x, &mut out_v, &mut ext_x, &mut ext_v, &mut ext_t, inputs[5], inputs[6],
                inputs[7], inputs[8], inputs[9],
            );
            vec![out_x, out_v, ext_x, ext_v, ext_t]
        }
        "dead_blending_update_decay" => {
            let mut out_x = inputs[0];
            let mut out_v = inputs[1];
            let mut ext_x = inputs[2];
            let mut ext_v = inputs[3];
            let mut ext_t = inputs[4];
            dead_blending_update_decay(
                &mut out_x, &mut out_v, &mut ext_x, &mut ext_v, &mut ext_t, inputs[5], inputs[6],
                inputs[7], inputs[8], inputs[9], inputs[10],
            );
            vec![out_x, out_v, ext_x, ext_v, ext_t]
        }
        "smoothstep_dt" => {
            vec![smoothstep_dt(inputs[0], inputs[1])]
        }
        "smoothstep_solve" => {
            let mut t = inputs[0];
            let mut s = inputs[1];
            smoothstep_solve(&mut t, &mut s, inputs[2], inputs[3], inputs[4], inputs[5]);
            vec![t, s]
        }
        "extrapolate" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            extrapolate(&mut x, &mut v, inputs[2], inputs[3], inputs[4]);
            vec![x, v]
        }
        "velocity_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            let mut xi = inputs[2];
            velocity_spring_damper_exact(
                &mut x, &mut v, &mut xi, inputs[3], inputs[4], inputs[5], inputs[6], inputs[7],
            );
            vec![x, v, xi]
        }
        "inertialize_transition" => {
            let mut off_x = inputs[0];
            let mut off_v = inputs[1];
            inertialize_transition(
                &mut off_x, &mut off_v, inputs[2], inputs[3], inputs[4], inputs[5],
            );
            vec![off_x, off_v]
        }
        "inertialize_update" => {
            let mut out_x = inputs[0];
            let mut out_v = inputs[1];
            let mut off_x = inputs[2];
            let mut off_v = inputs[3];
            inertialize_update(
                &mut out_x, &mut out_v, &mut off_x, &mut off_v, inputs[4], inputs[5], inputs[6],
                inputs[7],
            );
            vec![out_x, out_v, off_x, off_v]
        }
        "cubic" => {
            vec![cubic(inputs[0], inputs[1], inputs[2])]
        }
        "cubic_dt" => {
            vec![cubic_dt(inputs[0], inputs[1], inputs[2])]
        }
        "timed_spring_damper_exact" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            let mut xi = inputs[2];
            timed_spring_damper_exact(
                &mut x, &mut v, &mut xi, inputs[3], inputs[4], inputs[5], inputs[6],
            );
            vec![x, v, xi]
        }
        "piecewise_interpolation" => {
            let mut x = inputs[0];
            let mut v = inputs[1];
            piecewise_interpolation(&mut x, &mut v, inputs[2], &[0.1, 0.8, -0.4, 1.0]);
            vec![x, v]
        }
        "spring_energy" => {
            vec![spring_energy(
                inputs[0], inputs[1], inputs[2], inputs[3], inputs[4], inputs[5],
            )]
        }
        "resonant_frequency" => {
            vec![resonant_frequency(inputs[0], inputs[1])]
        }
        _ => panic!("unknown upstream function {name}"),
    }
}

#[test]
fn whole_trajectories_match_upstream_without_resetting_each_step() {
    let fixture: serde_json::Value = serde_json::from_reader(flate2::read::GzDecoder::new(
        &include_bytes!("goldens/scalar.json.gz")[..],
    ))
    .unwrap();
    for (name, family) in fixture.as_object().unwrap() {
        let refs = match name.as_str() {
            "spring_damper_exact_stiffness_damping" => 2,
            "spring_damper_exact" => 2,
            "spring_damper_exact_ratio" => 2,
            "critical_spring_damper_exact" => 2,
            "simple_spring_damper_exact" => 2,
            "decay_spring_damper_exact" => 2,
            "double_spring_damper_exact" => 4,
            "spring_character_update" => 3,
            "tracking_spring_update" => 2,
            "tracking_spring_update_no_acceleration" => 2,
            "tracking_spring_update_no_velocity_acceleration" => 2,
            "tracking_spring_update_improved" => 2,
            "tracking_spring_update_no_acceleration_improved" => 2,
            "tracking_spring_update_no_velocity_acceleration_improved" => 2,
            "tracking_spring_update_exact" => 2,
            "tracking_spring_update_no_acceleration_exact" => 2,
            "tracking_spring_update_no_velocity_acceleration_exact" => 2,
            "dead_blending_transition" => 3,
            "dead_blending_update" => 5,
            "dead_blending_update_decay" => 5,
            "smoothstep_solve" => 2,
            "extrapolate" => 2,
            "velocity_spring_damper_exact" => 3,
            "inertialize_transition" => 2,
            "inertialize_update" => 4,
            "timed_spring_damper_exact" => 3,
            "piecewise_interpolation" => 2,
            "damper_exact" => 1,
            "damper_decay_exact" => 1,
            _ => continue,
        };
        let mut previous: Option<Vec<f32>> = None;
        for (index, row) in family["rows"].as_array().unwrap().iter().enumerate() {
            let mut inputs: Vec<f32> = family["template"]
                .as_array()
                .unwrap()
                .iter()
                .map(|n| n.as_f64().unwrap() as f32)
                .collect();
            for (column, value) in family["columns"]
                .as_array()
                .unwrap()
                .iter()
                .zip(row[0].as_array().unwrap())
            {
                inputs[column.as_u64().unwrap() as usize] = value.as_f64().unwrap() as f32;
            }
            if index % 300 != 0
                && let Some(previous) = &previous
            {
                inputs[..refs].copy_from_slice(&previous[..refs]);
            }
            let actual = evaluate(name, &inputs);
            for (i, value) in actual.iter().enumerate() {
                let expected = row[1][i].as_f64().unwrap() as f32;
                assert!(
                    (value - expected).abs() <= 1e-5,
                    "{name}, step {index}, output {i}: {value} vs {expected}"
                );
            }
            previous = Some(actual);
        }
    }
}
