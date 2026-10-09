# Porting record

## Pins

- Spring-It-On: `4d97b497a78e40c7b47d49e8a9da27e4aa5616d6`.
- Motion-Matching: `57b7250e0d34a4e456a34d47e24c2f05fdcc711e`.
- Scalar inputs and outputs are `f32`; caller-owned mutable references replace C++ output references.
- `tools/goldens/upstream-manifest.json` records original-file SHA-256 values. Vendored files keep their original bytes following an added provenance header. Licence texts are copied verbatim in THIRD_PARTY_NOTICES.md.

## Function map

The Rust names and argument order match the upstream unless this table or the deviations section says otherwise.

| Rust module | Upstream files | Rust items |
|---|---|---|
| `convert` | common.h | `DEFAULT_EPS`, `fast_negexp`, `halflife_to_damping`, `damping_to_halflife`, `frequency_to_stiffness`, `stiffness_to_frequency`, `critical_halflife`, `critical_frequency`, `damping_ratio_to_stiffness`, `damping_ratio_to_damping`, `halflife_to_lag`, `lag_to_halflife` |
| `damper` | common.h | `damper_exact`, `damper_decay_exact` |
| `spring` | common.h | `spring_damper_exact_stiffness_damping`, `spring_damper_exact`, `spring_damper_exact_ratio`, `critical_spring_damper_exact`, `simple_spring_damper_exact`, `decay_spring_damper_exact` |
| `composite` | doublespring.c, timedspring.c, velocityspring.c, resonance.c | `double_spring_damper_exact`, `timed_spring_damper_exact`, `velocity_spring_damper_exact`, `spring_energy`, `resonant_frequency` |
| `tracking` | tracking.c | `tracking_spring_update`, `tracking_spring_update_no_acceleration`, `tracking_spring_update_no_velocity_acceleration`, `tracking_spring_update_improved`, `tracking_spring_update_no_acceleration_improved`, `tracking_spring_update_no_velocity_acceleration_improved`, `tracking_spring_update_exact`, `tracking_spring_update_no_acceleration_exact`, `tracking_spring_update_no_velocity_acceleration_exact`, `tracking_target_acceleration`, `tracking_target_velocity` |
| `predict` | controller.c, extrapolation.c | `extrapolate`, `spring_character_update`, `spring_character_predict` |
| `inertialize` | inertialization.c | `inertialize_transition`, `inertialize_update`, `Inertializer`, `transition`, `update` |
| `dead_blend` | deadblending.c | `dead_blending_transition`, `dead_blending_update`, `dead_blending_update_decay`, `DeadBlend`, `transition`, `update` |
| `easing` | inertialeasing.c, cubiceasing.c, interpolation.c | `smoothstep`, `smoothstep_dt`, `smoothstep_solve`, `cubic`, `cubic_dt`, `piecewise_interpolation` |
| `value` | Motion-Matching spring.h and Spring-It-On common.h/extrapolation.c | `SpringValue`, `damper_exact_vec`, `simple_spring_damper_exact_vec`, `critical_spring_damper_exact_vec`, `decay_spring_damper_exact_vec`, `inertialize_transition_vec`, `inertialize_update_vec`, `extrapolate_vec` |
| `quat` | Motion-Matching spring.h, quat.h | `quat_abs`, `quat_to_scaled_angle_axis`, `quat_from_scaled_angle_axis`, `damper_exact_quat`, `damp_adjustment_exact_quat`, `simple_spring_damper_exact_quat`, `decay_spring_damper_exact_quat`, `inertialize_transition_quat`, `inertialize_update_quat` |
| `bevy_layer` | Native adapter (no upstream code) | `SpringClock`, `SpringDelta`, `SpringSystems`, `TranslationSpring`, `RotationSpring`, `ScaleSpring`, `Spring`, `SpringPlugin`, `add_spring_type` |

## Deliberate differences

1. All time-advancing functions freeze dynamic state for nonpositive `dt`. This fixes upstream divisions by zero in tracking/timed springs, preserves exact state at zero despite the underdamped approximation, and avoids running rational decay backward toward its poles. Transition/update outputs still compose the supplied input with unchanged offsets. Completing a dead blend does not change its elapsed-time sentinel at nonpositive dt. Invalid manual clock values also freeze the Bevy adapter.
2. C++ default parameters become explicit Rust arguments. `DEFAULT_EPS` is 1e-5. Dead-blending/easing eps defaults are 1e-8 in the C++ runner; explicit Rust arguments permit the same setting. The unused `eps` argument in velocity springs is retained.
3. Shared branches of the three general scalar spring functions are implemented once in `spring_damper_exact_stiffness_damping`. Arithmetic expression order is retained. The upstream wrappers deliberately use the conversion helper's default epsilon rather than their caller's epsilon, and the port preserves that distinction.
4. Internal expressions containing C++'s double-precision `M_PI` retain that promotion before converting to f32. Public state remains f32. Quaternion multiplication, normalization and division use component expressions matching the source rather than glam shortcuts: near-identity log-map comparisons exposed rounding differences beyond the fixed 1e-5-radian limit.
5. `spring_character_predict` preserves the upstream direct samples at `i * dt`, with index zero representing the initial state. It does **not** repeatedly advance the previous sample. The rational approximation of the exponential is not a semigroup, so the proposed repeated-update equality test would contradict the upstream. Tests compare both real C++ predictor outputs and the directly evaluated samples. Mismatched output slices panic before writing.
6. `piecewise_interpolation` clamps negative normalized time to zero and panics on empty control-point slices instead of performing an invalid memory access. The source's reversed/scaled velocity expression `(pnts[i0]-pnts[i1])/npnts` is retained.
7. The upstream negative-displacement recursion in `smoothstep_solve` drops the custom overshoot and epsilon, reverting to 0.05 and 1e-8. This is preserved and covered by the reference runner.
8. `SpringValue` adds a default `div_scalar` method. Built-in scalar/glam implementations divide each component directly to match scalar rounding; the default for custom values uses reciprocal multiplication. It adds no allocation.
9. The quaternion helpers expose their upstream epsilon explicitly. Motion-Matching's inverse quaternion is the negative of the conventional conjugate; that representative is retained where used, with its source's `quat_abs` selection. Angular velocities remain Vec3.
10. The Bevy adapter also updates built-in generic Spring values. `add_spring_type` registers additional user types; applications register their reflection metadata separately. Virtual time freezes when no virtual clock is installed. Updates occur after animation and before transform propagation, without allocating.
11. Reference JSON is stored as deterministic gzip, under 2 MB total. Its decoded data retains full nine-significant-digit C++ float output. Scalar constants are represented once per family. This is a storage-format change only.
12. The examples explicitly include the gizmo rendering backends for 2D and 3D. They use `Msaa::Off`: on the development macOS/Metal host, default multisampling produced all-black screenshots; disabling it restored nonuniform rendered captures. This is an example rendering setting, with no change to spring math or plugin behavior.
13. The external shared build-cache path is absolute in its local config to ensure both repositories use one cache. It is not part of the distributed crate.

No test tolerance has been loosened: scalar x/v absolute error is at most 1e-5; quaternion rotation error is at most 1e-5 radians. The quaternion test uses the sign-aligned chord of normalized quaternions rather than acos(dot), which loses precision for very small angular errors.

## Omitted upstream items

- `damper_bad` and `spring_damper_bad`: instructional counterexamples.
- `damper`, active `damper_exponential`, and commented earlier exact/exponential alternatives: the selected public API uses the final rational-exp exact damper. The active bodies remain in the verbatim source snapshot.
- Commented earlier underdamped-only `spring_damper_exact`: superseded by the compiled general solver.
- `*_function`, `*_function1`, `*_function2`: demo target-signal generators.
- raylib/raygui rendering, main functions, display history arrays, sliders and demo constants: examples are written directly for Bevy.
- Motion-Matching helpers not reached by the selected spring/quaternion API: vector operators, unrelated interpolation/conversions and animation data-model code. The quat_slerp_shortest_approx and needed log/exp/multiply/normalize helpers are internal; the source credits its approximation to https://zeux.io/2015/07/23/approximating-slerp/.
- No prose, images or figures from the accompanying articles are included.

## Release status

Part A implementation and local validation are tracked in STATUS.md. This crate has not been published. MSRV and Linux CI require a future authorized remote run. No source from a private game project is included.
