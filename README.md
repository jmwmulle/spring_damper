# spring_damper

Damped springs, tracking and smooth motion transitions, translated from Daniel Holden's MIT-licensed C++ animation code. The scalar math works with no dependencies; default features add glam vectors and quaternions. The optional Bevy layer drives transforms or application values using virtual or manual time.

## Scalar use

```rust
use spring_damper::simple_spring_damper_exact;
let (mut position, mut velocity) = (0.0, 0.0);
simple_spring_damper_exact(&mut position, &mut velocity, 1.0, 0.3, 1.0 / 60.0);
assert!(position > 0.0);
```

Time is in seconds. Pass nonnegative halflives, nonnegative stiffness/frequency/damping, and positive epsilons. General spring ratios must be positive. The lower-level conversion functions preserve upstream domain behavior: e.g. asking for a resonant frequency with negative effective stiffness yields NaN. Nonpositive update deltas freeze dynamic state. The core updates allocate no memory.

## Bevy use

Enable the `bevy` feature and install `SpringPlugin` after your normal Bevy plugins. Add `TranslationSpring`, `RotationSpring`, or `ScaleSpring` beside a `Transform`. Each carries a goal, persistent velocity and halflife. Goals use the transform's parent space. Updates run in PostUpdate after AnimationSystems and before transform propagation.

```rust,ignore
app.add_plugins(SpringPlugin);
commands.spawn((
    Transform::default(),
    TranslationSpring { goal: Vec3::X, velocity: Vec3::ZERO, halflife: 0.3 },
));
```

The default clock reads `Time<Virtual>`. To control pauses or simulation time, insert `SpringClock::Manual` and write `SpringDelta(dt)` before PostUpdate. Zero delta preserves transforms and velocities; invalid manual deltas freeze the update. Without an installed virtual clock, Virtual mode also freezes.

`Spring<T>` drives other values. The plugin registers f32, Vec2, Vec3, Vec3A and Vec4; `add_spring_type::<T>` registers updates for an application-defined SpringValue. Read the value in systems ordered after `SpringSystems`. Do not register the same type twice.

| bevy | spring_damper |
|---|---|
| 0.19 | 0.1 |

## Examples

- `cargo run --example spring_gallery --features bevy`: six evolving response plots. Top to bottom: underdamped exact spring, critically damped spring, double spring, timed spring, velocity spring, exact damper. Space restarts the plots; Escape closes.
- `cargo run --example transform_springs --features bevy`: wireframe cubes chase a moving orange marker while their rotations and scales settle. Space changes their halflife; Escape closes.

The examples disable multisample anti-aliasing for reliable captures on the tested macOS/Metal host.

## Verification

C++ reference fixtures cover scalar math, all nine tracking variants, transitions/easing, prediction, and quaternion forms at two frame-time schedules. Tests also cover zero/negative deltas, hitches, tiny halflives, transition continuity, quaternion sign, normalization and Bevy update scheduling. Regenerate with `tools/goldens/generate.sh`; no article text is copied. Fixtures are deterministic, losslessly compressed JSON below 2 MB. See PORTING.md for all intentional differences.

Local validation uses Rust 1.98.1. CI is prepared to check Rust 1.95; its compatibility is not yet verified by a remote run.

A rough core-only timing on the development Apple Silicon host: 1,000 scalar springs over 1,000 updates took 2.818 ms total, approximately 0.0028 ms per batch of 1,000. This uses `rustc -O` and `tools/timing.rs`; it measures the scalar kernel, excluding Bevy, rendering and frame scheduling.

## Origins and credits

- Daniel Holden, [Spring-It-On](https://github.com/orangeduck/Spring-It-On/tree/4d97b497a78e40c7b47d49e8a9da27e4aa5616d6), commit `4d97b497a78e40c7b47d49e8a9da27e4aa5616d6`.
- Daniel Holden, [Motion-Matching](https://github.com/orangeduck/Motion-Matching/tree/57b7250e0d34a4e456a34d47e24c2f05fdcc711e), commit `57b7250e0d34a4e456a34d47e24c2f05fdcc711e`.
- Background articles: [Spring-It-On: The Game Developer's Spring-Roll-Call](https://theorangeduck.com/page/spring-roll-call) and [Dead Blending](https://theorangeduck.com/page/dead-blending). They are linked for explanation; their prose and illustrations are not redistributed.
- The upstream quaternion interpolation approximation credits Arseny Kapoulkine's [Approximating slerp](https://zeux.io/2015/07/23/approximating-slerp/).

Not affiliated with or endorsed by the Godot Foundation, the Godot Engine project or Daniel Holden.

## Porting work

- Design and implementation plan: Claude (Anthropic).
- Rust port, Bevy integration, reference harnesses, implementation and verification: Codex (OpenAI).
- The human initiator proposed having AI tools examine and port existing animation libraries and authorized the release. They did not perform the porting.

These credits describe the work performed; they do not designate AI systems as copyright holders. Upstream algorithm authorship remains with Daniel Holden and the contributors credited above. The port is checked against executable upstream code. No credit to the human initiator or AI tools is required.

## Licence

The combined distribution is MIT, retaining the upstream notices in THIRD_PARTY_NOTICES.md and the source headers. Our original additions are dedicated to the public domain under CC0 1.0 Universal to the extent that the publisher can waive rights in them. Commercial and noncommercial use, modification and redistribution are permitted. See LICENSING.md for scope and LICENSE-CC0 for the waiver and fallback. This dedication does not remove the upstream notice requirement.
