# Implementation status

2026-10-09: Part A (spring_damper) implemented locally, awaiting the maintainer's example review and release decision.

## Completed locally

- Scalar, composite, tracking, transition, easing and prediction math.
- Generic vector and quaternion forms, with glam optional.
- Bevy 0.19.1 adapter, reflection, transform and generic value springs, manual/virtual clocks.
- Two runnable dark-background Bevy examples, with captured screenshots held outside the repository.
- Pinned upstream notices, function map, deviations, deterministic C++ fixtures and package inclusion checks.
- 20 all-feature tests pass; 11 tests pass with no default features. Both per-step and whole-trajectory scalar tests compare real C++ outputs without loosening the 1e-5 limit.
- cargo fmt --check, strict all-target/all-feature Clippy, strict rustdoc, normal-dependency-free scalar tree and example builds pass.
- Reference regeneration is byte-identical; fixtures total 382,569 bytes.
- Every translated Rust module has its upstream header; original source snapshots match the recorded hashes.

## Release gate

Package dry-run result is pending the final local check. No GitHub repository has been created, no remote has been pushed, and no crate has been published.

The maintainer must inspect the examples before release. Names, owner, licence and AI-assistance wording remain subject to the maintainer's publication decision. Current recommended choices are spring_damper, jmwmulle, MIT OR Apache-2.0 with retained upstream MIT notices, and an explicit AI-assistance note. Local commit identity was expressly approved.

Rust 1.95 and Linux CI are configured but not yet run remotely. Visual appearance has not been judged by the agent.
