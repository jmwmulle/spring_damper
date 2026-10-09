# Implementation status

2026-10-09: Part A (spring_damper) implemented and locally verified. The human initiator authorized public release after minimal Bevy integration verification on 2026-10-09.

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

## Release preparation

The release licence is MIT for the combined distribution, with original additions dedicated under CC0. Upstream MIT notices are preserved. Claude receives design credit; Codex receives port implementation and verification credit. The human initiator did not perform the porting and requires no credit. The approved GitHub identity is the repository owner and commit administrator; commit authorship identifies Codex.

All four minimal Bevy integration tests pass: goal convergence, zero-delta freeze, same-frame transform propagation, and generic scalar/rotation/scale updates. All 20 all-feature tests pass. Both native examples were launched and captured on macOS/Metal. Nonuniform captures confirm rendering, not artistic acceptance or a performance guarantee.

The updated package dry-run compiles and verifies. Package licence notices are included; tools/goldens/upstream snapshots are excluded. Public GitHub release and crates.io publication are authorized. Remote Linux/MSRV CI and publication results are recorded on the repository's Actions, Releases and crates.io pages as they complete.
