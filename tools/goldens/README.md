# Reference trajectories

Run `./tools/goldens/generate.sh` with Clang or GCC and Python 3. The harness calls extracted, unmodified math bodies from the pinned upstream. Verbatim files are under `upstream/` and excluded from distribution. `reference.h` removes raylib, display globals, commented alternatives and demo signal generators. C++17, O0, and disabled floating-point contraction are intentional.

The JSON records function inputs and all mutated outputs before/after every step. Dynamic families run 300 fixed-delta steps and 300 mixed-delta steps with goal changes at 0, 100 and 200.

Fixtures are losslessly gzip-compressed JSON with a fixed gzip timestamp, keeping the total below 2 MB. Python's `gzip` module can read them. Scalar inputs that never change are stored once in each family's template; columns list the varying positions. Quaternion rows omit redundant post-state except for inertialization update. Rust tests decode the files directly, including when testing the packaged crate.

The quaternion runner covers fixed and mixed deltas plus a negated-current goal. Transition fixtures reset the outgoing offsets for each independent transition; repeatedly multiplying the same offset without a source update is not a meaningful animation transition.
