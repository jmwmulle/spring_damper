#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT HUP INT TERM
c++ -std=c++17 -O0 -ffp-contract=off tools/goldens/harness.cpp -o "$scratch/goldens"
mkdir -p tests/goldens
"$scratch/goldens" > "$scratch/scalar.json"
c++ -std=c++17 -O0 -ffp-contract=off tools/goldens/quat_harness.cpp -o "$scratch/quats"
"$scratch/quats" > "$scratch/quat.json"
python3 tools/goldens/compact.py "$scratch/scalar.json" "$scratch/quat.json"
c++ -std=c++17 -O0 -ffp-contract=off tools/goldens/predict_harness.cpp -o "$scratch/predict"
"$scratch/predict" > tests/goldens/predict.json
