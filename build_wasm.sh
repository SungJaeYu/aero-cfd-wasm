#!/usr/bin/env bash
# Build the LBM solver to an ES6 WASM module: web/dist/lbm.mjs + lbm.wasm.
# Requires emscripten (emcc) on PATH. Run the native gate (ctest) first.
set -euo pipefail

if ! command -v emcc >/dev/null 2>&1; then
  echo "error: emcc not found. Install emsdk and 'source emsdk_env.sh' first." >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$ROOT/web/dist"
mkdir -p "$OUT"

EXPORTS='["_lbm_create","_lbm_destroy","_lbm_set_naca","_lbm_set_cylinder","_lbm_set_ellipse","_lbm_set_box","_lbm_step","_lbm_nx","_lbm_ny","_lbm_rho","_lbm_ux","_lbm_uy","_lbm_solid","_malloc","_free"]'

emcc "$ROOT/src/wasm_bindings.cpp" \
  -I"$ROOT/include" \
  -std=c++17 -O3 \
  -s MODULARIZE=1 \
  -s EXPORT_ES6=1 \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s EXPORTED_FUNCTIONS="$EXPORTS" \
  -s EXPORTED_RUNTIME_METHODS='["HEAPF32","HEAPU8","cwrap","ccall"]' \
  -s ENVIRONMENT=web \
  -o "$OUT/lbm.mjs"

echo "built: $OUT/lbm.mjs + lbm.wasm"
