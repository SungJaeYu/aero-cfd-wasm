# aero-cfd-wasm

Real-time CFD of flow over an aircraft wing section, in the browser. A
**Lattice-Boltzmann (D2Q9, BGK)** solver written in header-only C++17, compiled
to **WebAssembly**, rendered on a canvas with live velocity / density / vorticity
fields and AoA · Reynolds controls.

## Why Lattice-Boltzmann (not Navier–Stokes directly)

The quantities we want to show — density ρ and velocity **u** — are direct
moments of the distribution functions (`ρ = Σ fᵢ`, `u = Σ fᵢ cᵢ / ρ`). There is
no pressure-Poisson solve, solids are a one-line bounce-back, and the inner loop
is a tight stencil that maps cleanly onto a WASM hot path.

## Physics

- **D2Q9 BGK**, single relaxation time. `feq_i = w_i ρ (1 + 3(c_i·u) +
  4.5(c_i·u)² − 1.5|u|²)`, `cs² = 1/3`.
- Viscosity from Reynolds: `ν = u_in·chord / Re`, `τ = 3ν + 0.5`, `ω = 1/τ`
  (τ > 0.5, ω < 2 in the stable region).
- Step order: **collide → stream → applyBoundaries**. Streaming is pull-based
  with halfway bounce-back at solid cells; ping-pong buffers.
- Boundaries: inlet velocity Dirichlet (free-stream equilibrium), outlet
  zero-gradient, top/bottom free-stream (avoids external-flow blockage).
- Airfoil: NACA 4-digit thickness mapped to a boolean solid mask in the airfoil
  frame (rotated by −AoA). The mask is boolean, so any silhouette (e.g. an
  FA-50 section) drops in.

Macroscopic fields (ρ, uₓ, u_y) are stored as **float32** for zero-copy
`HEAPF32` views from JS; the distributions `f` are double; the solid mask is
uint8.

## Layout

```
include/lbm/lbm.hpp        D2Q9 BGK solver (header-only, no platform deps)
include/lbm/geometry.hpp   NACA 4-digit -> solid mask
src/native_main.cpp        native physics smoke test
src/wasm_bindings.cpp      extern "C" exports
web/index.html, render.js  canvas render harness
build_wasm.sh              emscripten build
CMakeLists.txt             native build + ctest
```

## Build & verify

**1. Native first (the gate).** Don't touch WASM until this passes:

```bash
cmake -S . -B build && cmake --build build && (cd build && ctest --output-on-failure)
```

The smoke test runs 3000 steps and checks: finite (no NaN/Inf),
`ρ ∈ [0.98, 1.02]`, `max|u| > u_in` (airfoil acceleration), and
`Ma = u_in·√3 < 0.3`.

**2. Then WASM.** Requires emscripten on PATH (`source emsdk_env.sh`):

```bash
./build_wasm.sh        # -> web/wasm/lbm.mjs + lbm.wasm
python3 -m http.server -d web 8000   # then open http://localhost:8000
```

The module is built with `MODULARIZE=1 EXPORT_ES6=1 ALLOW_MEMORY_GROWTH=1`, so it
imports directly into Astro. Because memory growth detaches the heap, the host
**re-acquires HEAP views every frame**.

### C exports

`lbm_create / lbm_destroy / lbm_set_naca / lbm_step / lbm_nx / lbm_ny /
lbm_rho / lbm_ux / lbm_uy / lbm_solid`. Pointer returns are read via
`Module.HEAPF32.subarray(ptr>>2, …)` (fields) or `HEAPU8` (mask).

## Limitations

- **Weakly-compressible / low-Mach** (`Ma < 0.3`). The density field *is* the
  pressure field (`p = cs²ρ`) — this is not a transonic shock solver.
- **2D wing section.** A single spanwise slice, not a 3D wing.
- **Single-relaxation BGK.** At high Re it grows diffusive; MRT is the fix.

## Roadmap

- Surface integration for **Cd / Cl vs AoA** (through stall).
- **MRT** collision for high-Re stability.
- **wasm-simd128** for the collide/stream loops.
- Couple with `flightdyn`: airspeed → Reynolds → flow.
