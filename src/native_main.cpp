// Native physics smoke test. The verification gate before any WASM build:
//   after 3000 steps -> finite (no NaN/Inf), rho in [0.98,1.02],
//   max|u| > u_in (airfoil acceleration), Ma = u_in*sqrt(3) < 0.3.
// Exits non-zero on failure so ctest catches regressions.
#include <cmath>
#include <cstdio>

#include "lbm/geometry.hpp"
#include "lbm/lbm.hpp"

int main() {
    // Default scenario from the spec.
    const int nx = 400, ny = 200;
    const double u_in = 0.10, Re = 200.0, chord = 60.0;
    const double pivotX = 120.0, pivotY = 100.0, thickness = 0.12;
    const double aoaDeg = 8.0;
    const double aoaRad = aoaDeg * M_PI / 180.0;

    lbm::Lbm sim(nx, ny, u_in, Re, chord);
    sim.setSolid(lbm::naca_solid_mask(nx, ny, chord, pivotX, pivotY, thickness, aoaRad));

    const int steps = 3000;
    sim.step(steps);

    const float* rho = sim.rho();
    const float* ux = sim.ux();
    const float* uy = sim.uy();
    const std::uint8_t* solid = sim.solid();

    bool finite = true;
    float rho_min = 1e9f, rho_max = -1e9f, umax = 0.0f;
    for (int c = 0; c < nx * ny; ++c) {
        if (solid[c]) continue;  // solid cells carry no physical macros
        if (!std::isfinite(rho[c]) || !std::isfinite(ux[c]) || !std::isfinite(uy[c]))
            finite = false;
        if (rho[c] < rho_min) rho_min = rho[c];
        if (rho[c] > rho_max) rho_max = rho[c];
        const float u = std::sqrt(ux[c] * ux[c] + uy[c] * uy[c]);
        if (u > umax) umax = u;
    }

    const double ma = sim.ma();

    const bool ok_finite = finite;
    const bool ok_rho = (rho_min >= 0.98f && rho_max <= 1.02f);
    const bool ok_accel = (umax > static_cast<float>(u_in));
    const bool ok_ma = (ma < 0.3);

    std::printf("steps      = %d\n", steps);
    std::printf("omega      = %.4f\n", sim.omega());
    std::printf("finite     = %s\n", ok_finite ? "yes" : "NO");
    std::printf("rho range  = [%.4f, %.4f]  (want [0.98, 1.02])\n", rho_min, rho_max);
    std::printf("max|u|     = %.4f  (u_in = %.4f)\n", umax, u_in);
    std::printf("Ma         = %.4f  (want < 0.3)\n", ma);

    const bool pass = ok_finite && ok_rho && ok_accel && ok_ma;
    std::printf("RESULT     = %s\n", pass ? "PASS" : "FAIL");
    if (!pass) {
        std::printf("  finite=%d rho=%d accel=%d ma=%d\n", ok_finite, ok_rho, ok_accel, ok_ma);
    }
    return pass ? 0 : 1;
}
