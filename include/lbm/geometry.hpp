// NACA 4-digit airfoil -> boolean solid mask. Header-only, zero platform deps.
//
// The mask is uint8 (1 = solid). Any boolean mask works as a drop-in, so the
// airfoil can be swapped for an arbitrary silhouette later.
#ifndef LBM_GEOMETRY_HPP
#define LBM_GEOMETRY_HPP

#include <cmath>
#include <cstdint>
#include <vector>

namespace lbm {

// NACA00xx half-thickness at chord fraction xc in [0,1], scaled to `chord`.
inline double naca_thickness(double xc, double t, double chord) {
    return 5.0 * t *
           (0.2969 * std::sqrt(xc) - 0.1260 * xc - 0.3516 * xc * xc +
            0.2843 * xc * xc * xc - 0.1015 * xc * xc * xc * xc) *
           chord;
}

// Build a solid mask for a NACA 4-digit section.
//   pivotX, pivotY : leading-edge pivot in world (grid) coordinates
//   thickness      : 't' (e.g. 0.12 for NACA0012)
//   aoaRad         : angle of attack in radians
inline std::vector<std::uint8_t> naca_solid_mask(int nx, int ny, double chord,
                                                 double pivotX, double pivotY,
                                                 double thickness,
                                                 double aoaRad) {
    std::vector<std::uint8_t> mask(static_cast<size_t>(nx) * ny, 0u);
    const double ca = std::cos(aoaRad);
    const double sa = std::sin(aoaRad);
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            const double dx = x - pivotX;
            const double dy = y - pivotY;
            // Rotate world -> airfoil frame by -aoa.
            const double xr = dx * ca + dy * sa;
            const double yr = -dx * sa + dy * ca;
            if (xr < 0.0 || xr > chord) continue;
            const double yt = naca_thickness(xr / chord, thickness, chord);
            if (std::abs(yr) <= yt) mask[static_cast<size_t>(x) + y * nx] = 1u;
        }
    }
    return mask;
}

// Cylinder (2D circle) — the canonical bluff body for a Kármán vortex street.
inline std::vector<std::uint8_t> cylinder_solid_mask(int nx, int ny,
                                                     double cxc, double cyc,
                                                     double radius) {
    std::vector<std::uint8_t> mask(static_cast<size_t>(nx) * ny, 0u);
    const double r2 = radius * radius;
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            const double dx = x - cxc, dy = y - cyc;
            if (dx * dx + dy * dy <= r2) mask[static_cast<size_t>(x) + y * nx] = 1u;
        }
    }
    return mask;
}

// Ellipse with semi-axes (rx, ry), rotated by aoaRad.
inline std::vector<std::uint8_t> ellipse_solid_mask(int nx, int ny, double cxc,
                                                    double cyc, double rx,
                                                    double ry, double aoaRad) {
    std::vector<std::uint8_t> mask(static_cast<size_t>(nx) * ny, 0u);
    const double ca = std::cos(aoaRad), sa = std::sin(aoaRad);
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            const double dx = x - cxc, dy = y - cyc;
            const double xr = dx * ca + dy * sa;   // rotate by -aoa
            const double yr = -dx * sa + dy * ca;
            const double e = (xr * xr) / (rx * rx) + (yr * yr) / (ry * ry);
            if (e <= 1.0) mask[static_cast<size_t>(x) + y * nx] = 1u;
        }
    }
    return mask;
}

// Axis-aligned box of half-extents (halfW, halfH), rotated by aoaRad.
inline std::vector<std::uint8_t> box_solid_mask(int nx, int ny, double cxc,
                                                double cyc, double halfW,
                                                double halfH, double aoaRad) {
    std::vector<std::uint8_t> mask(static_cast<size_t>(nx) * ny, 0u);
    const double ca = std::cos(aoaRad), sa = std::sin(aoaRad);
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            const double dx = x - cxc, dy = y - cyc;
            const double xr = dx * ca + dy * sa;
            const double yr = -dx * sa + dy * ca;
            if (std::abs(xr) <= halfW && std::abs(yr) <= halfH)
                mask[static_cast<size_t>(x) + y * nx] = 1u;
        }
    }
    return mask;
}

}  // namespace lbm

#endif  // LBM_GEOMETRY_HPP
