// D2Q9 BGK Lattice-Boltzmann solver, header-only, zero platform deps.
//
// Macroscopic fields (rho, ux, uy) are stored as float32 so a JS/WASM host can
// build a zero-copy HEAPF32 view. The distribution buffers f_ are double for
// numerical headroom; the solid mask is uint8.
//
// Step order: collide -> stream (pull + halfway bounce-back) -> applyBoundaries.
#ifndef LBM_LBM_HPP
#define LBM_LBM_HPP

#include <cmath>
#include <cstdint>
#include <vector>

namespace lbm {

// D2Q9 lattice constants. C++17: inline constexpr so the definitions live in
// the header without an out-of-line definition.
inline constexpr int Q = 9;
inline constexpr int cx[Q]  = {0, 1, 0, -1, 0, 1, -1, -1, 1};
inline constexpr int cy[Q]  = {0, 0, 1, 0, -1, 1, 1, -1, -1};
inline constexpr int opp[Q] = {0, 3, 4, 1, 2, 7, 8, 5, 6};
inline constexpr double w[Q] = {
    4.0 / 9.0,
    1.0 / 9.0,  1.0 / 9.0,  1.0 / 9.0,  1.0 / 9.0,
    1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};
inline constexpr double cs2 = 1.0 / 3.0;  // lattice speed of sound squared

class Lbm {
public:
    // u_in: inlet velocity (lattice units, keep Ma = u_in*sqrt(3) < 0.3).
    // Re computed against `chord`. nu = u_in*chord/Re, tau = 3*nu + 0.5.
    Lbm(int nx, int ny, double u_in, double Re, double chord)
        : nx_(nx), ny_(ny), u_in_(u_in), Re_(Re), chord_(chord) {
        const double nu = u_in_ * chord_ / Re_;
        const double tau = 3.0 * nu + 0.5;  // > 0.5 guaranteed for nu > 0
        omega_ = 1.0 / tau;                 // < 2 in the stable region

        const int n = nx_ * ny_;
        f_.assign(static_cast<size_t>(n) * Q, 0.0);
        fnew_.assign(static_cast<size_t>(n) * Q, 0.0);
        rho_.assign(n, 1.0f);
        ux_.assign(n, static_cast<float>(u_in_));
        uy_.assign(n, 0.0f);
        solid_.assign(n, 0u);

        // Initialise to free-stream equilibrium everywhere.
        for (int c = 0; c < n; ++c) {
            double fe[Q];
            equilibrium(1.0, u_in_, 0.0, fe);
            for (int i = 0; i < Q; ++i) f_[static_cast<size_t>(c) * Q + i] = fe[i];
        }
    }

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    double chord() const { return chord_; }
    double omega() const { return omega_; }
    double ma() const { return u_in_ * std::sqrt(3.0); }

    const float* rho() const { return rho_.data(); }
    const float* ux() const { return ux_.data(); }
    const float* uy() const { return uy_.data(); }
    const std::uint8_t* solid() const { return solid_.data(); }

    // Replace the solid mask (e.g. after an AoA change). Size must be nx*ny.
    void setSolid(const std::vector<std::uint8_t>& mask) {
        if (static_cast<int>(mask.size()) == nx_ * ny_) solid_ = mask;
    }

    // Advance `substeps` full LBM steps.
    void step(int substeps) {
        for (int s = 0; s < substeps; ++s) {
            collide();
            stream();
            applyBoundaries();
        }
    }

private:
    int idx(int x, int y) const { return x + y * nx_; }

    static void equilibrium(double rho, double ux, double uy, double fe[Q]) {
        const double usq = ux * ux + uy * uy;
        for (int i = 0; i < Q; ++i) {
            const double cu = cx[i] * ux + cy[i] * uy;
            fe[i] = w[i] * rho *
                    (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * usq);
        }
    }

    // Compute macros, store as float32, then relax toward equilibrium in place.
    void collide() {
        for (int y = 0; y < ny_; ++y) {
            for (int x = 0; x < nx_; ++x) {
                const int c = idx(x, y);
                if (solid_[c]) {
                    rho_[c] = 1.0f;
                    ux_[c] = 0.0f;
                    uy_[c] = 0.0f;
                    continue;
                }
                double* fc = &f_[static_cast<size_t>(c) * Q];
                double rho = 0.0, mx = 0.0, my = 0.0;
                for (int i = 0; i < Q; ++i) {
                    rho += fc[i];
                    mx += fc[i] * cx[i];
                    my += fc[i] * cy[i];
                }
                const double ux = mx / rho;
                const double uy = my / rho;
                rho_[c] = static_cast<float>(rho);
                ux_[c] = static_cast<float>(ux);
                uy_[c] = static_cast<float>(uy);

                double fe[Q];
                equilibrium(rho, ux, uy, fe);
                for (int i = 0; i < Q; ++i) fc[i] += omega_ * (fe[i] - fc[i]);
            }
        }
    }

    // Pull-streaming with halfway bounce-back. f_ is post-collision; write fnew_.
    void stream() {
        double fs_in[Q];
        equilibrium(1.0, u_in_, 0.0, fs_in);  // free-stream for out-of-grid pulls
        for (int y = 0; y < ny_; ++y) {
            for (int x = 0; x < nx_; ++x) {
                const int c = idx(x, y);
                double* fn = &fnew_[static_cast<size_t>(c) * Q];
                if (solid_[c]) {
                    // Solid cells carry no usable populations; keep buffer sane.
                    for (int i = 0; i < Q; ++i) fn[i] = f_[static_cast<size_t>(c) * Q + i];
                    continue;
                }
                for (int i = 0; i < Q; ++i) {
                    const int xs = x - cx[i];
                    const int ys = y - cy[i];
                    if (xs < 0 || xs >= nx_ || ys < 0 || ys >= ny_) {
                        fn[i] = fs_in[i];  // off-grid -> free-stream inflow
                    } else if (solid_[idx(xs, ys)]) {
                        // halfway bounce-back: reflect this cell's own
                        // post-collision opposite population.
                        fn[i] = f_[static_cast<size_t>(c) * Q + opp[i]];
                    } else {
                        fn[i] = f_[static_cast<size_t>(idx(xs, ys)) * Q + i];
                    }
                }
            }
        }
        f_.swap(fnew_);
    }

    // Applied to f_ after the swap.
    void applyBoundaries() {
        double fs[Q];
        equilibrium(1.0, u_in_, 0.0, fs);

        // inlet (x = 0): velocity Dirichlet via free-stream equilibrium.
        for (int y = 0; y < ny_; ++y) {
            const int c = idx(0, y);
            for (int i = 0; i < Q; ++i) f_[static_cast<size_t>(c) * Q + i] = fs[i];
        }
        // outlet (x = nx-1): zero-gradient copy from the previous column.
        for (int y = 0; y < ny_; ++y) {
            const int c = idx(nx_ - 1, y);
            const int cp = idx(nx_ - 2, y);
            for (int i = 0; i < Q; ++i)
                f_[static_cast<size_t>(c) * Q + i] = f_[static_cast<size_t>(cp) * Q + i];
        }
        // top / bottom: free-stream to avoid external-flow blockage.
        for (int x = 0; x < nx_; ++x) {
            const int cb = idx(x, 0);
            const int ct = idx(x, ny_ - 1);
            for (int i = 0; i < Q; ++i) {
                f_[static_cast<size_t>(cb) * Q + i] = fs[i];
                f_[static_cast<size_t>(ct) * Q + i] = fs[i];
            }
        }
    }

    int nx_, ny_;
    double u_in_, Re_, chord_, omega_;
    std::vector<double> f_, fnew_;
    std::vector<float> rho_, ux_, uy_;
    std::vector<std::uint8_t> solid_;
};

}  // namespace lbm

#endif  // LBM_LBM_HPP
