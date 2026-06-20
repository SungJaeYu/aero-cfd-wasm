// Plain C exports for WASM (no Embind). Pointers returned to JS are read via
// Module.HEAPF32.subarray(ptr>>2, ...) / HEAPU8. With ALLOW_MEMORY_GROWTH the
// heap can detach, so the JS side must re-acquire views every frame.
#include <cstdint>

#include "lbm/geometry.hpp"
#include "lbm/lbm.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

extern "C" {

EMSCRIPTEN_KEEPALIVE
lbm::Lbm* lbm_create(int nx, int ny, double u_in, double Re, double chord) {
    return new lbm::Lbm(nx, ny, u_in, Re, chord);
}

EMSCRIPTEN_KEEPALIVE
void lbm_destroy(lbm::Lbm* s) { delete s; }

// Rebuild the solid mask for a NACA section (e.g. on an AoA slider change).
EMSCRIPTEN_KEEPALIVE
void lbm_set_naca(lbm::Lbm* s, double pivotX, double pivotY, double thickness,
                  double aoaDeg) {
    const double aoaRad = aoaDeg * 3.14159265358979323846 / 180.0;
    s->setSolid(lbm::naca_solid_mask(s->nx(), s->ny(), s->chord(), pivotX,
                                     pivotY, thickness, aoaRad));
}

EMSCRIPTEN_KEEPALIVE
void lbm_set_cylinder(lbm::Lbm* s, double cx, double cy, double radius) {
    s->setSolid(lbm::cylinder_solid_mask(s->nx(), s->ny(), cx, cy, radius));
}

EMSCRIPTEN_KEEPALIVE
void lbm_set_ellipse(lbm::Lbm* s, double cx, double cy, double rx, double ry,
                     double aoaDeg) {
    const double a = aoaDeg * 3.14159265358979323846 / 180.0;
    s->setSolid(lbm::ellipse_solid_mask(s->nx(), s->ny(), cx, cy, rx, ry, a));
}

EMSCRIPTEN_KEEPALIVE
void lbm_set_box(lbm::Lbm* s, double cx, double cy, double halfW, double halfH,
                 double aoaDeg) {
    const double a = aoaDeg * 3.14159265358979323846 / 180.0;
    s->setSolid(lbm::box_solid_mask(s->nx(), s->ny(), cx, cy, halfW, halfH, a));
}

EMSCRIPTEN_KEEPALIVE
void lbm_step(lbm::Lbm* s, int substeps) { s->step(substeps); }

EMSCRIPTEN_KEEPALIVE
int lbm_nx(lbm::Lbm* s) { return s->nx(); }

EMSCRIPTEN_KEEPALIVE
int lbm_ny(lbm::Lbm* s) { return s->ny(); }

EMSCRIPTEN_KEEPALIVE
const float* lbm_rho(lbm::Lbm* s) { return s->rho(); }

EMSCRIPTEN_KEEPALIVE
const float* lbm_ux(lbm::Lbm* s) { return s->ux(); }

EMSCRIPTEN_KEEPALIVE
const float* lbm_uy(lbm::Lbm* s) { return s->uy(); }

EMSCRIPTEN_KEEPALIVE
const std::uint8_t* lbm_solid(lbm::Lbm* s) { return s->solid(); }

}  // extern "C"
