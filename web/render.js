// Canvas render harness for the LBM WASM module.
// Pulls rho/ux/uy each frame, maps a selected field to a cockpit palette, and
// blits an NX*NY ImageData scaled up to the display canvas.
//
// ALLOW_MEMORY_GROWTH detaches HEAP views on growth, so views are re-acquired
// every frame from Module.HEAPF32 / Module.HEAPU8.
import createModule from "./dist/lbm.mjs";

// --- cockpit theme -----------------------------------------------------------
const BASE = [0x0b, 0x0f, 0x12];
const CYAN = [0x3f, 0xc8, 0xd4];
const AMBER = [0xf0, 0xa9, 0x3b];
const SOLID = [0x2e, 0x3a, 0x3f];

const lerp = (a, b, t) => a + (b - a) * t;
const mix = (c1, c2, t) => [
  lerp(c1[0], c2[0], t),
  lerp(c1[1], c2[1], t),
  lerp(c1[2], c2[2], t),
];

// speed: dark -> cyan -> amber
function speedColor(t) {
  t = Math.max(0, Math.min(1, t));
  return t < 0.5 ? mix(BASE, CYAN, t / 0.5) : mix(CYAN, AMBER, (t - 0.5) / 0.5);
}
// diverging: cyan (negative) -> dark (zero) -> amber (positive); s in [-1,1]
function divergingColor(s) {
  s = Math.max(-1, Math.min(1, s));
  return s < 0 ? mix(BASE, CYAN, -s) : mix(BASE, AMBER, s);
}

// --- defaults (spec) ---------------------------------------------------------
const DEFAULTS = {
  nx: 400,
  ny: 200,
  uIn: 0.1,
  Re: 200,
  chord: 60,
  pivotX: 120,
  pivotY: 100,
  thickness: 0.12,
  aoaDeg: 8,
  substeps: 10,
};

async function main() {
  const Module = await createModule();
  const api = {
    create: Module.cwrap("lbm_create", "number", ["number", "number", "number", "number", "number"]),
    destroy: Module.cwrap("lbm_destroy", null, ["number"]),
    setNaca: Module.cwrap("lbm_set_naca", null, ["number", "number", "number", "number", "number"]),
    step: Module.cwrap("lbm_step", null, ["number", "number"]),
    nx: Module.cwrap("lbm_nx", "number", ["number"]),
    ny: Module.cwrap("lbm_ny", "number", ["number"]),
    rho: Module.cwrap("lbm_rho", "number", ["number"]),
    ux: Module.cwrap("lbm_ux", "number", ["number"]),
    uy: Module.cwrap("lbm_uy", "number", ["number"]),
    solid: Module.cwrap("lbm_solid", "number", ["number"]),
  };

  const display = document.getElementById("view");
  const off = document.createElement("canvas");
  const offCtx = off.getContext("2d");
  const dispCtx = display.getContext("2d");
  dispCtx.imageSmoothingEnabled = false;

  const state = { ...DEFAULTS, sim: 0, field: "vorticity", playing: true, img: null };

  function build() {
    if (state.sim) api.destroy(state.sim);
    state.sim = api.create(state.nx, state.ny, state.uIn, state.Re, state.chord);
    api.setNaca(state.sim, state.pivotX, state.pivotY, state.thickness, state.aoaDeg);
    off.width = state.nx;
    off.height = state.ny;
    state.img = offCtx.createImageData(state.nx, state.ny);
  }

  // float32 view over a returned pointer, re-acquired (growth may detach).
  const f32 = (ptr, n) => Module.HEAPF32.subarray(ptr >> 2, (ptr >> 2) + n);
  const u8 = (ptr, n) => Module.HEAPU8.subarray(ptr, ptr + n);

  function render() {
    const { nx, ny } = state;
    const n = nx * ny;
    const rho = f32(api.rho(state.sim), n);
    const ux = f32(api.ux(state.sim), n);
    const uy = f32(api.uy(state.sim), n);
    const solid = u8(api.solid(state.sim), n);
    const data = state.img.data;

    for (let y = 0; y < ny; y++) {
      for (let x = 0; x < nx; x++) {
        const c = x + y * nx;
        let col;
        if (solid[c]) {
          col = SOLID;
        } else if (state.field === "speed") {
          const sp = Math.hypot(ux[c], uy[c]);
          col = speedColor(sp / (state.uIn * 1.8));
        } else if (state.field === "density") {
          col = divergingColor((rho[c] - 1.0) * 40.0); // p = cs^2 * rho
        } else {
          // vorticity: central difference of velocity, guarded at borders
          const xm = x > 0 ? c - 1 : c;
          const xp = x < nx - 1 ? c + 1 : c;
          const ym = y > 0 ? c - nx : c;
          const yp = y < ny - 1 ? c + nx : c;
          const w = (uy[xp] - uy[xm]) * 0.5 - (ux[yp] - ux[ym]) * 0.5;
          col = divergingColor(w * 60.0);
        }
        // flip y so +y points up on screen
        const px = (x + (ny - 1 - y) * nx) * 4;
        data[px] = col[0];
        data[px + 1] = col[1];
        data[px + 2] = col[2];
        data[px + 3] = 255;
      }
    }
    offCtx.putImageData(state.img, 0, 0);
    dispCtx.drawImage(off, 0, 0, display.width, display.height);
  }

  function frame() {
    if (state.playing) api.step(state.sim, state.substeps);
    render();
    requestAnimationFrame(frame);
  }

  // --- controls --------------------------------------------------------------
  const $ = (id) => document.getElementById(id);
  $("field").addEventListener("change", (e) => (state.field = e.target.value));
  $("aoa").addEventListener("input", (e) => {
    state.aoaDeg = +e.target.value;
    $("aoaVal").textContent = state.aoaDeg.toFixed(0) + "°";
    api.setNaca(state.sim, state.pivotX, state.pivotY, state.thickness, state.aoaDeg);
  });
  $("re").addEventListener("input", (e) => {
    state.Re = +e.target.value;
    $("reVal").textContent = state.Re;
    build(); // viscosity changes omega -> rebuild
  });
  const showMa = () => {
    $("maVal").textContent = "Ma " + (state.uIn * Math.sqrt(3)).toFixed(2);
  };
  $("uin").addEventListener("input", (e) => {
    state.uIn = +e.target.value;
    $("uinVal").textContent = state.uIn.toFixed(3);
    showMa();
    build(); // u_in feeds omega (nu = u_in*chord/Re) and the free-stream BCs
  });
  $("chord").addEventListener("input", (e) => {
    state.chord = +e.target.value;
    $("chordVal").textContent = state.chord;
    build(); // chord feeds omega and the mask -> rebuild
  });
  $("thick").addEventListener("input", (e) => {
    state.thickness = +e.target.value;
    $("thickVal").textContent = state.thickness.toFixed(2);
    api.setNaca(state.sim, state.pivotX, state.pivotY, state.thickness, state.aoaDeg);
  });
  showMa();
  $("playpause").addEventListener("click", (e) => {
    state.playing = !state.playing;
    e.target.textContent = state.playing ? "pause · 정지" : "play · 재생";
  });
  $("reset").addEventListener("click", build);

  build();
  requestAnimationFrame(frame);
}

main();
