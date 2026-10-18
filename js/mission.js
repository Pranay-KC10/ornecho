/* RS-1 mission replay engine.
 * Shared by the website (window.RSMission) and the offline video renderer
 * (require('./mission.js')). Draws onto any CanvasRenderingContext2D.
 *
 * Mission: vertical takeoff from the home pad, transition to wing-borne flight
 * at 30 m, climb to the top layer, survey each layer top-down on a
 * boustrophedon path (gliding down between layers), return home, hover, land,
 * offload the log over WiFi, then show the finished 3D map slice by slice.
 * World units are survey-grid cells; altitude converts with ZS.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.RSMission = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const NX = 18, NY = 8, LAYERS = 3;
  const ALTS = [40, 70, 100];        // layer altitudes in metres, index 0 = lowest
  const ALT_TRANS = 30;              // hover <-> wing transition height
  const ZS = 0.1;                    // world units per metre
  const PAD = { x: -4.0, y: 7.0 };
  const GS = { x: -5.9, y: 10.1 };
  const GROUND = { x0: -6.6, x1: NX + 3.2, y0: -1.6, y1: NY + 3.4 };
  const EDGE = 0.6, R_TURN = 0.5, R_LOITER = 0.6, R_HELIX = 1.2;
  const SPR = 1.05;                  // aircraft half-span, in cells
  const TOWER_H = 2.6;
  const V_SURVEY = 13, V_CLIMB = 7.5, V_GLIDE = 4.5, V_RTH = 8, V_VERT = 2.0;
  const MS_CRUISE = 18;              // displayed cruise speed, m/s
  const TIME_SCALE = 64;             // replay seconds -> mission-clock seconds
  const SLICE = 1.6, HOLD = 3 * SLICE, FADE = 0.9;
  const ISO_A = Math.cos(Math.PI / 6), ISO_B = 0.5;
  const STOPS = [
    [0.00, [43, 61, 128]],
    [0.28, [34, 81, 163]],
    [0.52, [20, 163, 160]],
    [0.76, [143, 209, 79]],
    [1.00, [246, 215, 67]],
  ];
  const EVENTS = [
    ['takeoff', 'TAKEOFF'], ['transition', 'TRANSITION'], ['climb', 'CLIMB'],
    ['survey-0', 'LAYER 100 M'], ['descend-0', 'GLIDE'], ['survey-1', 'LAYER 70 M'],
    ['descend-1', 'GLIDE'], ['survey-2', 'LAYER 40 M'], ['rth', 'RETURN'],
    ['land', 'LANDING'], ['offload', 'OFFLOAD'],
  ];
  const ROTORS = [[0.335, 0.372], [0.335, -0.372], [-0.55, 0.372], [-0.55, -0.372]];
  const WING = [[0.354, 0.073], [-0.232, 0.976], [-0.305, 1.012], [-0.439, 1.006], [-0.476, 0.963], [-0.378, 0.073]];

  const lerp = (a, b, f) => a + (b - a) * f;
  const smooth = (f) => f * f * (3 - 2 * f);
  const pad2 = (n) => String(n).padStart(2, '0');

  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let r = Math.imul(a ^ (a >>> 15), 1 | a);
      r = (r + Math.imul(r ^ (r >>> 7), 61 | r)) ^ r;
      return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
    };
  }

  function colorAt(v) {
    for (let k = 1; k < STOPS.length; k++) {
      if (v <= STOPS[k][0]) {
        const [p0, c0] = STOPS[k - 1], [p1, c1] = STOPS[k];
        const f = (v - p0) / (p1 - p0);
        return `rgb(${c0.map((ch, n) => Math.round(lerp(ch, c1[n], f))).join(',')})`;
      }
    }
    return `rgb(${STOPS[STOPS.length - 1][1].join(',')})`;
  }

  // ---- Mission path (independent of the signal field) ----------------
  function buildMission() {
    const P = [];
    const layerRange = new Array(LAYERS);
    let tt = 0;
    const lastPt = () => P[P.length - 1];
    const setHeading = () => {
      const b = lastPt(), a = P[P.length - 2];
      const dx = b.x - a.x, dy = b.y - a.y;
      b.h = Math.hypot(dx, dy) > 1e-4 ? Math.atan2(dy, dx) : a.h;
    };
    const at = (x, y, z, dt, o) => {
      tt += dt;
      P.push(Object.assign({ x, y, z, t: tt }, o));
      setHeading();
    };
    const go = (x, y, z, v, o) => {
      const a = lastPt();
      at(x, y, z, Math.hypot(x - a.x, y - a.y, z - a.z) / v, o);
    };
    // Cubic Bezier from the current pose to (p1, heading h1), easing altitude to z1.
    const bezier = (p1, h1, z1, n, v, o) => {
      const a = lastPt(), h0 = a.h, z0 = a.z;
      const L = Math.max(0.5, Math.hypot(p1.x - a.x, p1.y - a.y) * 0.4);
      const c0 = { x: a.x + Math.cos(h0) * L, y: a.y + Math.sin(h0) * L };
      const c1 = { x: p1.x - Math.cos(h1) * L, y: p1.y - Math.sin(h1) * L };
      for (let k = 1; k <= n; k++) {
        const f = k / n, g = 1 - f;
        const x = g * g * g * a.x + 3 * g * g * f * c0.x + 3 * g * f * f * c1.x + f * f * f * p1.x;
        const y = g * g * g * a.y + 3 * g * g * f * c0.y + 3 * g * f * f * c1.y + f * f * f * p1.y;
        go(x, y, lerp(z0, z1, smooth(f)), v, o);
      }
    };

    const zT = ALT_TRANS * ZS;
    const zL = ALTS.map((a) => a * ZS);
    const base = (phase, extra) => Object.assign({ phase, layer: -1, row: -1, rot: 0, push: 0, spd: 0 }, extra);

    // Pre-flight spool-up, then vertical takeoff to transition height.
    P.push(Object.assign({ x: PAD.x, y: PAD.y, z: 0, t: 0, h: 0 }, base('preflight')));
    at(PAD.x, PAD.y, 0, 1.0, base('preflight', { rot: 1 }));
    go(PAD.x, PAD.y, zT, V_VERT, base('takeoff', { rot: 1 }));

    // Transition: accelerate on the pusher while the lift motors spool down.
    for (let k = 1; k <= 12; k++) {
      const f = k / 12;
      go(PAD.x + 1.8 * f, PAD.y, zT, lerp(1.5, V_CLIMB, f),
        base('transition', { rot: 1 - f, push: f, spd: MS_CRUISE * 0.85 * f }));
    }

    // Climb as a plane: right-hand helix, then onto the first row of the top layer.
    {
      const a = lastPt();
      const cx = a.x, cy = a.y - R_HELIX, a0 = Math.PI / 2;
      const turns = 2, n = 72, zTop = zL[LAYERS - 1] - 0.4;
      const o = base('climb', { layer: LAYERS - 1, push: 1, spd: MS_CRUISE * 0.8 });
      for (let k = 1; k <= n; k++) {
        const ang = a0 - (k / n) * turns * 2 * Math.PI;
        go(cx + R_HELIX * Math.cos(ang), cy + R_HELIX * Math.sin(ang), lerp(zT, zTop, k / n), V_CLIMB, o);
      }
      bezier({ x: -EDGE, y: NY - 0.5 }, 0, zL[LAYERS - 1], 16, V_CLIMB, o);
    }

    // Survey the layers from the top down.
    for (let k = 0; k < LAYERS; k++) {
      const l = LAYERS - 1 - k, z = zL[l];
      const order = [];
      for (let j = 0; j < NY; j++) order.push(k % 2 === 0 ? NY - 1 - j : j);
      const first = P.length - 1;
      for (let m = 0; m < NY; m++) {
        const y = order[m] + 0.5;
        const dir = m % 2 === 0 ? 1 : -1;
        const xe = dir > 0 ? NX + EDGE : -EDGE;
        const o = base(`survey-${k}`, { layer: l, row: m, push: 1, spd: MS_CRUISE });
        go(xe, y, z, V_SURVEY, o);
        if (m < NY - 1) {
          const yn = order[m + 1] + 0.5;
          const mid = (y + yn) / 2, half = (yn - y) / 2;
          for (let n = 1; n < 12; n++) {
            const ang = (n / 12) * Math.PI;
            go(xe + dir * R_TURN * Math.sin(ang), mid - half * Math.cos(ang), z, V_SURVEY, o);
          }
        }
      }
      layerRange[l] = [first, P.length - 1];

      if (k < LAYERS - 1) {
        // Glide down in a loiter outside the site: 270 deg + one full turn,
        // then a 90 deg reverse turn back onto the same row, heading the other way.
        const a = lastPt();
        const x0 = a.x, y = a.y;
        const e = y > NY / 2 ? 1 : -1;
        const r = R_LOITER;
        const cy = y + r * e, c2x = x0 + 2 * r;
        const a0 = -e * Math.PI / 2;
        const sw1 = 3.5 * Math.PI, sw2 = 0.5 * Math.PI, total = sw1 + sw2;
        const zB = zL[l - 1];
        const o = base(`descend-${k}`, { layer: l - 1, push: 0.25, spd: MS_CRUISE * 0.85 });
        for (let n = 1; n <= 52; n++) {
          const u = (n / 52) * sw1, ang = a0 - e * u;
          go(x0 + r * Math.cos(ang), cy + r * Math.sin(ang), lerp(z, zB, smooth(u / total)), V_GLIDE, o);
        }
        for (let n = 1; n <= 10; n++) {
          const u = (n / 10) * sw2, ang = Math.PI + e * u;
          go(c2x + r * Math.cos(ang), cy + r * Math.sin(ang), lerp(z, zB, smooth((sw1 + u) / total)), V_GLIDE, o);
        }
      }
    }
    const tSurveyEnd = tt;

    // Return home and line up on final approach, descending to transition height.
    bezier({ x: PAD.x, y: PAD.y - 2.6 }, Math.PI / 2, zT, 28, V_RTH, base('rth', { push: 1, spd: MS_CRUISE }));

    // Back-transition over the pad, vertical landing, spool down, offload.
    {
      const a = lastPt();
      for (let k = 1; k <= 12; k++) {
        const f = k / 12;
        go(PAD.x, lerp(a.y, PAD.y, smooth(f)), zT, lerp(V_CLIMB, 1.2, f),
          base('land', { rot: f, push: 1 - f, spd: MS_CRUISE * (1 - f) }));
      }
    }
    go(PAD.x, PAD.y, 0, V_VERT, base('land', { rot: 1 }));
    at(PAD.x, PAD.y, 0, 0.6, base('offload'));
    const tOffload = tt;
    at(PAD.x, PAD.y, 0, 3.0, base('offload'));
    const tEnd = tt;

    // Battery: hover costs several times cruise; gliding costs least.
    let used = 0;
    P[0].e = 0;
    for (let i = 1; i < P.length; i++) {
      const a = P[i - 1], b = P[i], dt = b.t - a.t;
      const rot = (a.rot + b.rot) / 2, push = (a.push + b.push) / 2;
      used += dt * (6 * rot + push * (b.phase === 'climb' ? 1.5 : 1) + 0.05);
      b.e = used;
    }
    P.forEach((p) => { p.batt = 100 - 63 * (p.e / used); });

    // Event start times for the timeline.
    const events = EVENTS.map(([phase, label], k) => {
      if (k === 0) return { phase, label, t: 0 };
      const i = P.findIndex((p) => p.phase === phase);
      return { phase, label, t: P[i - 1].t };
    });

    return { P, layerRange, tSurveyEnd, tOffload, tEnd, events };
  }

  const MISSION = buildMission();

  // ---- Renderer -------------------------------------------------------
  function create(ctx, options) {
    const o = Object.assign({
      layout: 'hero',             // 'hero' | 'video' | 'panel'
      hud: true,
      font: 'Barlow, "Helvetica Neue", Arial, sans-serif',
      background: null,           // fill colour, or null for transparent
      explode: 1,                 // vertical spread of the layers (map panel)
      seed: 5,
      logo: null,                 // optional image drawn in the video title block
      createCanvas: (w, h) => {
        const c = document.createElement('canvas');
        c.width = w; c.height = h;
        return c;
      },
    }, options);

    const { P, layerRange, tSurveyEnd, tOffload, tEnd, events } = MISSION;
    let seed = o.seed;
    let towers = [], colors = [], revealT = [], sortedReveal = [];
    let W = 0, H = 0, dpr = 1, s = 1, ox = 0, oy = 0;
    let L = {};
    let groundLayer = null;
    let heading = 0;

    const layerZ = (l) => ALTS[l] * ZS * o.explode;

    // ---- Signal field: transmitters along the near edges of the site ----
    function makeField() {
      const rnd = mulberry32(seed++);
      towers = [
        { x: NX + 1.6, y: 1.2 + rnd() * (NY - 2.4), p: 7 + rnd() * 3 },
        { x: 3 + rnd() * (NX - 6), y: NY + 1.8, p: 7 + rnd() * 3 },
      ];
      if (rnd() > 0.35) towers.push({ x: NX + 1.4, y: NY + 1.5, p: 4 + rnd() * 3 });
      const ph = [rnd() * 6.283, rnd() * 6.283, rnd() * 6.283];
      const raw = [];
      let lo = Infinity, hi = -Infinity;
      for (let l = 0; l < LAYERS; l++) {
        const h = ALTS[l] * ZS;
        const arr = new Float32Array(NX * NY);
        for (let j = 0; j < NY; j++) {
          for (let i = 0; i < NX; i++) {
            const cx = i + 0.5, cy = j + 0.5;
            let best = -Infinity;
            for (const tw of towers) {
              const d = Math.hypot(cx - tw.x, cy - tw.y, h - TOWER_H);
              best = Math.max(best, tw.p - 24 * Math.log10(d));
            }
            const shade = (Math.sin(cx * 0.8 + ph[0]) * Math.cos(cy * 1.1 + ph[1]) +
              0.6 * Math.sin((cx - cy) * 0.6 + ph[2])) * 2.2 / (1 + 0.5 * l);
            const v = best + shade;
            arr[j * NX + i] = v;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
          }
        }
        raw.push(arr);
      }
      // Clamp: arr holds float32 values, which can round just outside [lo, hi].
      colors = raw.map((arr) => Array.from(arr, (v) => colorAt(Math.pow(Math.min(1, Math.max(0, (v - lo) / (hi - lo))), 0.9))));

      // First time each cell is flown over during its layer's survey.
      revealT = [];
      for (let l = 0; l < LAYERS; l++) revealT.push(new Float32Array(NX * NY).fill(Infinity));
      for (let i = 1; i < P.length; i++) {
        const a = P[i - 1], b = P[i];
        if (!b.phase.startsWith('survey')) continue;
        const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.08));
        for (let n = 0; n <= steps; n++) {
          const f = n / steps;
          const x = lerp(a.x, b.x, f), y = lerp(a.y, b.y, f);
          if (x < 0 || y < 0 || x >= NX || y >= NY) continue;
          const idx = Math.floor(y) * NX + Math.floor(x);
          const tc = lerp(a.t, b.t, f);
          if (tc < revealT[b.layer][idx]) revealT[b.layer][idx] = tc;
        }
      }
      sortedReveal = [];
      revealT.forEach((arr) => arr.forEach((v) => { if (v !== Infinity) sortedReveal.push(v); }));
      sortedReveal.sort((p, q) => p - q);
    }

    function stateAt(tm) {
      if (tm >= tEnd) return Object.assign({}, P[P.length - 1], { i: P.length - 1, phase: 'complete' });
      let lo = 0, hi = P.length - 1;
      while (hi - lo > 1) {
        const mid = (lo + hi) >> 1;
        if (P[mid].t <= tm) lo = mid; else hi = mid;
      }
      const a = P[lo], b = P[hi];
      const f = (tm - a.t) / (b.t - a.t || 1);
      return {
        x: lerp(a.x, b.x, f), y: lerp(a.y, b.y, f), z: lerp(a.z, b.z, f),
        rot: lerp(a.rot, b.rot, f), push: lerp(a.push, b.push, f), spd: lerp(a.spd, b.spd, f),
        batt: lerp(a.batt, b.batt, f), h: b.h,
        phase: b.phase, layer: b.layer, row: b.row, i: lo,
      };
    }

    function countRevealed(tm) {
      let lo = 0, hi = sortedReveal.length;
      while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (sortedReveal[mid] <= tm) lo = mid + 1; else hi = mid;
      }
      return lo;
    }

    // ---- Projection & layout ----
    const px = (x, y) => ox + (x - y) * ISO_A * s;
    const py = (x, y, z) => oy + (x + y) * ISO_B * s - z * s;
    function plane(x, y, z) {
      ctx.setTransform(dpr * ISO_A * s, dpr * ISO_B * s, -dpr * ISO_A * s, dpr * ISO_B * s,
        dpr * px(x, y), dpr * py(x, y, z));
    }
    const screen = () => ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    function resize(width, height, ratio) {
      W = width; H = height; dpr = ratio || 1;
      const hud = o.hud && o.layout !== 'panel';
      L = {
        bar: hud ? (W >= 720 ? 150 : 104) : 0,
        gauges: hud && W >= 900,
        labels: hud && W >= 720,
        legend: hud && W >= 560,
        groundLabels: W >= 700,
        compact: W < 720,
      };
      let r;
      if (o.layout === 'hero' && W >= 1000) r = { x0: W * 0.34, x1: W - 28, y0: 96, y1: H - L.bar - 8 };
      else if (o.layout === 'hero') r = { x0: 8, x1: W - 8, y0: W < 560 ? 66 : 84, y1: H - L.bar };
      else if (o.layout === 'video') r = { x0: 150, x1: W - 150, y0: 70, y1: H - L.bar + 6 };
      else if (W >= 1000) r = { x0: W * 0.36, x1: W - 40, y0: 60, y1: H - 50 };
      else r = { x0: 12, x1: W - 12, y0: 12, y1: H - 12 };
      L.scene = r;

      const pts = [];
      for (const x of [GROUND.x0, GROUND.x1]) for (const y of [GROUND.y0, GROUND.y1]) pts.push([x, y, 0]);
      for (const x of [0, NX]) for (const y of [0, NY]) pts.push([x, y, layerZ(LAYERS - 1) + 0.4]);
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const [x, y, z] of pts) {
        const X = (x - y) * ISO_A, Y = (x + y) * ISO_B - z;
        minX = Math.min(minX, X); maxX = Math.max(maxX, X);
        minY = Math.min(minY, Y); maxY = Math.max(maxY, Y);
      }
      const aw = r.x1 - r.x0, ah = r.y1 - r.y0;
      s = Math.min(aw / (maxX - minX), ah / (maxY - minY));
      ox = r.x0 + (aw - (maxX - minX) * s) / 2 - minX * s;
      oy = r.y0 + (ah - (maxY - minY) * s) / 2 - minY * s;
      buildGround();
    }

    // Static ground grid, faded towards the edges, cached offscreen.
    function buildGround() {
      groundLayer = o.createCanvas(Math.max(1, Math.round(W * dpr)), Math.max(1, Math.round(H * dpr)));
      const g = groundLayer.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      const P2 = (x, y) => [ox + (x - y) * ISO_A * s, oy + (x + y) * ISO_B * s];
      const { x0, x1, y0, y1 } = GROUND;
      g.beginPath();
      [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].forEach(([x, y], n) => {
        const [X, Y] = P2(x, y);
        if (n) g.lineTo(X, Y); else g.moveTo(X, Y);
      });
      g.closePath();
      g.fillStyle = 'rgba(160,170,190,0.05)';
      g.fill();
      g.strokeStyle = 'rgba(255,255,255,0.055)';
      g.lineWidth = 1;
      g.beginPath();
      for (let x = Math.ceil(x0); x <= x1; x++) { g.moveTo(...P2(x, y0)); g.lineTo(...P2(x, y1)); }
      for (let y = Math.ceil(y0); y <= y1; y++) { g.moveTo(...P2(x0, y)); g.lineTo(...P2(x1, y)); }
      g.stroke();
      const [cx, cy] = P2((x0 + x1) / 2, (y0 + y1) / 2);
      const rad = Math.max(L.scene.x1 - L.scene.x0, L.scene.y1 - L.scene.y0) * 0.62;
      const fade = g.createRadialGradient(cx, cy, rad * 0.3, cx, cy, rad);
      fade.addColorStop(0, 'rgba(0,0,0,1)');
      fade.addColorStop(1, 'rgba(0,0,0,0)');
      g.globalCompositeOperation = 'destination-in';
      g.fillStyle = fade;
      g.fillRect(0, 0, W, H);
      g.globalCompositeOperation = 'source-over';
    }

    // ---- Text ----
    function text(str, x, y, size, weight, color, align, spacing) {
      ctx.font = `${weight} ${size}px ${o.font}`;
      ctx.fillStyle = color;
      ctx.textBaseline = 'middle';
      if (!spacing) {
        ctx.textAlign = align || 'left';
        ctx.fillText(str, x, y);
        ctx.textAlign = 'left';
        return;
      }
      ctx.textAlign = 'left';
      const ws = [];
      let w = 0;
      for (const ch of str) { const cw = ctx.measureText(ch).width; ws.push(cw); w += cw + spacing; }
      w -= spacing;
      let cx = align === 'center' ? x - w / 2 : align === 'right' ? x - w : x;
      let k = 0;
      for (const ch of str) { ctx.fillText(ch, cx, y); cx += ws[k++] + spacing; }
    }

    // ---- Scene ----
    function quad(x0, y0, x1, y1, z) {
      ctx.beginPath();
      ctx.moveTo(px(x0, y0), py(x0, y0, z));
      ctx.lineTo(px(x1, y0), py(x1, y0, z));
      ctx.lineTo(px(x1, y1), py(x1, y1, z));
      ctx.lineTo(px(x0, y1), py(x0, y1, z));
      ctx.closePath();
    }

    // mode: 'planned' (outline) | 'active' | 'ghost' (dimmed) | 'done' | 'slice' (highlight)
    function drawLayer(l, mode, t, fadeOut) {
      const z = layerZ(l);
      const lit = mode === 'active' || mode === 'slice';
      if (mode === 'planned') {
        quad(0, 0, NX, NY, z);
        ctx.fillStyle = 'rgba(255,255,255,0.012)';
        ctx.fill();
        ctx.setLineDash([4, 5]);
        ctx.strokeStyle = 'rgba(255,255,255,0.24)';
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.setLineDash([]);
      } else {
        const cellAlpha = { active: 0.95, slice: 0.95, ghost: 0.18, done: 0.9 }[mode] * fadeOut;
        quad(0, 0, NX, NY, z);
        ctx.fillStyle = lit ? 'rgba(200,210,230,0.05)' : 'rgba(200,210,230,0.02)';
        ctx.fill();
        const rt = revealT[l], col = colors[l];
        for (let j = 0; j < NY; j++) {
          for (let i = 0; i < NX; i++) {
            const idx = j * NX + i;
            const since = t - rt[idx];
            if (since < 0) continue;
            ctx.globalAlpha = Math.min(1, since / 0.35) * cellAlpha;
            ctx.fillStyle = col[idx];
            quad(i, j, i + 1, j + 1, z);
            ctx.fill();
          }
        }
        ctx.globalAlpha = 1;
        if (mode !== 'ghost') {
          ctx.strokeStyle = 'rgba(0,0,0,0.3)';
          ctx.lineWidth = 1;
          ctx.beginPath();
          for (let i = 0; i <= NX; i++) { ctx.moveTo(px(i, 0), py(i, 0, z)); ctx.lineTo(px(i, NY), py(i, NY, z)); }
          for (let j = 0; j <= NY; j++) { ctx.moveTo(px(0, j), py(0, j, z)); ctx.lineTo(px(NX, j), py(NX, j, z)); }
          ctx.stroke();
        }
        quad(0, 0, NX, NY, z);
        ctx.strokeStyle = lit ? 'rgba(255,255,255,0.75)' : 'rgba(255,255,255,0.18)';
        ctx.lineWidth = lit ? 1.3 : 1;
        ctx.stroke();
      }
      text(`${ALTS[l]} M`, px(NX, 0) + 10, py(NX, 0, z), 11, 600,
        lit ? '#ffffff' : 'rgba(255,255,255,0.45)', 'left', 1.5);
    }

    function strokePath(from, to, tail) {
      if (to < from) return;
      ctx.beginPath();
      ctx.moveTo(px(P[from].x, P[from].y), py(P[from].x, P[from].y, P[from].z));
      for (let i = from + 1; i <= to; i++) ctx.lineTo(px(P[i].x, P[i].y), py(P[i].x, P[i].y, P[i].z));
      if (tail) ctx.lineTo(px(tail.x, tail.y), py(tail.x, tail.y, tail.z));
      ctx.stroke();
    }

    function drawTower(tw, clock) {
      for (let k = 0; k < 3; k++) {
        const f = (clock * 0.35 + k / 3) % 1;
        plane(tw.x, tw.y, 0);
        ctx.beginPath();
        ctx.arc(0, 0, 0.3 + f * 3.2, 0, Math.PI * 2);
        ctx.lineWidth = 0.05;
        ctx.strokeStyle = `rgba(255,255,255,${0.28 * (1 - f)})`;
        ctx.stroke();
      }
      screen();
      const bx = px(tw.x, tw.y), by = py(tw.x, tw.y, 0), ty = py(tw.x, tw.y, TOWER_H);
      const w = 0.22 * s;
      ctx.strokeStyle = 'rgba(225,230,240,0.85)';
      ctx.lineWidth = 1.3;
      ctx.beginPath();
      ctx.moveTo(bx - w, by); ctx.lineTo(bx, ty); ctx.lineTo(bx + w, by);
      for (let k = 1; k <= 3; k++) {
        const f = k / 4, yy = lerp(by, ty, f), ww = w * (1 - f);
        ctx.moveTo(bx - ww, yy); ctx.lineTo(bx + ww, yy);
      }
      ctx.stroke();
      ctx.fillStyle = 'rgba(225,230,240,0.9)';
      ctx.fillRect(bx - 4, ty + 3, 3, 7);
      ctx.fillRect(bx + 1, ty + 3, 3, 7);
      const on = (clock * 1.2 + tw.x) % 1 < 0.5;
      ctx.fillStyle = on ? '#ff4d5e' : 'rgba(255,77,94,0.3)';
      ctx.beginPath(); ctx.arc(bx, ty - 2, 2.4, 0, Math.PI * 2); ctx.fill();
    }

    function drawPad() {
      plane(PAD.x, PAD.y, 0);
      ctx.beginPath();
      ctx.arc(0, 0, 0.85, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,255,255,0.05)';
      ctx.fill();
      ctx.lineWidth = 0.05;
      ctx.strokeStyle = 'rgba(255,255,255,0.5)';
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(-0.28, -0.35); ctx.lineTo(-0.28, 0.35);
      ctx.moveTo(0.28, -0.35); ctx.lineTo(0.28, 0.35);
      ctx.moveTo(-0.28, 0); ctx.lineTo(0.28, 0);
      ctx.lineWidth = 0.09;
      ctx.strokeStyle = 'rgba(255,255,255,0.6)';
      ctx.stroke();
      screen();
      if (L.groundLabels) text('HOME', px(PAD.x, PAD.y) + 1.15 * s, py(PAD.x, PAD.y, 0), 10, 600, 'rgba(255,255,255,0.55)', 'left', 1.5);
    }

    function drawStation() {
      screen();
      const X = px(GS.x, GS.y), Y = py(GS.x, GS.y, 0);
      ctx.fillStyle = 'rgba(225,230,240,0.9)';
      ctx.beginPath();
      ctx.moveTo(X - 13, Y + 2); ctx.lineTo(X + 13, Y + 2); ctx.lineTo(X + 16, Y + 6); ctx.lineTo(X - 16, Y + 6);
      ctx.closePath(); ctx.fill();
      ctx.fillStyle = '#111';
      ctx.strokeStyle = 'rgba(225,230,240,0.9)';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.rect(X - 11, Y - 15, 22, 15); ctx.fill(); ctx.stroke();
      ctx.fillStyle = 'rgba(255,255,255,0.8)';
      ctx.fillRect(X - 8, Y - 6, 4, 4);
      ctx.fillRect(X - 3, Y - 10, 4, 8);
      ctx.fillRect(X + 2, Y - 8, 4, 6);
      if (L.groundLabels) text('GROUND STATION', X - 16, Y + 18, 10, 600, 'rgba(255,255,255,0.55)', 'left', 1.5);
    }

    function aircraft(st, clock, shadow) {
      // Local frame: +u is the nose, v is span. Half-span = 1.
      ctx.beginPath();
      WING.forEach(([u, v], n) => (n ? ctx.lineTo(u, v) : ctx.moveTo(u, v)));
      for (let n = WING.length - 1; n >= 0; n--) ctx.lineTo(WING[n][0], -WING[n][1]);
      ctx.closePath();
      if (shadow) {
        ctx.fillStyle = 'rgba(0,0,0,0.45)';
        ctx.fill();
        if (st.rot > 0.03) {
          ctx.beginPath();
          for (const [u, v] of ROTORS) { ctx.moveTo(u + 0.19 * st.rot, v); ctx.arc(u, v, 0.19 * st.rot, 0, Math.PI * 2); }
          ctx.fill();
        }
        return;
      }
      ctx.fillStyle = '#e4e8ef';
      ctx.fill();
      ctx.lineWidth = 0.03;
      ctx.strokeStyle = 'rgba(0,0,0,0.8)';
      ctx.stroke();

      ctx.fillStyle = '#7b828f';
      ctx.fillRect(-0.55, 0.354, 0.885, 0.036);
      ctx.fillRect(-0.55, -0.39, 0.885, 0.036);

      ctx.beginPath();
      ctx.moveTo(0.61, 0);
      ctx.bezierCurveTo(0.61, 0.1, 0.4, 0.128, 0.152, 0.128);
      ctx.lineTo(-0.427, 0.11);
      ctx.quadraticCurveTo(-0.6, 0.07, -0.622, 0);
      ctx.quadraticCurveTo(-0.6, -0.07, -0.427, -0.11);
      ctx.lineTo(0.152, -0.128);
      ctx.bezierCurveTo(0.4, -0.128, 0.61, -0.1, 0.61, 0);
      ctx.fillStyle = '#ffffff';
      ctx.fill();
      ctx.stroke();

      // Lift rotors: spinning discs in hover, parked fore-aft in cruise.
      ctx.lineCap = 'round';
      ROTORS.forEach(([u, v], n) => {
        if (st.rot > 0.03) {
          ctx.beginPath();
          ctx.arc(u, v, 0.19, 0, Math.PI * 2);
          ctx.fillStyle = `rgba(220,228,240,${0.22 * st.rot})`;
          ctx.fill();
          ctx.lineWidth = 0.015;
          ctx.strokeStyle = `rgba(255,255,255,${0.5 * st.rot})`;
          ctx.stroke();
          const ang = clock * 38 * (n % 2 ? -1 : 1) + n;
          ctx.beginPath();
          ctx.moveTo(u - Math.cos(ang) * 0.18, v - Math.sin(ang) * 0.18);
          ctx.lineTo(u + Math.cos(ang) * 0.18, v + Math.sin(ang) * 0.18);
          ctx.lineWidth = 0.035;
          ctx.strokeStyle = `rgba(0,0,0,${0.25 + 0.4 * (1 - st.rot)})`;
          ctx.stroke();
        } else {
          ctx.beginPath();
          ctx.moveTo(u - 0.15, v); ctx.lineTo(u + 0.15, v);
          ctx.lineWidth = 0.04;
          ctx.strokeStyle = '#000';
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(u, v, 0.05, 0, Math.PI * 2);
        ctx.fillStyle = '#9aa3b2';
        ctx.fill();
      });

      // Pusher prop, edge-on.
      if (st.push > 0.05) {
        ctx.fillStyle = `rgba(255,255,255,${0.3 + 0.6 * st.push})`;
        ctx.fillRect(-0.7, -0.24, 0.05, 0.48);
      } else {
        ctx.fillStyle = '#5a606b';
        ctx.fillRect(-0.69, -0.2, 0.03, 0.4);
      }
    }

    function drawScene(t, dt, clock, st, mapSlice) {
      const mapOnly = mapSlice !== undefined;
      const done = mapOnly || t >= tEnd;
      const fadeOut = !mapOnly && t > tEnd + HOLD ? Math.max(0, 1 - (t - tEnd - HOLD) / FADE) : 1;
      const surveyed = mapOnly || t >= tSurveyEnd;
      const inSurvey = !mapOnly && st.phase.startsWith('survey');
      const tCells = mapOnly ? Infinity : t;

      if (groundLayer) ctx.drawImage(groundLayer, 0, 0, W, H);

      // Survey footprint.
      quad(0, 0, NX, NY, 0);
      ctx.fillStyle = 'rgba(255,255,255,0.025)';
      ctx.fill();
      ctx.setLineDash([5, 5]);
      ctx.strokeStyle = 'rgba(255,255,255,0.3)';
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.setLineDash([]);
      if (L.groundLabels) text('SURVEY AREA', px(NX, NY), py(NX, NY, 0) + 15, 10, 600, 'rgba(255,255,255,0.45)', 'center', 1.5);

      // Volume pillars.
      ctx.setLineDash([2, 4]);
      ctx.strokeStyle = 'rgba(255,255,255,0.12)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      const zTop = layerZ(LAYERS - 1);
      for (const [x, y] of [[0, 0], [NX, 0], [NX, NY], [0, NY]]) {
        ctx.moveTo(px(x, y), py(x, y, 0)); ctx.lineTo(px(x, y), py(x, y, zTop));
      }
      ctx.stroke();
      ctx.setLineDash([]);

      // Layers, bottom to top.
      const slice = mapOnly ? mapSlice : (t >= tEnd ? LAYERS - 1 - (Math.floor((t - tEnd) / SLICE) % LAYERS) : -1);
      for (let l = 0; l < LAYERS; l++) {
        let mode;
        if (done && slice >= 0) mode = l === slice ? 'slice' : 'ghost';
        else if (surveyed) mode = 'done';
        else if (st.layer < 0) mode = 'planned';
        else if (l === st.layer) mode = 'active';
        else mode = l > st.layer ? 'ghost' : 'planned';
        drawLayer(l, mode, tCells, fadeOut);

        if (mode === 'active') {
          const [first, lastIdx] = layerRange[l];
          ctx.lineCap = 'round';
          ctx.lineJoin = 'round';
          ctx.setLineDash([3, 5]);
          ctx.strokeStyle = 'rgba(255,255,255,0.28)';
          ctx.lineWidth = 1;
          if (inSurvey) {
            ctx.beginPath();
            ctx.moveTo(px(st.x, st.y), py(st.x, st.y, st.z));
            for (let i = st.i + 1; i <= lastIdx; i++) ctx.lineTo(px(P[i].x, P[i].y), py(P[i].x, P[i].y, P[i].z));
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.strokeStyle = 'rgba(255,255,255,0.5)';
            ctx.lineWidth = 1.1;
            strokePath(first, st.i, st);
          } else {
            strokePath(first, lastIdx);
            ctx.setLineDash([]);
          }
        }
      }

      // Trail of the current climb, glide or return leg.
      if (!done && !inSurvey && st.phase !== 'preflight') {
        let k = st.i;
        while (k > 0 && P[k].phase === st.phase) k--;
        ctx.strokeStyle = 'rgba(255,255,255,0.7)';
        ctx.lineWidth = 1.4;
        if (st.phase.startsWith('descend')) ctx.setLineDash([4, 3]);
        strokePath(k, st.i, st);
        ctx.setLineDash([]);
      }

      towers.forEach((tw) => drawTower(tw, clock));
      drawPad();
      drawStation();
      if (mapOnly) { screen(); return; }

      // WiFi offload between the aircraft and the ground station.
      if (st.phase === 'offload' && t >= tOffload) {
        screen();
        const ax = px(PAD.x, PAD.y), ay = py(PAD.x, PAD.y, 0.4);
        const gx = px(GS.x, GS.y), gy = py(GS.x, GS.y, 0) - 10;
        ctx.setLineDash([3, 4]);
        ctx.strokeStyle = 'rgba(255,255,255,0.45)';
        ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(gx, gy); ctx.stroke();
        ctx.setLineDash([]);
        const ang = Math.atan2(gy - ay, gx - ax);
        for (let k = 0; k < 3; k++) {
          const f = (clock * 1.4 + k / 3) % 1;
          ctx.beginPath();
          ctx.arc(ax, ay, 6 + f * 22, ang - 0.6, ang + 0.6);
          ctx.strokeStyle = `rgba(255,255,255,${0.85 * (1 - f)})`;
          ctx.lineWidth = 2;
          ctx.stroke();
        }
        const prog = Math.min(1, (t - tOffload) / (tEnd - tOffload));
        const bw = 128, bx = gx - 20, by = gy - 48;
        ctx.fillStyle = 'rgba(0,0,0,0.85)';
        ctx.strokeStyle = 'rgba(255,255,255,0.45)';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.rect(bx - 10, by - 15, bw + 20, 38); ctx.fill(); ctx.stroke();
        text(`OFFLOADING LOG  ${Math.round(prog * 100)}%`, bx, by - 3, 10, 600, '#ffffff', 'left', 1.2);
        ctx.fillStyle = 'rgba(255,255,255,0.15)';
        ctx.fillRect(bx, by + 9, bw, 3);
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(bx, by + 9, bw * prog, 3);
      }

      // Aircraft.
      if (fadeOut > 0) {
        let d = st.h - heading;
        d = Math.atan2(Math.sin(d), Math.cos(d));
        heading += dt > 0 ? d * Math.min(1, dt * 10) : d;
        const airborne = st.z > 0.05;

        if (airborne) {
          screen();
          ctx.setLineDash([2, 4]);
          ctx.strokeStyle = 'rgba(255,255,255,0.25)';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(px(st.x, st.y), py(st.x, st.y, 0));
          ctx.lineTo(px(st.x, st.y), py(st.x, st.y, st.z));
          ctx.stroke();
          ctx.setLineDash([]);
          plane(st.x, st.y, 0);
          ctx.rotate(heading);
          ctx.scale(SPR, SPR);
          aircraft(st, clock, true);
        }
        if (airborne && st.push > 0.5) {
          // Forward radar field of view.
          plane(st.x, st.y, st.z);
          ctx.rotate(heading);
          const R = 3.0;
          const g = ctx.createRadialGradient(0, 0, 0.3, 0, 0, R);
          g.addColorStop(0, 'rgba(255,255,255,0.16)');
          g.addColorStop(1, 'rgba(255,255,255,0)');
          ctx.fillStyle = g;
          ctx.beginPath();
          ctx.moveTo(0, 0);
          ctx.arc(0, 0, R, -0.6, 0.6);
          ctx.closePath();
          ctx.fill();
        }
        if (inSurvey) {
          plane(st.x, st.y, st.z);
          const g = ctx.createRadialGradient(0, 0, 0, 0, 0, 1.1);
          g.addColorStop(0, 'rgba(255,255,255,0.32)');
          g.addColorStop(1, 'rgba(255,255,255,0)');
          ctx.fillStyle = g;
          ctx.beginPath(); ctx.arc(0, 0, 1.1, 0, Math.PI * 2); ctx.fill();
        }

        plane(st.x, st.y, st.z + Math.min(0.35, st.z) + 0.05);
        ctx.rotate(heading);
        ctx.scale(SPR, SPR);
        ctx.globalAlpha = fadeOut;
        aircraft(st, clock, false);
        ctx.globalAlpha = 1;
      }
      screen();
    }

    // ---- HUD (webcast-style overlay) ----
    function phaseText(st, t) {
      if (t >= tEnd) return `3D COVERAGE MAP READY  ·  ${ALTS[LAYERS - 1 - (Math.floor((t - tEnd) / SLICE) % LAYERS)]} M SLICE`;
      const ph = st.phase;
      if (ph.startsWith('survey')) return `SURVEYING  ·  LAYER ${+ph.slice(-1) + 1} OF ${LAYERS}  ·  ${ALTS[st.layer]} M`;
      if (ph.startsWith('descend')) return `GLIDING DOWN TO ${ALTS[st.layer]} M`;
      return {
        preflight: 'PRE-FLIGHT CHECKS  ·  SPOOLING UP',
        takeoff: 'VERTICAL TAKEOFF',
        transition: 'TRANSITION TO WING-BORNE FLIGHT',
        climb: 'CLIMBING TO 100 M AS A PLANE',
        rth: 'RETURNING HOME',
        land: 'TRANSITION TO HOVER  ·  LANDING',
        offload: 'OFFLOADING DATA OVER WIFI',
      }[ph];
    }

    function gauge(cx, cy, r, frac, value, unit, label) {
      const a0 = Math.PI * 0.75, sweep = Math.PI * 1.5;
      ctx.lineCap = 'butt';
      ctx.lineWidth = 3;
      ctx.strokeStyle = 'rgba(255,255,255,0.18)';
      ctx.beginPath(); ctx.arc(cx, cy, r, a0, a0 + sweep); ctx.stroke();
      ctx.strokeStyle = '#ffffff';
      ctx.beginPath(); ctx.arc(cx, cy, r, a0, a0 + sweep * Math.max(0.001, Math.min(1, frac))); ctx.stroke();
      text(value, cx, cy - 2, Math.round(r * 0.52), 600, '#ffffff', 'center');
      text(unit, cx, cy + r * 0.42, 9, 600, 'rgba(255,255,255,0.6)', 'center', 1.2);
      text(label, cx, cy + r + 13, 10, 600, 'rgba(255,255,255,0.75)', 'center', 2);
    }

    function drawHud(t, st, clock) {
      const top = H - L.bar;
      const g = ctx.createLinearGradient(0, top - 70, 0, H);
      g.addColorStop(0, 'rgba(0,0,0,0)');
      g.addColorStop(1, 'rgba(0,0,0,0.88)');
      ctx.fillStyle = g;
      ctx.fillRect(0, top - 70, W, L.bar + 70);

      const cx = W / 2;
      const c = L.compact;
      const missionSec = Math.round(Math.min(t, tEnd) * TIME_SCALE);
      const clockStr = `T+ ${pad2(Math.floor(missionSec / 3600))}:${pad2(Math.floor(missionSec / 60) % 60)}:${pad2(missionSec % 60)}`;
      text(phaseText(st, t), cx, top + (c ? 14 : 22), c ? 9.5 : 11, 600, 'rgba(255,255,255,0.72)', 'center', c ? 1.2 : 2);
      text(clockStr, cx, top + (c ? 40 : 52), c ? 22 : 30, 600, '#ffffff', 'center', 1);

      // Timeline: events evenly spaced, progress interpolated within each event.
      const tw = Math.min(W * (c ? 0.86 : 0.44), 640);
      const x0 = cx - tw / 2, ty = top + (c ? 76 : 106);
      const N = events.length;
      let k = N - 1;
      while (k > 0 && t < events[k].t) k--;
      const segEnd = k + 1 < N ? events[k + 1].t : tEnd;
      const frac = t >= tEnd ? 1 : (k + Math.min(1, (t - events[k].t) / (segEnd - events[k].t))) / N;
      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(255,255,255,0.22)';
      ctx.beginPath(); ctx.moveTo(x0, ty); ctx.lineTo(x0 + tw, ty); ctx.stroke();
      ctx.strokeStyle = '#ffffff';
      ctx.beginPath(); ctx.moveTo(x0, ty); ctx.lineTo(x0 + tw * frac, ty); ctx.stroke();
      events.forEach((ev, n) => {
        const ex = x0 + (n / N) * tw;
        const passed = t >= ev.t;
        const current = passed && (n === N - 1 ? t < tEnd : t < events[n + 1].t);
        if (current) {
          const f = (clock * 1.2) % 1;
          ctx.beginPath(); ctx.arc(ex, ty, 4 + f * 8, 0, Math.PI * 2);
          ctx.strokeStyle = `rgba(255,255,255,${0.6 * (1 - f)})`;
          ctx.lineWidth = 1.5; ctx.stroke();
        }
        ctx.beginPath(); ctx.arc(ex, ty, c ? 3 : 4, 0, Math.PI * 2);
        ctx.fillStyle = passed ? '#ffffff' : '#000000';
        ctx.fill();
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = passed ? '#ffffff' : 'rgba(255,255,255,0.45)';
        ctx.stroke();
        if (L.labels) {
          const above = n % 2 === 0;
          text(ev.label, ex, ty + (above ? -15 : 17), 9.5, 600,
            current ? '#ffffff' : passed ? 'rgba(255,255,255,0.7)' : 'rgba(255,255,255,0.38)', 'center', 1.2);
        }
      });
      // End marker: map ready.
      ctx.fillStyle = t >= tEnd ? '#ffffff' : 'rgba(255,255,255,0.45)';
      ctx.fillRect(x0 + tw - 3, ty - 3, 6, 6);

      if (L.gauges) {
        const r = 34, gy = top + 68;
        gauge(64, gy, r, st.spd * 3.6 / 80, String(Math.round(st.spd * 3.6)), 'KM/H', 'SPEED');
        gauge(160, gy, r, (st.z / ZS) / 120, String(Math.max(0, Math.round(st.z / ZS))), 'M', 'ALTITUDE');
        gauge(W - 160, gy, r, st.batt / 100, String(Math.round(st.batt)), '%', 'BATTERY');
        const pts = countRevealed(t);
        gauge(W - 64, gy, r, pts / sortedReveal.length, String(pts), 'POINTS', 'LOGGED');
      }

      if (L.legend) {
        const lx = W - 28, ly = o.layout === 'video' ? 36 : 104;
        text('RSSI', lx - 150, ly, 10, 600, 'rgba(255,255,255,0.75)', 'left', 2);
        const grd = ctx.createLinearGradient(lx - 150, 0, lx, 0);
        STOPS.forEach(([p, col]) => grd.addColorStop(p, `rgb(${col.join(',')})`));
        ctx.fillStyle = grd;
        ctx.fillRect(lx - 150, ly + 12, 150, 4);
        text('WEAK', lx - 150, ly + 28, 9, 600, 'rgba(255,255,255,0.5)', 'left', 1.2);
        text('STRONG', lx, ly + 28, 9, 600, 'rgba(255,255,255,0.5)', 'right', 1.2);
        if (o.layout !== 'video') text('SIMULATED MISSION', lx, ly + 48, 9, 600, 'rgba(255,255,255,0.4)', 'right', 1.2);
      }

      if (o.layout === 'video') {
        let tx = 34;
        if (o.logo) {
          const lh = 44, lw = lh * o.logo.width / o.logo.height;
          ctx.drawImage(o.logo, 34, 38, lw, lh);
          tx = 34 + lw + 16;
        }
        text('ORNECHO', tx, 38, 10, 600, 'rgba(255,255,255,0.6)', 'left', 3);
        text('RS-1 SURVEYOR', tx, 62, 24, 700, '#ffffff', 'left', 1.5);
        text('MISSION PROFILE  ·  SIMULATION', tx, 86, 10, 600, 'rgba(255,255,255,0.6)', 'left', 2.5);
      }
    }

    // ---- Public API ----
    function render(t, dt, clock) {
      screen();
      if (o.background) { ctx.fillStyle = o.background; ctx.fillRect(0, 0, W, H); }
      else ctx.clearRect(0, 0, W, H);
      if (!W) return;
      const st = stateAt(t);
      drawScene(t, dt || 0, clock || 0, st);
      if (o.hud && o.layout !== 'panel') drawHud(t, st, clock || 0);
    }

    function renderMap(slice, clock) {
      screen();
      if (o.background) { ctx.fillStyle = o.background; ctx.fillRect(0, 0, W, H); }
      else ctx.clearRect(0, 0, W, H);
      if (!W) return;
      drawScene(0, 0, clock || 0, null, slice);
    }

    function reset() {
      makeField();
      heading = 0;
    }

    makeField();

    return {
      resize, render, renderMap, reset,
      setHeadingAt: (t) => { heading = stateAt(t).h; },
      get missionEnd() { return tEnd; },
      get cycle() { return tEnd + HOLD + FADE; },
      get surveyMid() {
        const [a, b] = layerRange[1];
        return lerp(P[a].t, P[b].t, 0.55);
      },
    };
  }

  return { create, LAYERS };
});
