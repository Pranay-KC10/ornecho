// Ornecho — RS-1 site: navigation, units toggle, the hero mission replay and the map panel.
// The drawing itself lives in mission.js (shared with tools/render-mission.js).
(function () {
  'use strict';

  // ---- Navigation -------------------------------------------------------
  const nav = document.querySelector('.nav');
  const toggle = document.querySelector('.nav__toggle');
  const menu = document.getElementById('nav-menu');

  const onScroll = () => nav.classList.toggle('is-scrolled', window.scrollY > 40);
  window.addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  function setMenu(open) {
    toggle.setAttribute('aria-expanded', String(open));
    toggle.setAttribute('aria-label', open ? 'Close menu' : 'Open menu');
    menu.classList.toggle('is-open', open);
  }
  toggle.addEventListener('click', () => setMenu(toggle.getAttribute('aria-expanded') !== 'true'));
  menu.addEventListener('click', (e) => { if (e.target.closest('a')) setMenu(false); });
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape') setMenu(false); });

  // ---- Metric / US toggle -----------------------------------------------
  const unitBtns = document.querySelectorAll('.units__btn');
  unitBtns.forEach((btn) => btn.addEventListener('click', () => {
    const key = btn.dataset.units === 'us' ? 'us' : 'metric';
    unitBtns.forEach((b) => {
      const on = b === btn;
      b.classList.toggle('is-on', on);
      b.setAttribute('aria-pressed', String(on));
    });
    document.querySelectorAll('[data-metric]').forEach((el) => { el.textContent = el.dataset[key]; });
  }));

  const year = document.getElementById('year');
  if (year) year.textContent = String(new Date().getFullYear());

  if (!window.RSMission) return;
  const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)');

  // Size a canvas's backing store to its CSS box.
  function fit(canvas) {
    const r = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.max(1, Math.round(r.width * dpr));
    canvas.height = Math.max(1, Math.round(r.height * dpr));
    return [r.width, r.height, dpr];
  }

  function watch(canvas, onResize, onVisible) {
    if ('ResizeObserver' in window) new ResizeObserver(onResize).observe(canvas);
    else window.addEventListener('resize', onResize);
    if ('IntersectionObserver' in window) {
      new IntersectionObserver((entries) => onVisible(entries[0].isIntersecting)).observe(canvas);
    } else {
      onVisible(true);
    }
  }

  // ---- Hero: live mission replay ---------------------------------------
  const heroCanvas = document.getElementById('mission-canvas');
  if (heroCanvas) {
    const mission = RSMission.create(heroCanvas.getContext('2d'), { layout: 'hero' });
    const SPEED = 1.15;
    let t = 0, last = 0, raf = 0, running = false, visible = true;

    // Preview a given moment of the mission, e.g. index.html?t=30
    const tParam = parseFloat(new URLSearchParams(location.search).get('t'));
    if (tParam > 0) t = tParam;

    const still = () => {
      if (reduceMotion.matches) t = mission.surveyMid;
      mission.setHeadingAt(t);
      mission.render(t, 0, performance.now() / 1000);
    };
    const frame = (ts) => {
      const dt = last ? Math.min(0.05, (ts - last) / 1000) * SPEED : 0;
      last = ts;
      t += dt;
      if (t > mission.cycle) { t = 0; mission.reset(); }
      mission.render(t, dt, ts / 1000);
      raf = requestAnimationFrame(frame);
    };
    const start = () => {
      if (running || reduceMotion.matches || !visible) return;
      running = true;
      last = 0;
      raf = requestAnimationFrame(frame);
    };
    const stop = () => { running = false; cancelAnimationFrame(raf); };

    watch(heroCanvas, () => {
      const [w, h, d] = fit(heroCanvas);
      mission.resize(w, h, d);
      if (!running) still();
    }, (v) => { visible = v; if (v) start(); else stop(); });

    if (reduceMotion.addEventListener) {
      reduceMotion.addEventListener('change', () => { if (reduceMotion.matches) { stop(); still(); } else start(); });
    }
  }

  // ---- Map panel: the finished map, one altitude slice at a time --------
  const mapCanvas = document.getElementById('map-canvas');
  if (mapCanvas) {
    const map = RSMission.create(mapCanvas.getContext('2d'), { layout: 'panel', hud: false, explode: 1.5, seed: 9 });
    let slice = RSMission.LAYERS - 1, timer = 0;
    const draw = () => map.renderMap(slice, 0.4);

    watch(mapCanvas, () => {
      const [w, h, d] = fit(mapCanvas);
      map.resize(w, h, d);
      draw();
    }, (v) => {
      clearInterval(timer);
      if (v && !reduceMotion.matches) {
        timer = setInterval(() => { slice = (slice + RSMission.LAYERS - 1) % RSMission.LAYERS; draw(); }, 2200);
      }
    });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(draw);
  }
})();
