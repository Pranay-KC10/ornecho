# Ornecho — RS-1 Surveyor website

Product site for the RS-1 Surveyor, Ornecho's autonomous VTOL drone for 3D RF coverage mapping.
Plain HTML/CSS/JS with no build step. The only external dependency is Google Fonts (Barlow).

```
index.html               all page content
css/style.css            styles; colours and fonts are tokens at the top (:root)
js/mission.js            mission replay engine (canvas), shared by the site and the video renderer
js/main.js               nav, metric/US toggle, hero loop, map panel
tools/render-mission.js  renders the mission replay to MP4 + GIF (Node)
assets/                  Ornecho logo (white knockout for the dark site, original for light backgrounds), favicons
```

## Preview

Open `index.html` in a browser, or serve the folder. Add `?t=30` to the URL to jump the hero animation to a given second.

```
python3 -m http.server 8000   # then open http://localhost:8000
```

## Presentation video / GIF

Needs ffmpeg and node-canvas. Install node-canvas outside this repo and point `NODE_PATH` at it:

```
mkdir -p /tmp/rs1 && (cd /tmp/rs1 && npm install canvas)
# optional: Barlow TTFs in a folder for the exact site font (github.com/google/fonts/tree/main/ofl/barlow)
NODE_PATH=/tmp/rs1/node_modules node tools/render-mission.js --out rs1_mission --speed 1.6 --fonts /path/to/barlow
```

Options: `--width 1280 --height 720 --fps 20 --speed 1.6 --gif-width 1280 --still <seconds>` (a single PNG frame).

## Before publishing

- Replace `team@example.com` in the Contact section of `index.html` with a real address.
- Specs marked `target` / `est.` are not yet measured. Update them after flight tests.

## GitHub Pages

`index.html` is at the repo root, so Pages can serve it directly:
Settings → Pages → Build and deployment → Deploy from a branch → `main` / `(root)`.
