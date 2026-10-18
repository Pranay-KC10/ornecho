#!/usr/bin/env node
// Renders the RS-1 mission replay (js/mission.js) to MP4 and GIF for presentations.
// Needs node-canvas and ffmpeg:
//   npm install canvas            (in any folder; point NODE_PATH at its node_modules)
//   NODE_PATH=/path/to/node_modules node tools/render-mission.js --out rs1_mission --fonts ./fonts
// Options: --width 1280 --height 720 --fps 20 --speed 1.6 --gif-width 1280 --still <seconds>
'use strict';

const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const { once } = require('events');
const { createCanvas, registerFont, loadImage } = require('canvas');
const RSMission = require('../js/mission.js');

const opt = { out: 'rs1_mission', width: 1280, height: 720, fps: 20, speed: 1.6, 'gif-width': 1280, 'gif-fps': 15 };
for (let i = 2; i < process.argv.length; i += 2) opt[process.argv[i].replace(/^--/, '')] = process.argv[i + 1];
const W = +opt.width, H = +opt.height, FPS = +opt.fps, SPEED = +opt.speed;

// Barlow is the site font; register it if a folder of TTFs is given.
if (opt.fonts) {
  for (const [file, weight] of [['Regular', 'normal'], ['Medium', '500'], ['SemiBold', '600'], ['Bold', '700']]) {
    const f = path.join(opt.fonts, `Barlow-${file}.ttf`);
    if (fs.existsSync(f)) registerFont(f, { family: 'Barlow', weight });
  }
}

const canvas = createCanvas(W, H);
const ctx = canvas.getContext('2d');
let mission;

function run(cmd, args) {
  return new Promise((resolve, reject) => {
    const p = spawn(cmd, args, { stdio: ['ignore', 'ignore', 'inherit'] });
    p.on('close', (code) => (code === 0 ? resolve() : reject(new Error(`${cmd} exited ${code}`))));
  });
}

async function main() {
  const logo = await loadImage(path.join(__dirname, '..', 'assets', 'ornecho-mark.png'));
  mission = RSMission.create(ctx, { layout: 'video', background: '#000000', createCanvas, logo });
  mission.resize(W, H, 1);

  if (opt.still !== undefined) {
    const t = +opt.still;
    mission.setHeadingAt(t);
    mission.render(t, 0, t);
    fs.writeFileSync(`${opt.out}.png`, canvas.toBuffer('image/png'));
    console.log(`wrote ${opt.out}.png`);
    return;
  }

  const frames = Math.ceil((mission.cycle / SPEED) * FPS);
  console.log(`rendering ${frames} frames (${(frames / FPS).toFixed(1)} s) at ${W}x${H}, ${SPEED}x speed`);
  const ff = spawn('ffmpeg', ['-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgra', '-s', `${W}x${H}`,
    '-r', String(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '14', '-pix_fmt', 'yuv420p',
    '-movflags', '+faststart', `${opt.out}.mp4`], { stdio: ['pipe', 'ignore', 'inherit'] });

  for (let n = 0; n < frames; n++) {
    const t = (n / FPS) * SPEED;
    mission.render(t, SPEED / FPS, n / FPS);
    if (!ff.stdin.write(canvas.toBuffer('raw'))) await once(ff.stdin, 'drain');
  }
  ff.stdin.end();
  await once(ff, 'close');
  console.log(`wrote ${opt.out}.mp4`);

  const gw = +opt['gif-width'];
  await run('ffmpeg', ['-y', '-loglevel', 'error', '-i', `${opt.out}.mp4`, '-vf',
    `fps=${opt['gif-fps']},scale=${gw}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160:stats_mode=diff[p];` +
    '[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle', '-loop', '0', `${opt.out}.gif`]);
  console.log(`wrote ${opt.out}.gif`);
}

main().catch((e) => { console.error(e); process.exit(1); });
