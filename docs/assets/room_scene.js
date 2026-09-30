/* RoboMNIST room scene. Robot and path coordinates are measured, robot-local data.
   Furniture, walls and sensor stand provide illustrative context only. */
(function () {
  'use strict';

  const C = {
    background: '#081826', floor: '#10273a', wall: '#173449', grid: '#2b5368',
    table: '#a6b5bf', tableEdge: '#4d6474', leg: '#263b49', base: '#243946',
    robot: '#e6edf0', joint: '#2a3945', path: '#6b8ba2', writing: '#62e2e8',
    gold: '#ffd077', text: '#b8cedd'
  };

  // Paper-scale schematic footprint, expressed relative to the active robot's
  // local base. The paper does not provide a base-to-room calibration.
  const FOOTPRINT = { x0: -1.2, x1: 2.4, y0: -1.65, y1: 1.65, floor: -0.75 };
  const TABLE = { x0: -0.35, x1: 0.55, y0: -1.55, y1: 0.25, top: -0.065 };

  const ROOMS = {
    detail: {
      x: [-0.24, 1.03], y: [-0.55, 0.57], z: [-0.18, 1.20],
      aspect: { x: 1.27, y: 1.12, z: 1.38 },
      camera: { eye: { x: 1.65, y: -1.65, z: 1.13 }, up: { x: 0, y: 0, z: 1 } }
    },
    room: {
      x: [-1.32, 2.52], y: [-1.79, 1.79], z: [-0.83, 1.76],
      aspect: { x: 1.91, y: 1.79, z: 1.30 },
      camera: { eye: { x: 1.85, y: -1.88, z: 1.38 }, up: { x: 0, y: 0, z: 1 } }
    }
  };

  const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const vmul = (a, n) => [a[0] * n, a[1] * n, a[2] * n];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = a => Math.hypot(a[0], a[1], a[2]);
  const unit = a => vmul(a, 1 / (norm(a) || 1));

  function box(x0, x1, y0, y1, z0, z1, color, opacity, name) {
    return {
      type: 'mesh3d', name, hoverinfo: 'skip', showscale: false,
      x: [x0, x1, x1, x0, x0, x1, x1, x0],
      y: [y0, y0, y1, y1, y0, y0, y1, y1],
      z: [z0, z0, z0, z0, z1, z1, z1, z1],
      i: [0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3],
      j: [1, 2, 6, 7, 1, 5, 2, 6, 3, 7, 0, 4],
      k: [2, 3, 5, 6, 5, 4, 6, 5, 7, 6, 4, 7],
      color, opacity, flatshading: true,
      lighting: { ambient: 0.67, diffuse: 0.65, specular: 0.22, roughness: 0.68 },
      lightposition: { x: 1.6, y: -2.2, z: 3.3 }
    };
  }

  function roomGrid() {
    const x = [], y = [], z = [];
    const line = (a, b) => { x.push(a[0], b[0], null); y.push(a[1], b[1], null); z.push(a[2], b[2], null); };
    // The two walls and floor are deliberately wireframe: they establish room
    // depth without obscuring the actual measured movement.
    for (let gx = FOOTPRINT.x0; gx <= FOOTPRINT.x1 + 0.001; gx += 0.3)
      line([gx, FOOTPRINT.y0, FOOTPRINT.floor + 0.015], [gx, FOOTPRINT.y1, FOOTPRINT.floor + 0.015]);
    for (let gy = FOOTPRINT.y0; gy <= FOOTPRINT.y1 + 0.001; gy += 0.3)
      line([FOOTPRINT.x0, gy, FOOTPRINT.floor + 0.015], [FOOTPRINT.x1, gy, FOOTPRINT.floor + 0.015]);
    for (let gx = FOOTPRINT.x0; gx <= FOOTPRINT.x1 + 0.001; gx += 0.45)
      line([gx, FOOTPRINT.y1, FOOTPRINT.floor], [gx, FOOTPRINT.y1, 1.62]);
    for (let gz = FOOTPRINT.floor; gz <= 1.65; gz += 0.3)
      line([FOOTPRINT.x0, FOOTPRINT.y1, gz], [FOOTPRINT.x1, FOOTPRINT.y1, gz]);
    for (let gy = FOOTPRINT.y0; gy <= FOOTPRINT.y1 + 0.001; gy += 0.45)
      line([FOOTPRINT.x0, gy, FOOTPRINT.floor], [FOOTPRINT.x0, gy, 1.62]);
    for (let gz = FOOTPRINT.floor; gz <= 1.65; gz += 0.3)
      line([FOOTPRINT.x0, FOOTPRINT.y0, gz], [FOOTPRINT.x0, FOOTPRINT.y1, gz]);
    return { type: 'scatter3d', mode: 'lines', x, y, z,
      line: { color: C.grid, width: 1 }, opacity: 0.33,
      name: 'Illustrative room grid', hoverinfo: 'skip', showlegend: false };
  }

  function cylinder(points, radii) {
    // All link meshes share one trace. Stable triangle indices allow fast XYZ
    // updates as the recorded seven-joint pose changes.
    const x = [], y = [], z = [], i = [], j = [], k = [], vertexcolor = [];
    const sides = 12;
    for (let link = 0; link < points.length - 1; link++) {
      const rawA = points[link], rawB = points[link + 1];
      const displacement = vsub(rawB, rawA);
      const length = norm(displacement);
      // Panda joint centers can coincide. Keep a tiny finite collar in those
      // cases, preserving the mesh topology while the joint marker covers it.
      const a = length > 1e-7 ? rawA : vadd(rawA, [0, 0, -0.002]);
      const b = length > 1e-7 ? rawB : vadd(rawA, [0, 0, 0.002]);
      const direction = length > 1e-7 ? vmul(displacement, 1 / length) : [0, 0, 1];
      const radial = cross(direction, [0, 0, 1]);
      const radialA = norm(radial) > 1e-7 ? unit(radial) : unit(cross(direction, [0, 1, 0]));
      const radialB = unit(cross(direction, radialA));
      const radius = radii[link] || 0.034;
      const base = x.length;
      for (let ring = 0; ring < 2; ring++) {
        const center = ring ? b : a;
        for (let s = 0; s < sides; s++) {
          const angle = (s / sides) * Math.PI * 2;
          const offset = vadd(vmul(radialA, radius * Math.cos(angle)), vmul(radialB, radius * Math.sin(angle)));
          const p = vadd(center, offset);
          x.push(p[0]); y.push(p[1]); z.push(p[2]);
          const tone = Math.cos(angle - 0.72) > 0.15 ? '#f0f4f5' : '#b7c5cf';
          vertexcolor.push(link === 7 ? '#334b59' : tone);
        }
      }
      for (let s = 0; s < sides; s++) {
        const n = (s + 1) % sides;
        i.push(base + s, base + n);
        j.push(base + n, base + sides + n);
        k.push(base + sides + s, base + sides + s);
      }
    }
    return { type: 'mesh3d', name: 'Articulated Panda arm', x, y, z, i, j, k,
      vertexcolor, opacity: 1, hoverinfo: 'skip', showlegend: false,
      flatshading: false,
      lighting: { ambient: 0.73, diffuse: 0.72, specular: 0.34, roughness: 0.43, fresnel: 0.1 },
      lightposition: { x: 1.8, y: -2.5, z: 3.7 } };
  }

  function axis(title, range) {
    return { title: { text: title, font: { color: C.text, size: 11 } },
      range, tickfont: { color: '#86a5b8', size: 10 }, nticks: 4,
      showbackground: false, showgrid: false, zeroline: false,
      showspikes: false, showline: true, linecolor: '#476778', linewidth: 2 };
  }

  function measuredPath(d, cut) {
    const x = [], y = [], z = [];
    for (let idx = 0; idx <= cut; idx++) {
      const writing = !d.writing || Boolean(d.writing[idx]);
      x.push(writing ? d.x[idx] : null);
      y.push(writing ? d.raw_y[idx] : null);
      z.push(writing ? d.z[idx] : null);
    }
    return { x, y, z };
  }

  function pointValid(p) {
    return Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
  }

  async function init(element, d) {
    if (!element || typeof element !== 'object') throw new Error('RoboRoom needs a plot element');
    if (!window.Plotly) throw new Error('Plotly was not loaded for the room scene');
    const n = d && d.x && d.x.length;
    if (!n || !d.raw_y || !d.z || !d.joint_points ||
        d.raw_y.length !== n || d.z.length !== n || d.joint_points.length !== n ||
        !Array.isArray(d.joint_points[0]) || d.joint_points[0].length !== 9 ||
        !d.joint_points[0].every(pointValid)) {
      throw new Error('RoboRoom received incomplete measured arm data');
    }

    const trace = [];
    const add = t => { trace.push(t); return trace.length - 1; };
    add(box(FOOTPRINT.x0, FOOTPRINT.x1, FOOTPRINT.y0, FOOTPRINT.y1,
      FOOTPRINT.floor - 0.035, FOOTPRINT.floor, C.floor, 0.97,
      'Schematic 3.60 × 3.30 m room floor'));
    add(box(FOOTPRINT.x0, FOOTPRINT.x1, FOOTPRINT.y1 - 0.018, FOOTPRINT.y1,
      FOOTPRINT.floor, 1.62, C.wall, 0.12, 'Schematic rear wall'));
    add(box(FOOTPRINT.x0, FOOTPRINT.x0 + 0.018, FOOTPRINT.y0, FOOTPRINT.y1,
      FOOTPRINT.floor, 1.62, C.wall, 0.12, 'Schematic side wall'));
    add(roomGrid());
    add({ type: 'scatter3d', mode: 'lines',
      x: [FOOTPRINT.x0, FOOTPRINT.x1, null, FOOTPRINT.x0 + 0.07, FOOTPRINT.x0 + 0.07],
      y: [FOOTPRINT.y0 + 0.08, FOOTPRINT.y0 + 0.08, null, FOOTPRINT.y0, FOOTPRINT.y1],
      z: Array(5).fill(FOOTPRINT.floor + 0.04),
      line: { color: '#7fc7d0', width: 4 }, opacity: 0.8,
      hoverinfo: 'skip', showlegend: false, name: 'Paper-scale room dimensions' });

    // The 0.90 × 1.80 m table is paper-scale. The two illustrative bases are
    // placed near opposite ends; their common room transform is not measured.
    add(box(TABLE.x0, TABLE.x1, TABLE.y0, TABLE.y1,
      -0.115, TABLE.top, C.table, 1, 'Schematic 0.90 × 1.80 m tabletop'));
    add(box(TABLE.x0 - 0.015, TABLE.x1 + 0.015, TABLE.y0 - 0.015, TABLE.y1 + 0.015,
      -0.137, -0.108, C.tableEdge, 1, 'Table rim'));
    for (const tx of [-0.275, 0.475]) for (const ty of [-1.47, 0.17]) {
      add(box(tx - 0.035, tx + 0.035, ty - 0.035, ty + 0.035,
        -0.73, -0.14, C.leg, 1, 'Table support'));
    }
    add(box(-0.12, 0.12, -0.12, 0.12, -0.065, 0.005, C.base, 1, 'Robot base plate'));

    const ghostOffsetY = -1.30;
    add(box(-0.12, 0.12, ghostOffsetY - 0.12, ghostOffsetY + 0.12,
      -0.065, 0.005, '#496273', 0.72, 'Schematic second robot base'));
    const ghostPoints = d.joint_points[0].map(p => [p[0], p[1] + ghostOffsetY, p[2]]);
    const ghost = cylinder(ghostPoints, [0.068, 0.057, 0.053, 0.051, 0.045, 0.042, 0.039, 0.025]);
    ghost.name = 'Static second arm · schematic, uncalibrated placement';
    ghost.vertexcolor = ghost.vertexcolor.map(() => '#7793a4');
    ghost.opacity = 0.39;
    add(ghost);
    add({ type: 'scatter3d', mode: 'text', x: [0.08], y: [-1.27], z: [0.99],
      text: ['ARM 2 · SCHEMATIC'], textfont: { color: '#9fbed0', size: 11 },
      hoverinfo: 'skip', showlegend: false, name: 'Second-arm context label' });

    // Three illustrative module stands, spaced 0.90 m centre-to-centre.
    const moduleY = [-0.9, 0, 0.9];
    for (const my of moduleY) {
      add(box(1.64, 1.685, my - 0.022, my + 0.022,
        FOOTPRINT.floor, 0.67, C.leg, 1, 'Schematic sensor stand'));
      add(box(1.55, 1.78, my - 0.085, my + 0.085,
        0.61, 0.70, '#324f61', 1, 'Schematic stereo sensor module'));
    }
    add({ type: 'scatter3d', mode: 'markers',
      x: [1.57, 1.74, 1.57, 1.74, 1.57, 1.74],
      y: [-0.985, -0.985, -0.085, -0.085, 0.815, 0.815],
      z: Array(6).fill(0.66),
      marker: { size: 5, color: C.writing, line: { color: '#ffffff', width: 1 } },
      hoverinfo: 'skip', showlegend: false, name: 'Schematic sensor lenses' });
    add({ type: 'scatter3d', mode: 'text', x: [1.78, 1.78, 1.78],
      y: moduleY, z: [0.82, 0.82, 0.82],
      text: ['MODULE 1', 'MODULE 2', 'MODULE 3'],
      textfont: { color: '#a4dce5', size: 10 },
      hoverinfo: 'skip', showlegend: false, name: 'Schematic module labels' });

    const farX = [...d.x].sort((a, b) => a - b)[Math.floor((n - 1) * 0.98)];
    const planeX = Math.max(0.52, Math.min(0.85, farX));
    add({ type: 'mesh3d', x: [planeX, planeX, planeX, planeX],
      y: [-0.34, 0.34, 0.34, -0.34], z: [0.25, 0.25, 0.96, 0.96],
      i: [0, 0], j: [1, 2], k: [2, 3], color: '#3ddadd', opacity: 0.085,
      hoverinfo: 'skip', showlegend: false, name: 'Approximate writing plane' });
    add({ type: 'scatter3d', mode: 'lines',
      x: [planeX, planeX, planeX, planeX, planeX],
      y: [-0.34, 0.34, 0.34, -0.34, -0.34],
      z: [0.25, 0.25, 0.96, 0.96, 0.25],
      line: { color: '#55d9dd', width: 2 }, opacity: 0.46,
      hoverinfo: 'skip', showlegend: false, name: 'Writing-plane outline' });
    add({ type: 'scatter3d', mode: 'lines', x: d.x, y: d.raw_y, z: d.z,
      line: { color: C.path, width: 4 }, opacity: 0.35,
      hoverinfo: 'skip', showlegend: false, name: 'Full measured EE path' });

    const revealedIndex = add({ type: 'scatter3d', mode: 'lines', x: [], y: [], z: [],
      line: { color: C.writing, width: 9 }, opacity: 0.95,
      hoverinfo: 'skip', showlegend: false, name: 'Revealed writing trajectory' });
    const linkIndex = add(cylinder(d.joint_points[0], [0.068, 0.057, 0.053, 0.051, 0.045, 0.042, 0.039, 0.025]));
    const jointIndex = add({ type: 'scatter3d', mode: 'markers',
      x: d.joint_points[0].slice(1, 8).map(p => p[0]),
      y: d.joint_points[0].slice(1, 8).map(p => p[1]),
      z: d.joint_points[0].slice(1, 8).map(p => p[2]),
      marker: { size: 10, color: C.joint, line: { color: '#aab9c4', width: 1 } },
      hoverinfo: 'skip', showlegend: false, name: 'FK joint centers from measured angles' });
    const eeIndex = add({ type: 'scatter3d', mode: 'markers',
      x: [d.x[0]], y: [d.raw_y[0]], z: [d.z[0]],
      marker: { size: 11, color: C.gold, line: { color: '#fff9e9', width: 2 } },
      hoverinfo: 'skip', showlegend: false, name: 'Current measured end effector' });

    const initial = ROOMS.detail;
    const layout = {
      paper_bgcolor: C.background, plot_bgcolor: C.background,
      margin: { l: 0, r: 0, t: 0, b: 0 }, showlegend: false,
      uirevision: 'room-camera',
      scene: {
        xaxis: axis('Robot X · m', initial.x),
        yaxis: axis('Robot Y · m', initial.y),
        zaxis: axis('Robot Z · m', initial.z),
        aspectmode: 'manual', aspectratio: initial.aspect,
        camera: initial.camera, dragmode: 'orbit'
      }
    };
    await Plotly.newPlot(element, trace, layout, {
      responsive: true, displaylogo: false, displayModeBar: false, scrollZoom: false
    });

    let disposed = false, pending = null, raf = 0;
    function draw() {
      raf = 0;
      if (disposed || pending === null) return;
      const idx = pending;
      pending = null;
      const pts = d.joint_points[idx];
      const mesh = cylinder(pts, [0.068, 0.057, 0.053, 0.051, 0.045, 0.042, 0.039, 0.025]);
      const measured = measuredPath(d, idx);
      element.data[revealedIndex].x = measured.x;
      element.data[revealedIndex].y = measured.y;
      element.data[revealedIndex].z = measured.z;
      element.data[linkIndex].x = mesh.x;
      element.data[linkIndex].y = mesh.y;
      element.data[linkIndex].z = mesh.z;
      element.data[jointIndex].x = pts.slice(1, 8).map(p => p[0]);
      element.data[jointIndex].y = pts.slice(1, 8).map(p => p[1]);
      element.data[jointIndex].z = pts.slice(1, 8).map(p => p[2]);
      element.data[eeIndex].x = [d.x[idx]];
      element.data[eeIndex].y = [d.raw_y[idx]];
      element.data[eeIndex].z = [d.z[idx]];
      Plotly.redraw(element);
    }
    const observer = typeof ResizeObserver !== 'undefined'
      ? new ResizeObserver(() => { if (!disposed) Plotly.Plots.resize(element); }) : null;
    if (observer) observer.observe(element);

    return {
      update(i) {
        if (disposed) return;
        pending = Math.max(0, Math.min(n - 1, Math.round(Number(i) || 0)));
        if (!raf) raf = requestAnimationFrame(draw);
      },
      view(mode) {
        if (disposed) return Promise.resolve();
        const preset = ROOMS[mode === 'room' ? 'room' : 'detail'];
        return Plotly.relayout(element, {
          'scene.xaxis.range': preset.x,
          'scene.yaxis.range': preset.y,
          'scene.zaxis.range': preset.z,
          'scene.aspectratio': preset.aspect,
          'scene.camera': preset.camera
        });
      },
      destroy() {
        disposed = true;
        if (raf) cancelAnimationFrame(raf);
        if (observer) observer.disconnect();
        Plotly.purge(element);
      }
    };
  }

  window.RoboRoom = { init };
})();
