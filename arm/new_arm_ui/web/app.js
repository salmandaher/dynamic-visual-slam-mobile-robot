// new_arm web UI frontend. Talks to server.js over a WebSocket; draws a live 2D
// schematic from /joint_states using the shared FK in fk.js.
(function () {
  'use strict';

  var ACTUATED = ['after_base_full_joint', 'link1_joint', 'link2_joint'];
  var PASSIVE = 'end_effector_joint';
  var JOINT_MIN = -3.14, JOINT_MAX = 3.14;

  var ws = null;
  var lastState = {}; // name -> position
  var el = function (id) { return document.getElementById(id); };

  // ---------------- WebSocket ----------------
  function connect() {
    ws = new WebSocket('ws://' + location.host);
    ws.onopen = function () { setConn(true); };
    ws.onclose = function () { setConn(false); setTimeout(connect, 1500); };
    ws.onmessage = function (e) {
      var msg;
      try { msg = JSON.parse(e.data); } catch (_) { return; }
      if (msg.type === 'joint_states') onState(msg);
      else if (msg.type === 'status') log(msg.text);
      else if (msg.type === 'config') log('config: actuated=' + msg.actuated.join(','));
    };
  }
  function send(obj) {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
    else log('not connected.');
  }
  function setConn(on) {
    var b = el('conn');
    b.textContent = on ? 'connected' : 'disconnected';
    b.className = 'badge ' + (on ? 'on' : 'off');
  }

  // ---------------- logging ----------------
  function log(text) {
    var box = el('log');
    var d = document.createElement('div');
    d.className = 'line';
    var t = new Date().toLocaleTimeString();
    d.textContent = '[' + t + '] ' + text;
    box.appendChild(d);
    box.scrollTop = box.scrollHeight;
  }

  // ---------------- state in ----------------
  function onState(msg) {
    for (var i = 0; i < msg.name.length; i++) lastState[msg.name[i]] = msg.position[i];
    // update jog sliders only if the user is not dragging them
    ACTUATED.forEach(function (j) {
      var s = el('slider-' + j);
      if (s && !s.dataset.dragging && lastState[j] != null) {
        s.value = lastState[j];
        el('val-' + j).textContent = (+lastState[j]).toFixed(3);
      }
    });
    renderReadout();
    draw();
  }

  function renderReadout() {
    var box = el('state-readout');
    box.innerHTML = '';
    ACTUATED.concat([PASSIVE]).forEach(function (j) {
      var v = lastState[j];
      var row = document.createElement('div');
      row.textContent = j + (v == null ? '  --' : '');
      if (v != null) {
        var span = document.createElement('span');
        span.textContent = (+v).toFixed(3) + ' rad  (' + (v * 57.2958).toFixed(1) + '°)';
        row.appendChild(span);
      }
      box.appendChild(row);
    });
  }

  // ---------------- jog sliders ----------------
  function buildJog() {
    var wrap = el('jog-sliders');
    ACTUATED.forEach(function (j) {
      var row = document.createElement('div');
      row.className = 'jog-row';
      var name = document.createElement('span');
      name.className = 'name'; name.textContent = j;
      var slider = document.createElement('input');
      slider.type = 'range'; slider.min = JOINT_MIN; slider.max = JOINT_MAX;
      slider.step = 0.01; slider.value = 0; slider.id = 'slider-' + j;
      var val = document.createElement('span');
      val.className = 'val'; val.id = 'val-' + j; val.textContent = '0.000';

      slider.addEventListener('pointerdown', function () { slider.dataset.dragging = '1'; });
      slider.addEventListener('pointerup', function () { delete slider.dataset.dragging; });
      slider.addEventListener('input', function () {
        val.textContent = (+slider.value).toFixed(3);
        if (el('jlive').checked) sendJog();
      });
      row.appendChild(name); row.appendChild(slider); row.appendChild(val);
      wrap.appendChild(row);
    });
  }
  function jogValues() {
    return ACTUATED.map(function (j) { return +el('slider-' + j).value; });
  }
  function sendJog() {
    send({ type: 'jog', joints: jogValues(), time: +el('jtime').value || 1.0 });
  }

  // ---------------- named buttons ----------------
  function buildNamed() {
    var wrap = el('named-buttons');
    ['home', 'ready'].forEach(function (n) {
      var b = document.createElement('button');
      b.textContent = n;
      b.addEventListener('click', function () {
        send({ type: 'named', name: n, time: +el('gtime').value || 2.0 });
      });
      wrap.appendChild(b);
    });
  }

  // ---------------- 2D schematic ----------------
  // meters -> pixels; origin near bottom-centre of each canvas.
  var SCALE = 480; // px per metre

  function poseAngles() {
    return [
      lastState[ACTUATED[0]] || 0,
      lastState[ACTUATED[1]] || 0,
      lastState[ACTUATED[2]] || 0,
      lastState[PASSIVE] || 0,
    ];
  }

  function draw() {
    var pts = window.ArmFK.fk(poseAngles());
    drawTop(pts);
    drawSide(pts);
  }

  function drawTop(pts) {
    var c = el('top'), g = c.getContext('2d');
    g.clearRect(0, 0, c.width, c.height);
    var cx = c.width / 2, cy = c.height / 2;
    grid(g, c, cx, cy);
    // x -> right, y -> up
    g.strokeStyle = '#4d8dff'; g.lineWidth = 3; g.beginPath();
    pts.forEach(function (p, i) {
      var px = cx + p[0] * SCALE, py = cy - p[1] * SCALE;
      if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
    });
    g.stroke();
    joints(g, pts, function (p) { return [cx + p[0] * SCALE, cy - p[1] * SCALE]; });
  }

  function drawSide(pts) {
    var c = el('side'), g = c.getContext('2d');
    g.clearRect(0, 0, c.width, c.height);
    var ox = 30, oy = c.height - 30; // origin bottom-left-ish
    grid(g, c, ox, oy);
    // horizontal = radius sqrt(x^2+y^2), vertical = z (up)
    g.strokeStyle = '#4d8dff'; g.lineWidth = 3; g.beginPath();
    pts.forEach(function (p, i) {
      var r = Math.sqrt(p[0] * p[0] + p[1] * p[1]);
      var px = ox + r * SCALE, py = oy - p[2] * SCALE;
      if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
    });
    g.stroke();
    joints(g, pts, function (p) {
      var r = Math.sqrt(p[0] * p[0] + p[1] * p[1]);
      return [ox + r * SCALE, oy - p[2] * SCALE];
    });
  }

  function grid(g, c, ox, oy) {
    g.strokeStyle = '#222a33'; g.lineWidth = 1;
    for (var x = 0; x <= c.width; x += 40) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, c.height); g.stroke(); }
    for (var y = 0; y <= c.height; y += 40) { g.beginPath(); g.moveTo(0, y); g.lineTo(c.width, y); g.stroke(); }
    g.fillStyle = '#3ecf8e'; g.beginPath(); g.arc(ox, oy, 4, 0, 7); g.fill();
  }
  function joints(g, pts, proj) {
    g.fillStyle = '#e6e9ee';
    pts.forEach(function (p, i) {
      var xy = proj(p); g.beginPath(); g.arc(xy[0], xy[1], i === pts.length - 1 ? 5 : 3, 0, 7); g.fill();
    });
    g.fillStyle = '#ff9d3d';
    var tip = proj(pts[pts.length - 1]);
    g.beginPath(); g.arc(tip[0], tip[1], 5, 0, 7); g.fill();
  }

  // click-to-set targets
  function wireCanvasClicks() {
    var top = el('top');
    top.addEventListener('click', function (ev) {
      var rect = top.getBoundingClientRect();
      var px = ev.clientX - rect.left, py = ev.clientY - rect.top;
      var x = (px - top.width / 2) / SCALE;
      var y = (top.height / 2 - py) / SCALE;
      el('gx').value = x.toFixed(3); el('gy').value = y.toFixed(3);
      log('target X,Y set to ' + x.toFixed(3) + ', ' + y.toFixed(3));
    });
    var side = el('side');
    side.addEventListener('click', function (ev) {
      var rect = side.getBoundingClientRect();
      var py = ev.clientY - rect.top;
      var z = ((side.height - 30) - py) / SCALE;
      el('gz').value = z.toFixed(3);
      log('target Z set to ' + z.toFixed(3));
    });
  }

  // ---------------- wire up ----------------
  function init() {
    buildJog();
    buildNamed();
    wireCanvasClicks();
    el('goto').addEventListener('click', function () {
      send({
        type: 'goto',
        x: +el('gx').value, y: +el('gy').value, z: +el('gz').value,
        time: +el('gtime').value || 2.0,
      });
    });
    el('jog').addEventListener('click', sendJog);
    el('stop').addEventListener('click', function () { send({ type: 'stop' }); });
    renderReadout();
    draw();
    connect();
  }
  window.addEventListener('DOMContentLoaded', init);
})();
