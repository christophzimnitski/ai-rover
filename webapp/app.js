'use strict';
/*
 * Rover Dashboard — zero-dependency display layer.
 * Talks to rosbridge directly over a WebSocket (no roslibjs) and shows the MJPEG feed.
 * Host is derived from the page URL, so reaching the Pi over LAN or Tailscale both work.
 * This file only READS topics and DISPLAYS them; it publishes nothing and owns no state
 * on the rover. The AI, logging, and DB are separate services (see the modularity rule).
 */

// ---- config --------------------------------------------------------------------
var HOST = location.hostname || '192.168.1.78';
var ROSBRIDGE_URL = 'ws://' + HOST + ':9090';
var VIDEO_URL = 'http://' + HOST + ':8080/stream?topic=/camera/image_raw';
var TRACK_WIDTH = 0.175;   // m — matches driver_params.yaml; for DISPLAYED angular velocity
var MAX_WHEEL = 0.5;       // m/s — ESP32 limit; bar/sparkline scaling

// ---- tiny rosbridge client (direct WebSocket, auto-reconnect) ------------------
function Rosbridge(url, onStatus) {
  this.url = url;
  this.onStatus = onStatus || function () {};
  this.subs = new Map();        // topic -> { type, cb }
  this._t = null;
  this._connect();
}
Rosbridge.prototype._connect = function () {
  this.onStatus('connecting');
  var self = this, ws;
  try { ws = new WebSocket(this.url); } catch (e) { this._retry(); return; }
  this.ws = ws;
  ws.onopen = function () {
    self.onStatus('up');
    self.subs.forEach(function (s, topic) { self._send({ op: 'subscribe', topic: topic, type: s.type }); });
  };
  ws.onclose = function () { self.onStatus('down'); self._retry(); };
  ws.onerror = function () { try { ws.close(); } catch (e) {} };
  ws.onmessage = function (ev) {
    var m; try { m = JSON.parse(ev.data); } catch (e) { return; }
    if (m.op === 'publish') { var s = self.subs.get(m.topic); if (s && s.cb) s.cb(m.msg); }
  };
};
Rosbridge.prototype._retry = function () {
  var self = this;
  if (this._t) return;
  this._t = setTimeout(function () { self._t = null; self._connect(); }, 1500);
};
Rosbridge.prototype._send = function (o) {
  if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(o));
};
Rosbridge.prototype.subscribe = function (topic, type, cb) {
  this.subs.set(topic, { type: type, cb: cb });
  this._send({ op: 'subscribe', topic: topic, type: type });
};

// ---- helpers -------------------------------------------------------------------
function $(id) { return document.getElementById(id); }
function fmt(x, d) { d = (d == null) ? 2 : d; return (x == null || isNaN(x)) ? '–' : Number(x).toFixed(d); }
function pill(el, up, text) { el.className = 'pill ' + (up ? 'up' : 'down'); if (text) el.textContent = text; }

// ---- state ---------------------------------------------------------------------
var lastTelem = 0;
var odom = { x: 0, y: 0, h: 0 };   // heading in radians
var lastOdomT = 0;
var spark = { L: [], R: [], n: 160 };
var trail = [];

// ---- attitude (artificial horizon) --------------------------------------------
function updateAttitude(roll, pitch) {
  $('v-roll').textContent = fmt(roll, 1);
  $('v-pitch').textContent = fmt(pitch, 1);
  var g = $('horizon-g');
  if (g) g.setAttribute('transform', 'rotate(' + (-roll) + ') translate(0,' + (pitch * 2) + ')');
}

// ---- motion --------------------------------------------------------------------
function setBar(el, val, max) {   // bipolar: fills right for +, left for −, from center
  var f = Math.max(-1, Math.min(1, val / max));
  el.style.width = (Math.abs(f) * 50) + '%';
  if (f >= 0) { el.style.left = '50%'; el.style.right = 'auto'; }
  else { el.style.right = '50%'; el.style.left = 'auto'; }
  el.classList.toggle('rev', val < 0);
}
function drawSpark() {
  var c = $('spark'); if (!c) return; var ctx = c.getContext('2d');
  var w = c.width, h = c.height, mid = h / 2, sc = (mid - 6) / MAX_WHEEL;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.12)'; ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();
  function line(arr, color) {
    if (arr.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath();
    var step = w / (spark.n - 1);
    for (var i = 0; i < arr.length; i++) {
      var x = i * step, y = mid - arr[i] * sc;
      if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    }
    ctx.stroke();
  }
  line(spark.L, '#5ac8fa'); line(spark.R, '#ffd60a');
}

// ---- position (wheel-odometry estimate) ----------------------------------------
function drawTrail() {
  var c = $('trail'); if (!c) return; var ctx = c.getContext('2d');
  var w = c.width, h = c.height; ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.10)'; ctx.beginPath();
  ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
  if (!trail.length) return;
  var minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, p;
  for (var i = 0; i < trail.length; i++) {
    p = trail[i];
    if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
  }
  var span = Math.max(0.5, maxX - minX, maxY - minY) * 1.2;
  var cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, sc = Math.min(w, h) / span;
  function px(x) { return w / 2 + (x - cx) * sc; }
  function py(y) { return h / 2 - (y - cy) * sc; }
  ctx.strokeStyle = '#30d158'; ctx.lineWidth = 1.5; ctx.beginPath();
  for (var j = 0; j < trail.length; j++) { if (j) ctx.lineTo(px(trail[j].x), py(trail[j].y)); else ctx.moveTo(px(trail[j].x), py(trail[j].y)); }
  ctx.stroke();
  var last = trail[trail.length - 1];
  ctx.fillStyle = '#30d158'; ctx.beginPath(); ctx.arc(px(last.x), py(last.y), 3, 0, Math.PI * 2); ctx.fill();
}

// ---- handlers ------------------------------------------------------------------
function onTelemetry(msg) {
  lastTelem = performance.now();
  var L = +msg.left_speed, R = +msg.right_speed;
  var v = (L + R) / 2, w = (R - L) / TRACK_WIDTH;
  updateAttitude(+msg.roll, +msg.pitch);
  $('v-left').textContent = fmt(L); $('v-right').textContent = fmt(R);
  $('v-lin').textContent = fmt(v); $('v-ang').textContent = fmt(w);
  setBar($('bar-left'), L, MAX_WHEEL);
  setBar($('bar-right'), R, MAX_WHEEL);
  spark.L.push(L); spark.R.push(R);
  if (spark.L.length > spark.n) { spark.L.shift(); spark.R.shift(); }
  drawSpark();
  // rough wheel-odometry integration (drifts — quantized wheel speed, no IMU/GPS fusion)
  var now = performance.now();
  if (lastOdomT) {
    var dt = (now - lastOdomT) / 1000;
    if (dt > 0 && dt < 0.5) {
      odom.h += w * dt; odom.x += v * Math.cos(odom.h) * dt; odom.y += v * Math.sin(odom.h) * dt;
      trail.push({ x: odom.x, y: odom.y }); if (trail.length > 600) trail.shift();
      $('v-x').textContent = fmt(odom.x); $('v-y').textContent = fmt(odom.y);
      var deg = (odom.h * 180 / Math.PI) % 360; if (deg < 0) deg += 360;
      $('v-hdg').textContent = Math.round(deg);
      drawTrail();
    }
  }
  lastOdomT = now;
}

function onBattery(msg) {
  var v = +msg.voltage;
  $('v-volt').textContent = fmt(v, 2);
  var pct = Math.max(0, Math.min(1, (v - 9.9) / (12.6 - 9.9)));
  var bar = $('bar-batt');
  bar.style.width = (pct * 100) + '%';
  bar.className = v >= 11.1 ? 'ok' : (v >= 10.5 ? 'warn' : 'crit');
}

// ---- telemetry freshness watchdog ----------------------------------------------
setInterval(function () {
  var fresh = performance.now() - lastTelem < 1500;
  pill($('pill-telem'), fresh, fresh ? 'telemetry' : 'telemetry stale');
}, 500);

// ---- camera --------------------------------------------------------------------
function initCamera() {
  var img = $('cam'), pc = $('pill-cam'), overlay = $('overlay');
  function load() { img.src = VIDEO_URL + '&t=' + Date.now(); }
  img.onload = function () { pill(pc, true, 'camera'); resize(); };
  img.onerror = function () { pill(pc, false, 'camera down'); setTimeout(load, 2000); };
  function resize() { overlay.width = img.clientWidth || 0; overlay.height = img.clientHeight || 0; }
  if (window.ResizeObserver) new ResizeObserver(resize).observe(img);
  window.addEventListener('resize', resize);
  load();
}

// ---- boot ----------------------------------------------------------------------
var ros = new Rosbridge(ROSBRIDGE_URL, function (s) {
  pill($('pill-ros'), s === 'up', s === 'up' ? 'rosbridge' : 'rosbridge ' + s);
});
ros.subscribe('/rover/telemetry', 'ugv_msgs/msg/Telemetry', onTelemetry);
ros.subscribe('/rover/battery', 'sensor_msgs/msg/BatteryState', onBattery);
$('reset-odom').onclick = function () {
  odom = { x: 0, y: 0, h: 0 }; trail.length = 0; drawTrail();
  $('v-x').textContent = '0.00'; $('v-y').textContent = '0.00'; $('v-hdg').textContent = '0';
};
initCamera();
