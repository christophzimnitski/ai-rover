'use strict';
/*
 * Rover Dashboard — zero-dependency thin layer.
 * Talks to rosbridge directly over a WebSocket (no roslibjs) and shows the MJPEG feed.
 * Host is derived from the page URL, so reaching the Pi over LAN or Tailscale both work.
 * READS /rover/* topics for display, and FORWARDS user intent (drive / lights / OLED /
 * e-stop) as plain messages. No rover state or safety logic lives here: the driver owns
 * the watchdog, the e-stop latch and gamepad-over-web priority, and the board's own
 * heartbeat backstops the driver if this page or the Pi stack dies mid-drive.
 */

// ---- config --------------------------------------------------------------------
var HOST = location.hostname || '192.168.1.78';
var ROSBRIDGE_URL = 'ws://' + HOST + ':9090';
// ros_compressed = serve the camera's own once-encoded JPEGs (no per-client re-encode).
var VIDEO_URL_FULL = 'http://' + HOST + ':8080/stream?topic=/camera/image_raw&type=ros_compressed';
// scaled fallback: web_video_server can't rescale pre-encoded JPEGs, so smaller sizes
// use its mjpeg re-encode path (more Pi CPU, fewer bytes — for slow links).
var VIDEO_URL_SCALED = 'http://' + HOST + ':8080/stream?topic=/camera/image_raw&quality=60';
var SNAPSHOT_URL = 'http://' + HOST + ':8080/snapshot?topic=/camera/image_raw';
var TRACK_WIDTH = 0.175;   // m — matches driver_params.yaml; for DISPLAYED angular velocity
var MAX_WHEEL = 0.5;       // m/s — {"T":1} saturation; bar/sparkline scaling
var MAX_ANGULAR = 2.4;     // rad/s at full pad deflection (matches teleop feel)
var DRIVE_RATE_MS = 50;    // 20Hz /web_cmd_vel while the pad/keys are active — same cadence
                           // as the gamepad, with headroom if the driver's cmd_timeout or
                           // the board heartbeat window are ever tightened
var IMU_TRUSTED = true;    // ICM20948 read path verified 2026-07-02 (docs/FIRMWARE.md);
                           // yaw is a relative heading (uncalibrated mag), not compass truth

// ---- tiny rosbridge client (direct WebSocket, auto-reconnect) ------------------
function Rosbridge(url, onStatus) {
  this.url = url;
  this.onStatus = onStatus || function () {};
  this.subs = new Map();        // topic -> { type, cb }
  this.ads = new Map();         // topic -> type (advertised publishers)
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
    self.ads.forEach(function (type, topic) { self._send({ op: 'advertise', topic: topic, type: type }); });
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
Rosbridge.prototype.advertise = function (topic, type) {
  this.ads.set(topic, type);
  this._send({ op: 'advertise', topic: topic, type: type });
};
Rosbridge.prototype.publish = function (topic, msg) {
  this._send({ op: 'publish', topic: topic, msg: msg });
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
var estopped = false;

// ---- attitude (artificial horizon) --------------------------------------------
function updateAttitude(roll, pitch, yaw) {
  $('v-roll').textContent = fmt(roll, 1);
  $('v-pitch').textContent = fmt(pitch, 1);
  $('v-yaw').textContent = yaw === 0 ? '–' : fmt(yaw, 1);   // 0.0 = firmware doesn't send it
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
  updateAttitude(+msg.roll, +msg.pitch, +msg.yaw);
  $('v-temp').textContent = msg.temp ? fmt(msg.temp, 0) : '–';
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

function onLink(msg) {
  $('v-heals').textContent = (msg.reconnects + msg.rearms) || '0';
  var ok = msg.connected && msg.telemetry_age >= 0 && msg.telemetry_age < 2;
  if (msg.estop) pill($('pill-driver'), false, 'E-STOPPED');
  else pill($('pill-driver'), ok, ok ? 'driver' : 'driver stale');
  if (msg.estop !== estopped) { estopped = msg.estop; renderEstop(); }
}

// ---- telemetry freshness watchdog ----------------------------------------------
setInterval(function () {
  var fresh = performance.now() - lastTelem < 1500;
  pill($('pill-telem'), fresh, fresh ? 'telemetry' : 'telemetry stale');
}, 500);

// ---- camera --------------------------------------------------------------------
function initCamera() {
  var img = $('cam'), pc = $('pill-cam'), overlay = $('overlay'), res = $('stream-res');
  function url() {
    var u = res.value ? (VIDEO_URL_SCALED + '&width=' + res.value) : VIDEO_URL_FULL;
    return u + '&t=' + Date.now();
  }
  function load() { img.src = url(); }
  img.onload = function () { pill(pc, true, 'camera'); resize(); };
  img.onerror = function () { pill(pc, false, 'camera down'); setTimeout(load, 2000); };
  function resize() { overlay.width = img.clientWidth || 0; overlay.height = img.clientHeight || 0; }
  if (window.ResizeObserver) new ResizeObserver(resize).observe(img);
  window.addEventListener('resize', resize);
  res.onchange = load;
  $('snap').href = SNAPSHOT_URL;
  load();
}

// ---- drive (virtual joystick + WASD -> /web_cmd_vel) ---------------------------
// The driver applies /web_cmd_vel only while /cmd_vel (gamepad) is quiet, ignores it
// under e-stop, and its watchdog stops the rover 0.5s after we stop publishing —
// so a dropped WebSocket, a closed tab or a hidden page all end in a halt.
var drive = { x: 0, y: 0, active: false, timer: null, keys: {} };

function driveVector() {
  var k = drive.keys;
  var kx = (k.d ? 1 : 0) - (k.a ? 1 : 0);
  var ky = (k.w ? 1 : 0) - (k.s ? 1 : 0);
  if (kx || ky) return { x: kx, y: ky };
  return { x: drive.x, y: drive.y };
}
function publishDrive() {
  var v = driveVector();
  var maxLin = parseFloat($('speed').value);
  ros.publish('/web_cmd_vel', {
    linear: { x: v.y * maxLin, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: -v.x * MAX_ANGULAR * (maxLin / 0.5) }
  });
}
function driveLoop(on) {
  if (on && !drive.timer) {
    publishDrive();
    drive.timer = setInterval(publishDrive, DRIVE_RATE_MS);
  } else if (!on && drive.timer) {
    clearInterval(drive.timer); drive.timer = null;
    // a burst of zeros so the stop lands even if one packet is lost
    for (var i = 0; i < 3; i++) ros.publish('/web_cmd_vel',
      { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
  }
}
function initStick() {
  var pad = $('stick'), knob = $('knob');
  function setKnob(fx, fy) {
    var r = pad.clientWidth / 2 - knob.clientWidth / 2;
    knob.style.transform = 'translate(' + (fx * r) + 'px,' + (-fy * r) + 'px)';
  }
  function fromEvent(ev) {
    var rect = pad.getBoundingClientRect();
    var fx = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    var fy = -(((ev.clientY - rect.top) / rect.height) * 2 - 1);
    var m = Math.sqrt(fx * fx + fy * fy);
    if (m > 1) { fx /= m; fy /= m; }
    drive.x = fx; drive.y = fy;
    setKnob(fx, fy);
  }
  pad.addEventListener('pointerdown', function (ev) {
    pad.setPointerCapture(ev.pointerId);
    drive.active = true; fromEvent(ev); driveLoop(true);
  });
  pad.addEventListener('pointermove', function (ev) { if (drive.active) fromEvent(ev); });
  function release() {
    if (!drive.active && !anyKey()) return;
    drive.active = false; drive.x = 0; drive.y = 0; setKnob(0, 0);
    if (!anyKey()) driveLoop(false);
  }
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);

  function anyKey() { var k = drive.keys; return k.w || k.a || k.s || k.d; }
  window.addEventListener('keydown', function (ev) {
    if (ev.repeat || ev.target.tagName === 'INPUT' || ev.target.tagName === 'SELECT') return;
    var key = ev.key.toLowerCase();
    if (key === ' ') { setEstop(true); ev.preventDefault(); return; }
    if ('wasd'.indexOf(key) < 0) return;
    drive.keys[key] = true; driveLoop(true);
  });
  window.addEventListener('keyup', function (ev) {
    var key = ev.key.toLowerCase();
    if ('wasd'.indexOf(key) < 0) return;
    drive.keys[key] = false;
    if (!anyKey() && !drive.active) driveLoop(false);
  });
  document.addEventListener('visibilitychange', function () {
    if (document.hidden) { drive.keys = {}; release(); driveLoop(false); }
  });
  $('speed').oninput = function () { $('v-speed').textContent = fmt(parseFloat(this.value), 2); };
}

// ---- e-stop ---------------------------------------------------------------------
function renderEstop() {
  var b = $('estop');
  b.textContent = estopped ? 'RELEASE E-STOP' : 'E-STOP';
  b.classList.toggle('engaged', estopped);
}
function setEstop(on) {
  ros.publish('/rover/estop', { data: !!on });
  estopped = !!on;                 // optimistic; /rover/link confirms within 1s
  if (on) driveLoop(false);
  renderEstop();
}

// ---- accessories (lights + OLED) -------------------------------------------------
function publishLights() {
  ros.publish('/rover/lights', { io4: parseInt($('io4').value, 10), io5: parseInt($('io5').value, 10) });
  $('v-io4').textContent = $('io4').value; $('v-io5').textContent = $('io5').value;
}
function initAccessories() {
  $('io4').oninput = publishLights;
  $('io5').oninput = publishLights;
  $('lights-off').onclick = function () { $('io4').value = 0; $('io5').value = 0; publishLights(); };
  $('lights-full').onclick = function () { $('io4').value = 255; $('io5').value = 255; publishLights(); };
  $('oled-send').onclick = function () {
    ros.publish('/rover/oled', { line: parseInt($('oled-line').value, 10), text: $('oled-text').value });
  };
  $('oled-reset').onclick = function () { ros.publish('/rover/oled', { line: 255, text: '' }); };
}

// ---- boot ----------------------------------------------------------------------
var ros = new Rosbridge(ROSBRIDGE_URL, function (s) {
  pill($('pill-ros'), s === 'up', s === 'up' ? 'rosbridge' : 'rosbridge ' + s);
});
ros.subscribe('/rover/telemetry', 'ugv_msgs/msg/Telemetry', onTelemetry);
ros.subscribe('/rover/battery', 'sensor_msgs/msg/BatteryState', onBattery);
ros.subscribe('/rover/link', 'ugv_msgs/msg/LinkStatus', onLink);
ros.advertise('/web_cmd_vel', 'geometry_msgs/msg/Twist');
ros.advertise('/rover/estop', 'std_msgs/msg/Bool');
ros.advertise('/rover/lights', 'ugv_msgs/msg/Lights');
ros.advertise('/rover/oled', 'ugv_msgs/msg/OledText');

$('reset-odom').onclick = function () {
  odom = { x: 0, y: 0, h: 0 }; trail.length = 0; drawTrail();
  $('v-x').textContent = '0.00'; $('v-y').textContent = '0.00'; $('v-hdg').textContent = '0';
};
$('estop').onclick = function () { setEstop(!estopped); };
if (IMU_TRUSTED) $('imu-warn').style.display = 'none';
initCamera();
initStick();
initAccessories();
renderEstop();
