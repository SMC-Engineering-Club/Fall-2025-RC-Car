#include <WiFiS3.h>
#include <Servo.h>

#include "Arduino_LED_Matrix.h"
#include "data.h"

#include <UnoR4WiFi_WebServer.h>
#include <WebSocketsServer.h>      // NEW

ArduinoLEDMatrix Matrix;
#define DELAY_FRAME 91


// ======= Wi-Fi =======
const char* ssid = "RC-CAR";
const char* pass = "rcpass123";
UnoR4WiFi_WebServer server(80);
WebSocketsServer ws(81);   


// ======= Outputs =======
Servo esc, steer;
const int THROTTLE_PIN = 9;  // OUTPUT: ESC PWM signal
const int STEERING_PIN = 10; // OUTPUT: Servo PWM signal

// External indicator LEDs
const int LED_THROTTLE_PIN = 6; // OUTPUT: throttle exceed LED
const int LED_STEER_PIN    = 7; // OUTPUT: steering exceed LED

// ======= Control filters & failsafe =======
const float DEAD = 0.06f;          // deadband (6%)
const float ALPHA = 0.35f;         // EMA smoothing (0..1), higher = snappier
const int   FAILSAFE_MS = 300;     // neutral if stale > 300 ms
const float INDICATOR_THRESH = 0.50f; // 50% deflection → LED on

bool  invertThrottle = false;      // flip if forward/back feels reversed
bool  invertSteer    = false;      // flip if left/right feels reversed
float thFilt = 0.0f, stFilt = 0.0f;
unsigned long lastPacketMs = 0;

static inline float applyDeadband(float v, float d) {
  return (fabs(v) < d) ? 0.0f : v;
}



// ======= Keep your existing PAGE string here =======
const char PAGE[] = R"HTML(
<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>RC Car</title>
<style>
  :root{--fg:#111;--muted:#666;--bg:#fff;--accent:#0a84ff}
  body{font:16px system-ui;margin:18px;background:var(--bg);color:var(--fg)}
  h1{margin:0 0 .25rem}
  .row{display:flex;align-items:center;gap:.75rem;flex-wrap:wrap;margin:.5rem 0}
  button{padding:.6rem 1rem;border:1px solid #ddd;border-radius:.6rem;background:#f8f8f8}
  .chip{padding:.25rem .6rem;border:1px solid #e0e0e0;border-radius:999px;background:#fafafa;font-size:.9rem}
  #log{white-space:pre-line;color:var(--muted);margin-top:.25rem}
  .switch{position:relative;width:58px;height:32px}
  .switch input{display:none}
  .slider{position:absolute;cursor:pointer;inset:0;background:#ccc;border-radius:999px;transition:.2s}
  .slider:before{content:"";position:absolute;height:24px;width:24px;left:4px;top:4px;background:white;border-radius:50%;transition:.2s;box-shadow:0 1px 3px rgba(0,0,0,.2)}
  input:checked + .slider{background:var(--accent)}
  input:checked + .slider:before{transform:translateX(26px)}
  .label{font-weight:600;min-width:78px}
  .debug{border:1px solid #eee;border-radius:.6rem;padding:.6rem;margin-top:.5rem;background:#fcfcfc}
  .kv{display:grid;grid-template-columns:auto 1fr;gap:.25rem .75rem}
  .kv div:nth-child(odd){color:#555}
</style>

<h1>RC Car</h1>
<div class="row">
  <span class="label">Mode</span>
  <label class="switch">
    <input type="checkbox" id="modeToggle" aria-label="Mode toggle (off=Gamepad, on=Phone Tilt)">
    <span class="slider"></span>
  </label>
  <span id="modeText" class="chip">Gamepad</span>
  <button id="cal" title="Zero the phone tilt">Calibrate</button>
</div>

<div class="row">
  <button id="start">Start</button>
  <span id="status" class="chip">Idle</span>
</div>

<div class="debug">
  <div class="kv">
    <div>Device:</div><div id="ua"></div>
    <div>Secure context:</div><div id="sec"></div>
    <div>Events seen:</div><div id="evcount">0</div>
    <div>Alpha:</div><div id="a">—</div>
    <div>Beta Corrected:</div><div id="b">—</div>
    <div>Gamma Corrected:</div><div id="g">—</div>
    <div>Beta Original:</div><div id="bO">—</div>
    <div>Gamma Original:</div><div id="gO">—</div>
  </div>
</div>
<div class="debug" id="calcpanel">
  <div class="kv">
    <div>zeroBeta:</div><div id="zB">—</div>
    <div>zeroGamma:</div><div id="zG">—</div>

    <div>bRaw (β-zero):</div><div id="bRaw">—</div>
    <div>gRaw (γ-zero):</div><div id="gRaw">—</div>

    <div>bQ (quantized):</div><div id="bQ">—</div>
    <div>gQ (quantized):</div><div id="gQ">—</div>

    <div>th pre-clamp:</div><div id="thPre">—</div>
    <div>st pre-clamp:</div><div id="stPre">—</div>

    <div>th (clamped):</div><div id="thOut">—</div>
    <div>st (clamped):</div><div id="stOut">—</div>

    <div>th → µs (1000–2000):</div><div id="thUs">—</div>
    <div>st → µs (1500±400):</div><div id="stUs">—</div>

    <div>HTTP ok/fail:</div><div id="httpStats">0 / 0</div>
  </div>
</div>
<pre id="log">Tip: Toggle = Gamepad ↔ Phone Tilt. Calibrate after switching to Tilt.</pre>

<script>
let running=false, mode='gamepad';
let ws = null;
let wsOk = 0, wsFail = 0;
const WS_PORT = 81; // Arduino WS server will listen here
let zeroBeta=0, zeroGamma=0, lastTH=0, lastST=0;
let evCount=0, evTimer, ok=0, fail=0;

const ua=document.getElementById('ua');
const sec=document.getElementById('sec');
const evc=document.getElementById('evcount');
const A=document.getElementById('a'), B=document.getElementById('b'), G=document.getElementById('g'), BO=document.getElementById('bO'), GO=document.getElementById('gO');
ua.textContent = navigator.userAgent;
sec.textContent = (window.isSecureContext ? "yes" : "no");

const modeToggle=document.getElementById('modeToggle');
const modeText=document.getElementById('modeText');
const statusEl=document.getElementById('status');
const logEl=document.getElementById('log');

// NEW: calc panel elements
const zB = document.getElementById('zB');
const zG = document.getElementById('zG');
const bRawEl = document.getElementById('bRaw');
const gRawEl = document.getElementById('gRaw');
const bFloor = document.getElementById('bQ');
const gFloor = document.getElementById('gQ');
const thPreEl = document.getElementById('thPre');
const stPreEl = document.getElementById('stPre');
const thOutEl = document.getElementById('thOut');
const stOutEl = document.getElementById('stOut');
const thUsEl = document.getElementById('thUs');
const stUsEl = document.getElementById('stUs');
const httpStats = document.getElementById('httpStats');

const DEG_STEP = 3; // change to 2/5/etc. for coarser/finer steps
function signedFloorDegrees(d, step){
  const s = Math.sign(d) || 1;
  const a = Math.abs(d);
  return s * Math.floor(a / step) * step;
}
function clamp(x,min,max){return Math.min(max,Math.max(min,x))}
function log(msg){logEl.textContent=msg}
function setStatus(t){statusEl.textContent=t}
function fmt(x, n=3){ return (typeof x === 'number') ? x.toFixed(n) : String(x); }

function wsUrl() {
  // Use the same host the page came from, WS on :81
  return `ws://${location.hostname}:${WS_PORT}/`;
}

function startWS() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  ws = new WebSocket(wsUrl());
  ws.onopen = () => { log("WS connected"); };
  ws.onclose = () => { log("WS closed"); if (running) setTimeout(startWS, 500); };
  ws.onerror = () => { wsFail++; updateNetStats(); };
  ws.onmessage = (ev) => {
    // (optional) handle acks/telemetry from Arduino if you add any
  };
}
//------------------- websocket
function stopWS() {
  try { if (ws) ws.close(); } catch(_) {}
  ws = null;
}

function updateNetStats() {
  // reuse your existing counter element
  httpStats.textContent = `${wsOk} / ${wsFail}`;
}

// 60 Hz sender (tune to 16–20ms)
function tickWS() {
  if (!running) return;
  const { th, st } = getInputs(); // your function, unchanged
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      ws.send(`${th.toFixed(3)},${st.toFixed(3)}`); // no newline
      wsOk++;
    } catch (_) { wsFail++; }
  } else {
    wsFail++;
  }
  if ((wsOk + wsFail) % 10 === 0) updateNetStats();
  setTimeout(tickWS, 45); // ~33 Hz
}
//-------------------------------
// Client-side copies of Arduino’s mappings (for comparison on page)
function escMicrosFromTh(th){ // th in [-1,+1] → 1000..2000
  // same math as: int th_us = map((int)((th+1)*500), 0,1000, 1000,2000)
  const v = (th + 1) * 500;                 // 0..1000
  return Math.round(1000 + (v/1000) * 1000);// 1000..2000
}
function steerMicrosFromSt(st){ // st in [-1,+1] → 1500±400
  return Math.round(1500 + st * 400);
}

function enableGyro(){
  const needPerm = (typeof DeviceMotionEvent!=="undefined" && typeof DeviceMotionEvent.requestPermission==="function");
  const req = needPerm ? DeviceMotionEvent.requestPermission() : Promise.resolve("granted");
  return req.then(res=>{
    if(res!=="granted") throw new Error("Motion access denied");
    window.addEventListener('deviceorientation', onOrient);
    if(evTimer) clearInterval(evTimer);
    evTimer=setInterval(()=>{
      // windowed liveness check only; no counter reset anymore
      // (kept empty on purpose)
    }, 3000);
    mode='gyro'; modeText.textContent="Phone Tilt";
    calibrate();
    log("Tilt mode ON. Hold phone neutrally, tap Calibrate, then Start.");
  });
}
function disableGyro(){
  window.removeEventListener('deviceorientation', onOrient);
  if(evTimer) clearInterval(evTimer);
  mode='gamepad'; modeText.textContent="Gamepad";
  log("Gamepad mode ON. Pair a controller to your phone, press a button, then Start.");
}
modeToggle.addEventListener('change', ()=>{
  if(modeToggle.checked){
    enableGyro().catch(()=>{
      modeToggle.checked=false; disableGyro();
      alert("Motion sensors unavailable or blocked. Staying in Gamepad mode.");
    });
  }else{
    disableGyro();
  }
});

document.getElementById('cal').onclick=calibrate;
function calibrate(){
  const handler=(e)=>{
    zeroBeta=(e.beta??0); zeroGamma=(e.gamma??0);
    zB.textContent = fmt(zeroBeta,1);
    zG.textContent = fmt(zeroGamma,1);
    window.removeEventListener('deviceorientation', handler);
    log("Calibrated.\nzeroBeta="+zeroBeta.toFixed(1)+"  zeroGamma="+zeroGamma.toFixed(1));
  };
  window.addEventListener('deviceorientation', handler, {once:true});
}

function distFromNeg90beta(betan) {
  const target = -90;
  // wrap the difference to (-180, 180]
  const d = ((betan - target + 540) % 360) - 180;
  return Math.abs(d);
};

function distFrom90beta(betap) {
  const target = 90;
  // wrap the difference to (-180, 180]
  const d = ((betap - target + 540) % 360) - 180;
  return Math.abs(d);
};

function onOrient(e){
  evCount++;
  evc.textContent = evCount;
  const beta  = (e.beta  ?? 0);
  const gamma = (e.gamma ?? 0);
  A.textContent = (e.alpha ?? 0).toFixed(1);

  let gammaOut = 0, betaOut = 0; //defaults

  if (-0.1 >= gamma && gamma >= -90.0) {
    gammaOut = distFromNeg90beta(gamma);
  } else if (0.1 <= gamma && gamma <= 90.0) {
    gammaOut = -distFrom90beta(gamma);
  } else {
    gammaOut = 0; //deadzone
  }

  if (-90 <= beta && beta <= -0.1) {
    betaOut = beta
  } else if (90 >= beta && beta >= 0.0) {
    betaOut = beta
  } else if (-90 > beta && beta >= -179.9) {
    betaOut = distFromNeg90beta(beta) - 90
  } else if (90 < beta && beta <= 180.0) {
    betaOut = 90 - distFrom90beta(beta)
  } else {
    betaOut = 0; //just in case
  }

  B.textContent = betaOut.toFixed(1);
  G.textContent = gammaOut.toFixed(1);

  BO.textContent = beta.toFixed(1);
  GO.textContent = gamma.toFixed(1);

  const STEP_DEG = 2;
  const betaFloor = signedFloorDegrees(betaOut, STEP_DEG)
  const gammaFloor = signedFloorDegrees(gammaOut, 1)

  bFloor.textContent   = fmt(betaFloor,1);
  gFloor.textContent   = fmt(gammaFloor,1);

  const FULL_DEG = 45;
  const stPre = betaFloor / FULL_DEG;
  const thPre = gammaFloor / FULL_DEG;

  thPreEl.textContent= fmt(thPre,3);
  stPreEl.textContent= fmt(stPre,3);

  const THROTTLE_LIMIT = 1.00;

  const st = clamp(stPre, -1, 1);
  const th = clamp(thPre, 0, THROTTLE_LIMIT);

  thOutEl.textContent= fmt(th,3);
  stOutEl.textContent= fmt(st,3);

    thUsEl.textContent = escMicrosFromTh(th);
  stUsEl.textContent = steerMicrosFromSt(st);

  lastTH = th;
  lastST = st;
}

window.addEventListener('gamepadconnected',e=>{
  if(mode==='gamepad') log("Gamepad connected: "+e.gamepad.id+"\nTap Start to drive.");
});

document.getElementById('start').onclick = () => {
  running = !running;
  document.getElementById('start').textContent = running ? "Stop" : "Start";
  setStatus(running ? "Streaming" : "Idle");
  if (running) {
    wsOk = 0; wsFail = 0; updateNetStats();
    startWS();
    tickWS();
  } else {
    stopWS();
  }
};


function getInputs(){
  if(mode==='gyro') return {th:lastTH, st:lastST};
  const gp = navigator.getGamepads?.()[0];
  if(gp){
    // Add small client-side deadzone so the display is calmer
    const dz=(x,d=0.06)=> Math.abs(x)<d ? 0 : x;
    let th = dz(-gp.axes[1]);
    let st = dz( gp.axes[0]);
    return {th:clamp(th,-1,1), st:clamp(st,-1,1)};
  }
  return {th:0, st:0};
}

// Default to Gamepad mode
disableGyro();

window.addEventListener('beforeunload', () => { try { if (ws) ws.close(); } catch(_){} });

// Keep screen awake while streaming (Chrome/Android)
let wakeLock;
document.getElementById('start').addEventListener('click', async()=>{
  try{
    if(running && "wakeLock" in navigator){ wakeLock = await navigator.wakeLock.request("screen"); }
    else if(wakeLock){ await wakeLock.release(); wakeLock=null; }
  }catch(_){}
});
</script>
)HTML";


// Simple URL param parser
static float parseParam(const String& url, const char* key, float defv){
  int i = url.indexOf(String(key) + "=");
  if (i < 0) return defv;
  int j = url.indexOf('&', i);
  String v = url.substring(i + strlen(key) + 1, j < 0 ? url.length() : j);
  return v.toFloat();
}

// Soft deadband, matches your JS feel
static inline float applyDeadbandSoft(float v, float db) {
  float m = fabsf(v) - db;
  if (m <= 0) return 0.0f;
  return copysignf(m, v);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.print("WS CONNECT #"); Serial.println(num);
      // ws.sendTXT(num, "hi"); // optional ack
      break;

    case WStype_DISCONNECTED:
      
      break;

    case WStype_TEXT: {
      // Copy to c-string
      char buf[48];
      size_t n = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
      memcpy(buf, payload, n);
      buf[n] = 0;

      // Parse "th,st" (no newline)
      char *comma = strchr(buf, ',');
      if (!comma) { Serial.print("parse fail: '"); Serial.print(buf); Serial.println("'"); return; }
      *comma = 0;
      float th = strtof(buf,   nullptr);
      float st = strtof(comma+1, nullptr);

      // Filters
      th = applyDeadband(th, DEAD);
      st = applyDeadband(st, DEAD);
      thFilt += ALPHA * (th - thFilt);
      stFilt += ALPHA * (st - stFilt);
      lastPacketMs = millis();

      // Outputs
      int th_us = (int)lround((1000.0f + (thFilt + 1.0f) * 500.0f) - 500.0f); // 1000..2000
      int st_us = (int)lround(1500.0f +  stFilt * 400.0f);         // 1500±400
      th_us = constrain(th_us, 1000, 2000);
      st_us = constrain(st_us, 1000, 2000);
      esc.writeMicroseconds(th_us); //th_us
      steer.writeMicroseconds(st_us); //st_us

      // LEDs
      digitalWrite(LED_THROTTLE_PIN, fabs(thFilt) > INDICATOR_THRESH ? HIGH : LOW);
      digitalWrite(LED_STEER_PIN,    fabs(stFilt) > INDICATOR_THRESH ? HIGH : LOW);

      // Debug ~10 Hz
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print("TEXT th="); Serial.print(th,3);
        Serial.print(" st=");     Serial.print(st,3);
        Serial.print(" filt ");   Serial.print(thFilt,3); Serial.print(',');
        Serial.print(stFilt,3);
        Serial.print(" us ");     Serial.print(th_us); Serial.print(',');
        Serial.println(st_us);
      }
    } break;

    case WStype_BIN:   Serial.println("WS BIN");   break;
    case WStype_PING:  Serial.println("WS PING");  break;
    case WStype_PONG:  Serial.println("WS PONG");  break;
    case WStype_ERROR: Serial.println("WS ERROR"); break;
    default:
      Serial.print("WS type="); Serial.println((int)type);
      break;
  }
}


void setup() {
  Serial.begin(115200);
  Matrix.begin();

  // PWM devices (safe even if nothing connected)
  esc.attach(THROTTLE_PIN);
  steer.attach(STEERING_PIN);
  esc.writeMicroseconds(1000);  // OUTPUT: ESC PWM (arm low)
  // steer is centered via stFilt below
  delay(3000);

  // Wi-Fi AP
  WiFi.beginAP(ssid, pass);
  while (WiFi.status() != WL_AP_LISTENING) { delay(100); }
  server.begin();   // ← keep this since beginAP() isn't available
  Serial.print("AP up. SSID: "); Serial.println(ssid);
  Serial.print("Open: http://"); Serial.println(WiFi.localIP());


  ws.onEvent(onWsEvent);          // handler FIRST
  ws.begin();                 // start WebSocket server on :81

// make the link tolerant & snappy
ws.enableHeartbeat(15000, 3000, 2); // ping every 15s, 3s timeout, allow 2 misses

server.addRoute("/",
  [](WiFiClient& client,
     const String&, const String&, const QueryParams&, const String&)
  {
    extern const char PAGE[];
    const size_t len = strlen(PAGE);

    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: text/html; charset=utf-8\r\n");
    client.print("Cache-Control: no-store\r\n");
    client.print("Connection: close\r\n");
    client.print("Content-Length: "); client.print(len); client.print("\r\n\r\n");

    client.write((const uint8_t*)PAGE, len);  // <-- works on UNO R4
  }
);

// (optional) stop favicon 404 spam
server.addRoute("/favicon.ico",
  [](WiFiClient& client,
     const String& method,
     const String& path,
     const QueryParams& params,
     const String& body)
  {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println();
  }
);

  Serial.print("WS:   ws://");
Serial.print(WiFi.localIP());
Serial.println(":81/");

}

void loop() {

  static unsigned long lastFrame;
  static int frameIdx;
  if (millis() - lastFrame >= DELAY_FRAME) {
    Matrix.loadFrame(&bmp[frameIdx * 3]);
    frameIdx = (frameIdx + 1) % (sizeof(bmp)/sizeof(bmp[0])/3);
    lastFrame = millis();
  }

  server.handleClient();     // HTTP routing
  ws.loop(); 

  // --- NEW: failsafe if control stream goes stale ---
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    thFilt = 0.0f;
    stFilt = 0.0f;
    esc.writeMicroseconds(1000);   // or 1000 if your ESC is one-way
    steer.writeMicroseconds(1500);
    digitalWrite(LED_THROTTLE_PIN, LOW);
    digitalWrite(LED_STEER_PIN, LOW);
  }
}
