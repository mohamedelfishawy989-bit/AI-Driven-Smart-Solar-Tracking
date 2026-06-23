#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ================= SERIAL DEBUG =================
const unsigned long SERIAL_BAUD = 115200;
const unsigned long SERIAL_WAIT_TIMEOUT_MS = 1500;

// ================= WIFI =================
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
bool wifiAttemptActive = false;
unsigned long wifiAttemptStartedAt = 0;
unsigned long lastWifiRetryAt = 0;

WebServer server(80);

// ================= HARDWARE =================
const int relayPin = 23;

// MG996R 180-degree positional servos.
Servo servoH;
Servo servoV;
const int servoHPin = 25;
const int servoVPin = 26;

// LDRs for solar tracking.
const int LDR_TL = 32;
const int LDR_TR = 33;
const int LDR_BL = 35;
const int LDR_BR = 34;

// I2C sensors.
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_INA219 ina219;
bool lightSensorReady = false;
bool ina219Ready = false;

// DS18B20.
const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// ================= RTOS =================
SemaphoreHandle_t i2cMutex = NULL;
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t servoMutex = NULL;

// ================= SYSTEM DATA =================
int ldrTLValue = 0;
int ldrTRValue = 0;
int ldrBLValue = 0;
int ldrBRValue = 0;

bool pumpState = false;
bool autoMode = true;
bool trackingEnabled = true;

float sysVolts = 0.0f;
float sysAmps = 0.0f;
float sysWatts = 0.0f;
float panelEfficiency = 0.0f;
float panelTempC = 0.0f;
float luxValue = 0.0f;

const float MAX_PANEL_POWER = 18.0f;
const float SUNLIGHT_THRESHOLD = 5000.0f;
const float TEMP_THRESHOLD = 40.0f;

// ================= MG996R 180-DEGREE TRACKING =================
const int SERVO_MIN_PULSE_US = 500;
const int SERVO_MAX_PULSE_US = 2500;

const int SERVO_H_MIN_ANGLE = 45;
const int SERVO_H_MAX_ANGLE = 135;
const int SERVO_V_MIN_ANGLE = 45;
const int SERVO_V_MAX_ANGLE = 135;

const int SERVO_H_HOME_ANGLE = 90;
const int SERVO_V_HOME_ANGLE = 90;

const int H_DIRECTION = 1;
const int V_DIRECTION = 1;

const float LDR_DEADZONE = 170.0f;
const float LDR_ERROR_FOR_MAX_STEP = 1450.0f;
const float ERROR_FILTER_ALPHA = 0.35f;

const int SERVO_MIN_STEP_DEG = 1;
const int SERVO_MAX_STEP_DEG = 1;
const unsigned long TRACKING_INTERVAL_MS = 150; 
const unsigned long TRACKING_LOG_INTERVAL_MS = 1000;

float filteredErrorH = 0.0f;
float filteredErrorV = 0.0f;
float lastErrorH = 0.0f;
float lastErrorV = 0.0f;
int servoHAngle = SERVO_H_HOME_ANGLE;
int servoVAngle = SERVO_V_HOME_ANGLE;
int servoHStepDeg = 0;
int servoVStepDeg = 0;

// ================= NIGHT MODE =================
bool isNight = false;
float nightLuxThreshold = 150.0f;
const int NIGHT_CONFIRM_COUNT = 5;
const int DAY_CONFIRM_COUNT = 3;

// ================= CLOUD & AI =================
float lastLux = 0.0f;
bool isCloudy = false;
float expectedPowerAI = 0.0f;
float performanceRatio = 1.0f;
float lastKnownLux = 0.0f;

// ================= TIMING =================
unsigned long pumpStartTime = 0;
unsigned long lastCleaning = 0;
const unsigned long autoOffTime = 10000;
const unsigned long cleaningCooldown = 1800000;

// ================= DASHBOARD =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta charset="utf-8">
<style>
:root{--bg:#111317;--panel:#1a1d22;--panel2:#20252c;--line:#313842;--text:#f4f7fb;--muted:#9aa7b6;--good:#42d392;--bad:#ff5c72;--warn:#f5c542;--info:#4cc9f0;--soft:#15181d}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--text);font-family:Arial,Helvetica,sans-serif;margin:0;padding:18px}
.shell{max-width:1180px;margin:0 auto}
.topbar{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;margin-bottom:16px}
h1{font-size:28px;line-height:1.1;margin:0}
.sub{color:var(--muted);font-size:13px;margin-top:5px}
.statusline{display:flex;flex-wrap:wrap;gap:8px;justify-content:flex-end}
.badge{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);background:var(--panel);border-radius:999px;padding:7px 10px;font-size:12px;color:var(--muted)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}
.good .dot{background:var(--good)}.bad .dot{background:var(--bad)}.warn .dot{background:var(--warn)}
.layout{display:grid;grid-template-columns:1.45fr .85fr;gap:12px}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;min-width:0}
.wide{grid-column:span 2}.full{grid-column:1/-1}
.kicker{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:.08em;margin-bottom:8px}
.value{font-size:28px;font-weight:800;line-height:1.1;white-space:nowrap}.unit{font-size:13px;color:var(--muted);font-weight:700;margin-left:4px}
.small{font-size:12px;color:var(--muted);margin-top:6px}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:7px 0;border-bottom:1px solid rgba(255,255,255,.06)}
.row:last-child{border-bottom:0}
.label{color:var(--muted);font-size:13px}.num{font-weight:800}
.pill{display:inline-flex;align-items:center;justify-content:center;min-width:58px;border-radius:999px;padding:5px 9px;font-size:12px;font-weight:800;border:1px solid var(--line);background:var(--soft);color:var(--muted)}
.pill.on{color:#07130d;background:var(--good);border-color:var(--good)}.pill.off{color:#20070b;background:var(--bad);border-color:var(--bad)}.pill.warn{color:#1f1701;background:var(--warn);border-color:var(--warn)}
.actions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:12px}
button{height:42px;border:0;border-radius:8px;cursor:pointer;font-weight:800;color:#081016;background:var(--info)}
button.secondary{background:#2a3038;color:var(--text);border:1px solid var(--line)}button.danger{background:var(--bad);color:#26080d}
button:active{transform:translateY(1px)}
.bar{height:9px;background:#111419;border:1px solid var(--line);border-radius:999px;overflow:hidden;margin-top:9px}
.fill{height:100%;width:0;background:var(--good);transition:width .25s ease}
.fill.warn{background:var(--warn)}.fill.bad{background:var(--bad)}.fill.info{background:var(--info)}
.servo{height:18px;background:#111419;border:1px solid var(--line);border-radius:999px;position:relative;margin:12px 0 6px;overflow:hidden}
.servo:before{content:"";position:absolute;left:50%;top:0;bottom:0;width:1px;background:#59616d}.servo span{position:absolute;top:2px;bottom:2px;width:14px;background:var(--info);border-radius:999px;transition:.2s}
.quad{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}.cell{min-height:74px;border:1px solid var(--line);background:rgba(76,201,240,.12);border-radius:8px;padding:10px;display:flex;flex-direction:column;justify-content:space-between}.cell strong{font-size:18px}.cell span{font-size:12px;color:var(--muted)}
canvas{width:100%;height:210px;display:block;background:#111419;border:1px solid var(--line);border-radius:8px}
.legend{display:flex;gap:12px;flex-wrap:wrap;color:var(--muted);font-size:12px;margin-top:9px}.key{display:flex;align-items:center;gap:6px}.sw{width:18px;height:3px;border-radius:999px;background:var(--info)}.sw.power{background:var(--good)}.sw.temp{background:var(--warn)}
.insight{font-size:15px;line-height:1.4}.muted{color:var(--muted)}
@media(max-width:920px){.layout{grid-template-columns:1fr}.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.topbar{align-items:flex-start;flex-direction:column}.statusline{justify-content:flex-start}}
@media(max-width:560px){body{padding:12px}.grid{grid-template-columns:1fr}.wide{grid-column:auto}.actions{grid-template-columns:1fr}h1{font-size:23px}.value{font-size:24px}}
</style>
</head><body>
<div class="shell">
<div class="topbar">
<div><h1>Solar Tracker</h1><div class="sub">ESP32 live control panel - MG996R 180 deg</div></div>
<div class="statusline">
<span class="badge" id="conn"><span class="dot"></span><span id="connText">Connecting</span></span>
<span class="badge"><span class="dot"></span><span id="uptime">0s</span></span>
<span class="badge"><span class="dot"></span><span id="wifi">RSSI --</span></span>
<span class="badge"><span class="dot"></span><span id="heap">Heap -- KB</span></span>
</div>
</div>
<div class="layout">
<main class="grid">
<section class="card"><div class="kicker">Power</div><div class="value"><span id="p">0.000</span><span class="unit">W</span></div><div class="small">Expected <span id="expected">0.00</span> W</div><div class="bar"><div class="fill power" id="powerFill"></div></div></section>
<section class="card"><div class="kicker">Light</div><div class="value"><span id="lux">0.0</span><span class="unit">lux</span></div><div class="small" id="lightState">Waiting</div><div class="bar"><div class="fill info" id="luxFill"></div></div></section>
<section class="card"><div class="kicker">Temperature</div><div class="value"><span id="t">0.00</span><span class="unit">C</span></div><div class="small">Limit 40 C</div><div class="bar"><div class="fill warn" id="tempFill"></div></div></section>
<section class="card"><div class="kicker">Health</div><div class="value"><span id="ai">0.0</span><span class="unit">%</span></div><div class="small" id="healthText">Analyzing</div><div class="bar"><div class="fill" id="healthFill"></div></div></section>
<section class="card wide">
<div class="kicker">Live Trend</div>
<canvas id="chart" width="680" height="230"></canvas>
<div class="legend"><span class="key"><span class="sw power"></span>Power</span><span class="key"><span class="sw"></span>Lux</span><span class="key"><span class="sw temp"></span>Temp</span></div>
</section>
<section class="card wide">
<div class="kicker">LDR Balance</div>
<div class="quad">
<div class="cell" id="qtl"><span>Top Left</span><strong id="tl">0</strong></div>
<div class="cell" id="qtr"><span>Top Right</span><strong id="tr">0</strong></div>
<div class="cell" id="qbl"><span>Bottom Left</span><strong id="bl">0</strong></div>
<div class="cell" id="qbr"><span>Bottom Right</span><strong id="br">0</strong></div>
</div>
</section>
<section class="card wide">
<div class="kicker">Servo 180</div>
<div class="row"><span class="label">Horizontal angle</span><span class="num"><span id="sha">90</span> deg / step <span id="sh">0</span></span></div>
<div class="servo"><span id="barH"></span></div>
<div class="row"><span class="label">Vertical angle</span><span class="num"><span id="sva">90</span> deg / step <span id="sv">0</span></span></div>
<div class="servo"><span id="barV"></span></div>
<div class="row"><span class="label">H error</span><span class="num" id="eh">0</span></div>
<div class="row"><span class="label">V error</span><span class="num" id="ev">0</span></div>
</section>
<section class="card wide">
<div class="kicker">Electrical</div>
<div class="row"><span class="label">Voltage</span><span class="num"><span id="v">0.00</span> V</span></div>
<div class="row"><span class="label">Current</span><span class="num"><span id="i">0.000</span> A</span></div>
<div class="row"><span class="label">Efficiency</span><span class="num"><span id="eff">0.0</span>%</span></div>
</section>
</main>
<aside>
<section class="card">
<div class="kicker">System State</div>
<div class="row"><span class="label">Pump</span><span class="pill" id="pump">OFF</span></div>
<div class="row"><span class="label">Auto clean</span><span class="pill" id="auto">OFF</span></div>
<div class="row"><span class="label">Tracking</span><span class="pill" id="tracking">OFF</span></div>
<div class="row"><span class="label">Night mode</span><span class="pill" id="night">NO</span></div>
<div class="row"><span class="label">Cloud</span><span class="pill" id="cloud">CLEAR</span></div>
<div class="row"><span class="label">Pump timer</span><span class="num" id="pumpTimer">0s</span></div>
<div class="actions">
<button onclick="cmd('/toggle')">Pump</button>
<button class="secondary" onclick="cmd('/toggleAuto')">Auto</button>
<button class="danger" onclick="cmd('/toggleTracking')">Tracking</button>
</div>
</section>
<section class="card" style="margin-top:12px">
<div class="kicker">Insight</div>
<div class="insight" id="insight">Waiting for sensor data.</div>
</section>
</aside>
</div>
</div>
<script>
const $=id=>document.getElementById(id),hist={p:[],lux:[],t:[]},MAX=90;
function txt(id,v){$(id).textContent=v}
function pct(v,max){return Math.max(0,Math.min(100,(v/max)*100))}
function fmtTime(s){s=Math.max(0,Math.floor(s));let h=Math.floor(s/3600),m=Math.floor((s%3600)/60),r=s%60;return h?h+"h "+m+"m":m?m+"m "+r+"s":r+"s"}
function setPill(id,on,labelOn,labelOff,warn){let e=$(id);e.textContent=on?labelOn:labelOff;e.className="pill "+(warn?"warn":on?"on":"off")}
function setFill(id,v,max,mode){let e=$(id);e.style.width=pct(v,max)+"%";e.className="fill "+(mode||"")}
function servoAngleBar(id,angle){let e=$(id),p=Math.max(0,Math.min(100,(angle/180)*100));e.style.left="calc("+p+"% - 7px)"}
function ldrCell(id,val,max){let o=.10+.32*(max?val/max:0);$(id).style.background="rgba(76,201,240,"+o.toFixed(2)+")"}
function push(a,v){a.push(v);if(a.length>MAX)a.shift()}
function draw(){
let c=$("chart"),x=c.getContext("2d"),w=c.width,h=c.height,pad=28;
x.clearRect(0,0,w,h);x.fillStyle="#111419";x.fillRect(0,0,w,h);x.strokeStyle="#27303a";x.lineWidth=1;
for(let i=1;i<4;i++){let y=pad+i*(h-pad*2)/4;x.beginPath();x.moveTo(pad,y);x.lineTo(w-pad,y);x.stroke()}
line(hist.p,"#42d392",18);line(hist.lux,"#4cc9f0",12000);line(hist.t,"#f5c542",70);
function line(arr,col,max){if(arr.length<2)return;x.strokeStyle=col;x.lineWidth=2;x.beginPath();arr.forEach((v,i)=>{let xx=pad+i*(w-pad*2)/(MAX-1),yy=h-pad-Math.max(0,Math.min(1,v/max))*(h-pad*2);i?x.lineTo(xx,yy):x.moveTo(xx,yy)});x.stroke()}
}
async function cmd(url){try{await fetch(url);await update()}catch(e){}}
function insight(d){
if(!d.tracking)return "Tracking is stopped. The servos hold their last angle.";
if(d.night)return "Night mode is active. The servos are parking to the home angle.";
if(d.cloud)return "Cloud drop detected. Cleaning logic is waiting for stable sunlight.";
if(Math.abs(d.eh)<180&&Math.abs(d.ev)<180)return "Panel is balanced on the light source.";
if(d.ai<.6&&d.lux>5000)return "Power is low for current light. Panel may need cleaning.";
return "System is tracking with 180-degree position control.";
}
async function update(){
try{
let d=await fetch('/data',{cache:'no-store'}).then(r=>r.json());
$("conn").className="badge good";txt("connText","Online");txt("uptime",fmtTime(d.uptime));txt("wifi","RSSI "+d.rssi+" dBm");
txt("p",d.p.toFixed(3));txt("expected",d.expected.toFixed(2));txt("lux",d.lux.toFixed(1));txt("t",d.t.toFixed(2));txt("ai",(d.ai*100).toFixed(1));
txt("v",d.v.toFixed(2));txt("i",d.i.toFixed(3));txt("eff",d.eff.toFixed(1));txt("heap","Heap "+(d.heap/1024).toFixed(1)+" KB");txt("eh",d.eh.toFixed(0));txt("ev",d.ev.toFixed(0));txt("sha",d.sha);txt("sva",d.sva);txt("sh",d.sh);txt("sv",d.sv);
txt("tl",d.tl);txt("tr",d.tr);txt("bl",d.bl);txt("br",d.br);txt("pumpTimer",fmtTime(d.pumpRemaining));
setPill("pump",d.pump,"ON","OFF");setPill("auto",d.auto,"ON","OFF");setPill("tracking",d.tracking,"ON","OFF");setPill("night",d.night,"YES","NO",d.night);setPill("cloud",d.cloud,"YES","CLEAR",d.cloud);
setFill("powerFill",d.p,18,"power");setFill("luxFill",d.lux,12000,"info");setFill("tempFill",d.t,70,d.t>40?"bad":"warn");setFill("healthFill",d.ai,1,d.ai<.6?"bad":d.ai<.8?"warn":"");
txt("lightState",d.night?"Night":d.cloud?"Cloudy":"Clear");txt("healthText",d.ai<.6?"Low":d.ai<.8?"Watch":"Good");txt("insight",insight(d));
servoAngleBar("barH",d.sha);servoAngleBar("barV",d.sva);let m=Math.max(1,d.tl,d.tr,d.bl,d.br);ldrCell("qtl",d.tl,m);ldrCell("qtr",d.tr,m);ldrCell("qbl",d.bl,m);ldrCell("qbr",d.br,m);
push(hist.p,d.p);push(hist.lux,d.lux);push(hist.t,d.t);draw();
}catch(e){$("conn").className="badge bad";txt("connText","Offline")}
}
setInterval(update,1000);update();
</script>
</body></html>
)rawliteral";

// ================= HELPERS =================
int clampAngle(int value, int minAngle, int maxAngle) {
  return constrain(value, minAngle, maxAngle);
}

int errorToStep(float error, int direction) {
  float absError = fabsf(error);
  if (absError <= LDR_DEADZONE) {
    return 0;
  }

  float normalized = (absError - LDR_DEADZONE) / LDR_ERROR_FOR_MAX_STEP;
  normalized = constrain(normalized, 0.0f, 1.0f);

  int stepDeg = SERVO_MIN_STEP_DEG +
                (int)(((SERVO_MAX_STEP_DEG - SERVO_MIN_STEP_DEG) * normalized) + 0.5f);

  int sign = (error > 0.0f) ? 1 : -1;
  return direction * sign * stepDeg;
}

void attachTrackingServos() {
  if (!servoH.attached()) {
    servoH.setPeriodHertz(50);
    servoH.write(servoHAngle);
    servoH.attach(servoHPin, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  }
  if (!servoV.attached()) {
    servoV.setPeriodHertz(50);
    servoV.write(servoVAngle);
    servoV.attach(servoVPin, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  }
}

void setServoAngles(int hAngle, int vAngle, int hStep, int vStep) {
  hAngle = clampAngle(hAngle, SERVO_H_MIN_ANGLE, SERVO_H_MAX_ANGLE);
  vAngle = clampAngle(vAngle, SERVO_V_MIN_ANGLE, SERVO_V_MAX_ANGLE);

  bool servoWriteOk = false;
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(50))) {
    attachTrackingServos();
    servoH.write(hAngle);
    servoV.write(vAngle);
    servoWriteOk = true;
    xSemaphoreGive(servoMutex);
  }

  if (!servoWriteOk) {
    return;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  servoHAngle = hAngle;
  servoVAngle = vAngle;
  servoHStepDeg = hStep;
  servoVStepDeg = vStep;
  xSemaphoreGive(dataMutex);
}

void parkTrackingServosToHome() {
  setServoAngles(SERVO_H_HOME_ANGLE, SERVO_V_HOME_ANGLE, 0, 0);
}

void writePumpRelay(bool state) {
  digitalWrite(relayPin, state ? HIGH : LOW);
}

void setPumpLocked(bool state) {
  pumpState = state;
  writePumpRelay(state);
  if (state) {
    pumpStartTime = millis();
  }
}

void setPump(bool state) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  setPumpLocked(state);
  xSemaphoreGive(dataMutex);
}

int readAverage(int pin) {
  long sum = 0;
  const int samples = 6;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return sum / samples;
}

void updateCloudAndPerformance(float lux, float watts) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  float diff = lux - lastLux;
  if (diff < -1000.0f) {
    isCloudy = true;
  } else if (diff > 500.0f) {
    isCloudy = false;
  }
  lastLux = lux;

  expectedPowerAI = (lux / SUNLIGHT_THRESHOLD) * MAX_PANEL_POWER;
  expectedPowerAI = constrain(expectedPowerAI, 0.0f, MAX_PANEL_POWER);

  if (expectedPowerAI > 0.1f) {
    performanceRatio = watts / expectedPowerAI;
  } else {
    performanceRatio = 1.0f;
  }
  performanceRatio = constrain(performanceRatio, 0.0f, 1.0f);

  xSemaphoreGive(dataMutex);
}

void handleNightMode(float lux) {
  static int nightCount = 0;
  static int dayCount = 0;

  if (lux < nightLuxThreshold) {
    nightCount++;
    dayCount = 0;
  } else {
    dayCount++;
    nightCount = 0;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (!isNight && nightCount >= NIGHT_CONFIRM_COUNT) {
    isNight = true;
  }
  if (isNight && dayCount >= DAY_CONFIRM_COUNT) {
    isNight = false;
    servoHStepDeg = 0;
    servoVStepDeg = 0;
  }
  xSemaphoreGive(dataMutex);

}

void solarTracking180() {
  bool nightSnapshot = false;
  bool trackingSnapshot = false;
  int currentHAngle = SERVO_H_HOME_ANGLE;
  int currentVAngle = SERVO_V_HOME_ANGLE;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  nightSnapshot = isNight;
  trackingSnapshot = trackingEnabled;
  currentHAngle = servoHAngle;
  currentVAngle = servoVAngle;
  xSemaphoreGive(dataMutex);

  if (nightSnapshot) {
    int hStepToHome = 0;
    int vStepToHome = 0;

    if (currentHAngle > SERVO_H_HOME_ANGLE) hStepToHome = -1;
    else if (currentHAngle < SERVO_H_HOME_ANGLE) hStepToHome = 1;

    if (currentVAngle > SERVO_V_HOME_ANGLE) vStepToHome = -1;
    else if (currentVAngle < SERVO_V_HOME_ANGLE) vStepToHome = 1;

    if (hStepToHome == 0 && vStepToHome == 0) {
      return;
    }

    int targetHAngle = currentHAngle + hStepToHome;
    int targetVAngle = currentVAngle + vStepToHome;

    setServoAngles(targetHAngle, targetVAngle, hStepToHome, vStepToHome);
    return;
  }

  if (!trackingSnapshot) {
    setServoAngles(currentHAngle, currentVAngle, 0, 0);
    return;
  }

  int tl = readAverage(LDR_TL);
  int tr = readAverage(LDR_TR);
  int bl = readAverage(LDR_BL);
  int br = readAverage(LDR_BR);

  int avgTop = (tl + tr) / 2;
  int avgBottom = (bl + br) / 2;
  int avgLeft = (tl + bl) / 2;
  int avgRight = (tr + br) / 2;

  float rawErrorH = (float)(avgTop - avgBottom);
  
  float rawErrorV = (float)(avgRight - avgLeft);

  filteredErrorH = (ERROR_FILTER_ALPHA * rawErrorH) + ((1.0f - ERROR_FILTER_ALPHA) * filteredErrorH);
  filteredErrorV = (ERROR_FILTER_ALPHA * rawErrorV) + ((1.0f - ERROR_FILTER_ALPHA) * filteredErrorV);

  int hStep = errorToStep(filteredErrorH, H_DIRECTION);
  int vStep = errorToStep(filteredErrorV, V_DIRECTION);

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  ldrTLValue = tl;
  ldrTRValue = tr;
  ldrBLValue = bl;
  ldrBRValue = br;
  lastErrorH = filteredErrorH;
  lastErrorV = filteredErrorV;
  xSemaphoreGive(dataMutex);

  int targetHAngle = clampAngle(currentHAngle + hStep, SERVO_H_MIN_ANGLE, SERVO_H_MAX_ANGLE);
  int targetVAngle = clampAngle(currentVAngle + vStep, SERVO_V_MIN_ANGLE, SERVO_V_MAX_ANGLE);

  setServoAngles(targetHAngle, targetVAngle, hStep, vStep);

  static unsigned long lastLogAt = 0;
  if (millis() - lastLogAt >= TRACKING_LOG_INTERVAL_MS) {
    lastLogAt = millis();
    Serial.print("EH:");
    Serial.print(filteredErrorH);
    Serial.print(" EV:");
    Serial.print(filteredErrorV);
    Serial.print(" H_angle:");
    Serial.print(targetHAngle);
    Serial.print(" V_angle:");
    Serial.print(targetVAngle);
    Serial.print(" H_step:");
    Serial.print(hStep);
    Serial.print(" V_step:");
    Serial.println(vStep);
  }
}

// ================= WEB HANDLERS =================
void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleToggle() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  setPumpLocked(!pumpState);
  xSemaphoreGive(dataMutex);
  server.send(200, "text/plain", "OK");
}

void handleToggleAuto() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  autoMode = !autoMode;
  xSemaphoreGive(dataMutex);
  server.send(200, "text/plain", "OK");
}

void handleToggleTracking() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  trackingEnabled = !trackingEnabled;
  if (!trackingEnabled) {
    servoHStepDeg = 0;
    servoVStepDeg = 0;
  }
  xSemaphoreGive(dataMutex);

  server.send(200, "text/plain", "OK");
}

void handleData() {
  char jsonBuffer[1280];
  unsigned long nowMs = millis();
  unsigned long pumpRemaining = 0;
  int rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  uint32_t freeHeap = ESP.getFreeHeap();

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (pumpState) {
    unsigned long elapsed = nowMs - pumpStartTime;
    pumpRemaining = elapsed >= autoOffTime ? 0 : (autoOffTime - elapsed) / 1000;
  }

  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"auto\":%s,\"pump\":%s,\"tracking\":%s,\"t\":%.2f,\"eff\":%.2f,"
           "\"v\":%.2f,\"i\":%.3f,\"p\":%.3f,\"lux\":%.1f,"
           "\"expected\":%.3f,\"night\":%s,\"cloud\":%s,\"ai\":%.3f,"
           "\"tl\":%d,\"tr\":%d,\"bl\":%d,\"br\":%d,"
           "\"eh\":%.1f,\"ev\":%.1f,\"sha\":%d,\"sva\":%d,\"sh\":%d,\"sv\":%d,"
           "\"uptime\":%lu,\"rssi\":%d,\"heap\":%lu,\"pumpRemaining\":%lu}",
           autoMode ? "true" : "false",
           pumpState ? "true" : "false",
           trackingEnabled ? "true" : "false",
           panelTempC,
           panelEfficiency,
           sysVolts,
           sysAmps,
           sysWatts,
           luxValue,
           expectedPowerAI,
           isNight ? "true" : "false",
           isCloudy ? "true" : "false",
           performanceRatio,
           ldrTLValue,
           ldrTRValue,
           ldrBLValue,
           ldrBRValue,
           lastErrorH,
           lastErrorV,
           servoHAngle,
           servoVAngle,
           servoHStepDeg,
           servoVStepDeg,
           (unsigned long)(nowMs / 1000),
           rssi,
           (unsigned long)freeHeap,
           pumpRemaining);
  xSemaphoreGive(dataMutex);

  server.send(200, "application/json", jsonBuffer);
}

// ================= TASKS =================
void trackingTask(void* pvParameters) {
  for (;;) {
    solarTracking180();
    vTaskDelay(pdMS_TO_TICKS(TRACKING_INTERVAL_MS));
  }
}

void sensorTask(void* pvParameters) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    float lux = lastKnownLux;
    float volts = sysVolts;
    float amps = sysAmps;
    float watts = sysWatts;
    xSemaphoreGive(dataMutex);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100))) {
      if (lightSensorReady) {
        sensors_event_t event;
        lightSensor.getEvent(&event);
        if (!isnan(event.light) && event.light >= 0.0f) {
          lux = event.light;
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          lastKnownLux = lux;
          xSemaphoreGive(dataMutex);
        }
      }

      if (ina219Ready) {
        float busVolts = ina219.getBusVoltage_V();
        float current_mA = ina219.getCurrent_mA();

        if (!isnan(busVolts) && !isnan(current_mA)) {
          volts = busVolts;
          amps = current_mA / 1000.0f;
          watts = volts * amps;
        }
      }

      xSemaphoreGive(i2cMutex);
    }

    tempSensor.requestTemperatures();
    float tC = tempSensor.getTempCByIndex(0);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    luxValue = lux;
    sysVolts = volts;
    sysAmps = amps;
    sysWatts = watts;
    if (tC != DEVICE_DISCONNECTED_C) {
      panelTempC = tC;
    }
    if (MAX_PANEL_POWER > 0.1f) {
      panelEfficiency = constrain((sysWatts / MAX_PANEL_POWER) * 100.0f, 0.0f, 200.0f);
    }
    xSemaphoreGive(dataMutex);

    handleNightMode(lux);
    updateCloudAndPerformance(lux, watts);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (!isNight) {
      bool hasSunlight = lux > SUNLIGHT_THRESHOLD;
      bool isPowerLow = performanceRatio < 0.6f;

      if (autoMode && !pumpState && (millis() - lastCleaning > cleaningCooldown)) {
        if (panelTempC > TEMP_THRESHOLD || (hasSunlight && isPowerLow && !isCloudy)) {
          setPumpLocked(true);
          lastCleaning = millis();
        }
      }
    }
    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void pumpTask(void* pvParameters) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (pumpState && (millis() - pumpStartTime > autoOffTime)) {
      setPumpLocked(false);
    }
    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

const char* wifiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID";
    case WL_SCAN_COMPLETED:
      return "SCAN_DONE";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void startWiFiAttempt() {
  Serial.print("WiFi connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(false, false);
  delay(80);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  wifiAttemptActive = true;
  wifiAttemptStartedAt = millis();
}

void handleWiFiConnection() {
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (wifiAttemptActive) {
      wifiAttemptActive = false;
      Serial.print("WiFi IP: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiAttemptActive) {
    if (millis() - wifiAttemptStartedAt < WIFI_CONNECT_TIMEOUT_MS) {
      return;
    }

    Serial.print("WiFi connect timeout, status: ");
    Serial.println(wifiStatusText(status));
    WiFi.disconnect(false, false);
    wifiAttemptActive = false;
    lastWifiRetryAt = millis();
    return;
  }

  if (millis() - lastWifiRetryAt >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryAt = millis();
    startWiFiAttempt();
  }
}

void serverTask(void* pvParameters) {
  for (;;) {
    handleWiFiConnection();

    if (WiFi.status() == WL_CONNECTED) {
      server.handleClient();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void beginDebugSerial() {
  Serial.begin(SERIAL_BAUD);

  unsigned long startMs = millis();
  while (!Serial && (millis() - startMs < SERIAL_WAIT_TIMEOUT_MS)) {
    delay(10);
  }

  Serial.println();
  Serial.print("Serial debug baud: ");
  Serial.println(SERIAL_BAUD);
}

bool createMutexes() {
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  servoMutex = xSemaphoreCreateMutex();

  if (i2cMutex == NULL || dataMutex == NULL || servoMutex == NULL) {
    Serial.println("Mutex creation failed");
    return false;
  }
  return true;
}

void setupAdc() {
  analogReadResolution(12);
  analogSetPinAttenuation(LDR_TL, ADC_11db);
  analogSetPinAttenuation(LDR_TR, ADC_11db);
  analogSetPinAttenuation(LDR_BL, ADC_11db);
  analogSetPinAttenuation(LDR_BR, ADC_11db);
}

void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  servoHAngle = SERVO_H_HOME_ANGLE;
  servoVAngle = SERVO_V_HOME_ANGLE;
  
  attachTrackingServos();
}

void setupSensors() {
  Wire.begin();

  lightSensorReady = lightSensor.begin();
  if (!lightSensorReady) {
    Serial.println("TSL2561 init failed");
  } else {
    lightSensor.enableAutoRange(true);
    lightSensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  }

  ina219Ready = ina219.begin();
  if (!ina219Ready) {
    Serial.println("INA219 init failed");
  }

  tempSensor.begin();
}

void setupWiFiAndServer() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  startWiFiAttempt();

  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/toggleAuto", handleToggleAuto);
  server.on("/toggleTracking", handleToggleTracking);
  server.on("/data", handleData);
  server.begin();
}

// ================= SETUP =================
void setup() {
  beginDebugSerial();

  if (!createMutexes()) {
    return;
  }

  setupAdc();

  pinMode(relayPin, OUTPUT);
  writePumpRelay(false);

  setupServos();
  setupSensors();
  setupWiFiAndServer();

  xTaskCreatePinnedToCore(trackingTask, "tracking", 5000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensor", 6000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(pumpTask, "pump", 2000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(serverTask, "server", 5000, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}