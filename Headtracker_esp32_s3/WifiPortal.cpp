// WifiPortal.cpp - WiFi AP + web configuration portal implementation
// No ArduinoJson dependency — manual JSON building for small payloads
#include "WifiPortal.h"

// Simple JSON key extraction helpers (avoids ArduinoJson dependency)
static bool jsonGetFloat(const String& json, const char* key, float& out) {
  String search = String("\"") + key + "\":";
  int idx = json.indexOf(search);
  if (idx < 0) return false;
  idx += search.length();
  out = json.substring(idx).toFloat();
  return true;
}

static bool jsonGetInt(const String& json, const char* key, int& out) {
  String search = String("\"") + key + "\":";
  int idx = json.indexOf(search);
  if (idx < 0) return false;
  idx += search.length();
  out = json.substring(idx).toInt();
  return true;
}

static bool jsonGetBool(const String& json, const char* key, bool& out) {
  String search = String("\"") + key + "\":";
  int idx = json.indexOf(search);
  if (idx < 0) return false;
  idx += search.length();
  String rest = json.substring(idx);
  rest.trim();
  out = rest.startsWith("true");
  return true;
}

// ========== Web UI HTML (single-page app) ==========

const char WifiPortal::INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Headtracker Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:16px;max-width:480px;margin:0 auto}
h1{text-align:center;color:#e94560;margin-bottom:16px;font-size:1.4em}
.section{background:#16213e;border-radius:8px;padding:12px;margin-bottom:12px}
.section h2{color:#e94560;font-size:1em;margin-bottom:8px}
label{display:block;margin:6px 0 2px;font-size:0.85em;color:#aaa}
input[type=range]{width:100%;margin:4px 0}
select{width:100%;padding:6px;background:#0f3460;color:#eee;border:1px solid #333;border-radius:4px}
.val{float:right;color:#e94560;font-weight:bold}
.check{display:flex;align-items:center;gap:8px}
.check input{width:18px;height:18px}
.bar-container{background:#0f3460;border-radius:4px;height:20px;margin:4px 0;overflow:hidden}
.bar-fill{height:100%;background:#e94560;transition:width 0.1s}
.btns{display:flex;gap:8px;margin-top:12px}
.btns button{flex:1;padding:10px;border:none;border-radius:6px;font-size:1em;cursor:pointer;font-weight:bold}
.btn-save{background:#e94560;color:#fff}
.btn-reboot{background:#0f3460;color:#eee}
.btn-defaults{background:#333;color:#eee}
.footer{text-align:center;color:#555;font-size:0.75em;margin-top:12px}
</style>
</head>
<body>
<h1>HEADTRACKER CONFIG</h1>

<div class="section">
<h2>PAN (Yaw)</h2>
<label>Channel <span class="val" id="panChVal"></span></label>
<select id="panCh"></select>
<div class="check"><input type="checkbox" id="panInv"><label>Inverted</label></div>
<label>Gain <span class="val" id="panGainVal"></span></label>
<input type="range" id="panGain" min="0.1" max="3.0" step="0.1">
<div class="bar-container"><div class="bar-fill" id="panBar" style="width:50%"></div></div>
</div>

<div class="section">
<h2>TILT (Pitch)</h2>
<label>Channel <span class="val" id="tiltChVal"></span></label>
<select id="tiltCh"></select>
<div class="check"><input type="checkbox" id="tiltInv"><label>Inverted</label></div>
<label>Gain <span class="val" id="tiltGainVal"></span></label>
<input type="range" id="tiltGain" min="0.1" max="3.0" step="0.1">
<div class="bar-container"><div class="bar-fill" id="tiltBar" style="width:50%"></div></div>
</div>

<div class="section">
<h2>Advanced</h2>
<label>Deadband <span class="val" id="dbVal"></span></label>
<input type="range" id="deadband" min="0" max="10" step="0.5">
<label>Max Pan <span class="val" id="maxPanVal"></span></label>
<input type="range" id="maxPan" min="30" max="180" step="5">
<label>Max Tilt <span class="val" id="maxTiltVal"></span></label>
<input type="range" id="maxTilt" min="30" max="180" step="5">
<label>Brightness <span class="val" id="brightVal"></span></label>
<input type="range" id="bright" min="0" max="255" step="1">
<label>Filter Beta <span class="val" id="betaVal"></span></label>
<input type="range" id="beta" min="0.01" max="0.5" step="0.01">
<div class="check"><input type="checkbox" id="buzzer"><label>Buzzer enabled</label></div>
</div>

<div class="btns">
<button class="btn-save" onclick="saveSettings()">SAVE</button>
<button class="btn-reboot" onclick="reboot()">REBOOT</button>
<button class="btn-defaults" onclick="defaults()">DEFAULTS</button>
</div>

<div class="footer" id="footer">Headtracker v1.0.0</div>

<script>
for(let s of ['panCh','tiltCh']){
  let el=document.getElementById(s);
  for(let i=0;i<16;i++){
    let o=document.createElement('option');
    o.value=i;o.text='CH'+(i+1);
    el.add(o);
  }
}
function bindSlider(id,valId,fmt){
  let el=document.getElementById(id);
  let vl=document.getElementById(valId);
  el.oninput=()=>{vl.textContent=fmt?fmt(el.value):el.value};
}
bindSlider('panGain','panGainVal');
bindSlider('tiltGain','tiltGainVal');
bindSlider('deadband','dbVal',v=>v+'\u00B0');
bindSlider('maxPan','maxPanVal',v=>'\u00B1'+v+'\u00B0');
bindSlider('maxTilt','maxTiltVal',v=>'\u00B1'+v+'\u00B0');
bindSlider('bright','brightVal',v=>Math.round(v/255*100)+'%');
bindSlider('beta','betaVal');

async function loadSettings(){
  try{
    let r=await fetch('/api/settings');
    let s=await r.json();
    document.getElementById('panCh').value=s.panChannel;
    document.getElementById('tiltCh').value=s.tiltChannel;
    document.getElementById('panInv').checked=s.panInverted;
    document.getElementById('tiltInv').checked=s.tiltInverted;
    document.getElementById('panGain').value=s.panGain;
    document.getElementById('tiltGain').value=s.tiltGain;
    document.getElementById('deadband').value=s.deadband;
    document.getElementById('maxPan').value=s.maxPan;
    document.getElementById('maxTilt').value=s.maxTilt;
    document.getElementById('bright').value=s.brightness;
    document.getElementById('beta').value=s.beta;
    document.getElementById('buzzer').checked=s.buzzerEnabled;
    for(let id of['panGain','tiltGain','deadband','maxPan','maxTilt','bright','beta']){
      document.getElementById(id).dispatchEvent(new Event('input'));
    }
    document.getElementById('panChVal').textContent='CH'+(parseInt(s.panChannel)+1);
    document.getElementById('tiltChVal').textContent='CH'+(parseInt(s.tiltChannel)+1);
  }catch(e){console.error('Load failed:',e)}
}

async function saveSettings(){
  let body={
    panChannel:parseInt(document.getElementById('panCh').value),
    tiltChannel:parseInt(document.getElementById('tiltCh').value),
    panInverted:document.getElementById('panInv').checked,
    tiltInverted:document.getElementById('tiltInv').checked,
    panGain:parseFloat(document.getElementById('panGain').value),
    tiltGain:parseFloat(document.getElementById('tiltGain').value),
    deadband:parseFloat(document.getElementById('deadband').value),
    maxPan:parseFloat(document.getElementById('maxPan').value),
    maxTilt:parseFloat(document.getElementById('maxTilt').value),
    brightness:parseInt(document.getElementById('bright').value),
    beta:parseFloat(document.getElementById('beta').value),
    buzzerEnabled:document.getElementById('buzzer').checked
  };
  try{
    let r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    if(r.ok)alert('Settings saved!');
    else alert('Save failed');
  }catch(e){alert('Error: '+e)}
}

async function reboot(){
  if(!confirm('Reboot device?'))return;
  try{await fetch('/api/reboot',{method:'POST'})}catch(e){}
  document.getElementById('footer').textContent='Rebooting...';
}

async function defaults(){
  if(!confirm('Reset all settings to factory defaults?'))return;
  try{await fetch('/api/defaults',{method:'POST'});loadSettings()}catch(e){alert('Error: '+e)}
}

async function pollState(){
  try{
    let r=await fetch('/api/state');
    let s=await r.json();
    let panPct=Math.max(0,Math.min(100,(s.pan-988)/(2012-988)*100));
    let tiltPct=Math.max(0,Math.min(100,(s.tilt-988)/(2012-988)*100));
    document.getElementById('panBar').style.width=panPct+'%';
    document.getElementById('tiltBar').style.width=tiltPct+'%';
    document.getElementById('footer').textContent='v1.0.0 | '+s.voltage.toFixed(1)+'V | '+(s.stable?'Stable':'Calibrating');
  }catch(e){}
}

loadSettings();
setInterval(pollState,100);
</script>
</body>
</html>
)rawliteral";

// ========== Implementation ==========

WifiPortal::WifiPortal()
  : server(nullptr), settings(nullptr), active(false),
    startTime(0), lastClientActivity(0),
    livePanUs(1500), liveTiltUs(1500),
    liveGyroStable(false), liveVoltage(0.0f) {
}

WifiPortal::~WifiPortal() {
  stop();
}

void WifiPortal::begin(Settings* settingsPtr) {
  // If already active, stop first
  if (active) stop();

  settings = settingsPtr;

  // Create fresh server instance (AsyncWebServer can't restart after end())
  if (server) {
    delete server;
    server = nullptr;
  }
  server = new AsyncWebServer(80);

  // Configure AP
  WiFi.mode(WIFI_AP);
  delay(100);  // Let WiFi stack settle
  WiFi.softAPConfig(IPAddress(10, 0, 0, 1), IPAddress(10, 0, 0, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(HtConfig::WIFI_SSID, HtConfig::WIFI_PASSWORD);
  delay(500);  // AP needs time to start before serving pages

  Serial.print("WiFi AP started: ");
  Serial.println(HtConfig::WIFI_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server->begin();

  active = true;
  startTime = millis();
  lastClientActivity = startTime;

  Serial.println("Web server started on port 80");
}

void WifiPortal::stop() {
  if (!active) return;

  if (server) {
    server->end();
    delete server;
    server = nullptr;
  }

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  active = false;
  Serial.println("WiFi portal stopped");
}

bool WifiPortal::isActive() const {
  return active;
}

bool WifiPortal::hasClient() const {
  return WiFi.softAPgetStationNum() > 0;
}

uint32_t WifiPortal::getActiveTime() const {
  if (!active) return 0;
  return millis() - startTime;
}

void WifiPortal::setLiveData(int panUs, int tiltUs, bool gyroStable, float voltage) {
  livePanUs = panUs;
  liveTiltUs = tiltUs;
  liveGyroStable = gyroStable;
  liveVoltage = voltage;
}

void WifiPortal::setupRoutes() {
  // Serve index page
  server->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // GET settings
  server->on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetSettings(request);
  });

  // POST settings (with body handler)
  server->on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest* request) {},  // empty request handler
    nullptr,  // no upload handler
    [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      handlePostSettings(request, data, len);
    }
  );

  // GET live state
  server->on("/api/state", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetState(request);
  });

  // POST reboot
  server->on("/api/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
    handleReboot(request);
  });

  // POST defaults
  server->on("/api/defaults", HTTP_POST, [this](AsyncWebServerRequest* request) {
    handleDefaults(request);
  });
}

void WifiPortal::handleGetSettings(AsyncWebServerRequest* request) {
  lastClientActivity = millis();

  const HeadtrackerSettings& s = settings->get();

  // Build JSON manually — small fixed-format payload, no library needed
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"panGain\":%.1f,\"tiltGain\":%.1f,"
    "\"panChannel\":%d,\"tiltChannel\":%d,"
    "\"panInverted\":%s,\"tiltInverted\":%s,"
    "\"deadband\":%.1f,\"maxPan\":%.1f,\"maxTilt\":%.1f,"
    "\"brightness\":%d,\"beta\":%.2f,"
    "\"buzzerEnabled\":%s}",
    s.panGain, s.tiltGain,
    s.panChannel, s.tiltChannel,
    s.panInverted ? "true" : "false",
    s.tiltInverted ? "true" : "false",
    s.deadbandDegrees, s.maxPanDegrees, s.maxTiltDegrees,
    s.brightness, s.madgwickBeta,
    s.buzzerEnabled ? "true" : "false"
  );

  request->send(200, "application/json", buf);
}

void WifiPortal::handlePostSettings(AsyncWebServerRequest* request, uint8_t* data, size_t len) {
  lastClientActivity = millis();

  // Parse JSON body manually using simple key search
  String json = String((char*)data).substring(0, len);

  HeadtrackerSettings& s = settings->getMutable();

  float fVal;
  int iVal;
  bool bVal;

  if (jsonGetFloat(json, "panGain", fVal))      s.panGain = fVal;
  if (jsonGetFloat(json, "tiltGain", fVal))      s.tiltGain = fVal;
  if (jsonGetInt(json, "panChannel", iVal))      s.panChannel = (uint8_t)iVal;
  if (jsonGetInt(json, "tiltChannel", iVal))     s.tiltChannel = (uint8_t)iVal;
  if (jsonGetBool(json, "panInverted", bVal))    s.panInverted = bVal;
  if (jsonGetBool(json, "tiltInverted", bVal))   s.tiltInverted = bVal;
  if (jsonGetFloat(json, "deadband", fVal))      s.deadbandDegrees = fVal;
  if (jsonGetFloat(json, "maxPan", fVal))        s.maxPanDegrees = fVal;
  if (jsonGetFloat(json, "maxTilt", fVal))       s.maxTiltDegrees = fVal;
  if (jsonGetInt(json, "brightness", iVal))      s.brightness = (uint8_t)iVal;
  if (jsonGetFloat(json, "beta", fVal))          s.madgwickBeta = fVal;
  if (jsonGetBool(json, "buzzerEnabled", bVal))  s.buzzerEnabled = bVal;

  settings->save();
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void WifiPortal::handleGetState(AsyncWebServerRequest* request) {
  lastClientActivity = millis();

  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"pan\":%d,\"tilt\":%d,\"stable\":%s,\"voltage\":%.2f}",
    livePanUs, liveTiltUs,
    liveGyroStable ? "true" : "false",
    liveVoltage
  );

  request->send(200, "application/json", buf);
}

void WifiPortal::handleReboot(AsyncWebServerRequest* request) {
  request->send(200, "application/json", "{\"status\":\"rebooting\"}");
  delay(500);
  ESP.restart();
}

void WifiPortal::handleDefaults(AsyncWebServerRequest* request) {
  settings->resetToDefaults();
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}
