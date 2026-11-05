#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdlib.h>

#include "wifi_control.hpp"

namespace {

constexpr char AP_SSID[] = "MAGNESIS_ESP32";
constexpr char AP_PASSWORD[] = "magnesis123"; // GANTI sebelum dipakai.
constexpr uint8_t BUTTON_QUEUE_CAPACITY = 8;

WebServer server(80);
bool serverStarted = false;
WifiJoystickState joystick;
const char* currentModeLabel = "IDLE";

WifiButton buttonQueue[BUTTON_QUEUE_CAPACITY];
uint8_t buttonHead = 0;
uint8_t buttonCount = 0;

const char PAGE[] PROGMEM = R"MAGNESIS_HTML(
<!doctype html>
<html lang="id">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MAGNESIS | Robot Controller</title>
  <style>
    :root{color-scheme:dark;font-family:system-ui,-apple-system,Segoe UI,sans-serif}
    *{box-sizing:border-box}
    body{margin:0;min-height:100vh;background:#0b1322;color:#edf4ff;padding:20px;text-align:center}
    main{max-width:450px;margin:auto}
    h1{font-size:1.5rem;letter-spacing:.04em;margin:10px 0 4px}
    .subtitle{color:#a5b6d3;font-size:.9rem;margin:0 0 18px}
    .status{background:#17243b;border:1px solid #344965;border-radius:14px;padding:11px;margin-bottom:14px}
    .status strong{color:#a9ceff}
    .pad{position:relative;margin:14px auto 22px;width:min(74vw,300px);height:min(74vw,300px);
         border-radius:50%;border:3px solid #4d6a96;background:radial-gradient(circle,#253b5e,#101d33);
         touch-action:none;user-select:none;-webkit-user-select:none}
    .pad:before,.pad:after{content:"";position:absolute;background:#48617f;opacity:.45}
    .pad:before{width:1px;height:100%;left:50%;top:0}
    .pad:after{height:1px;width:100%;top:50%;left:0}
    .stick{position:absolute;left:50%;top:50%;width:76px;height:76px;border-radius:50%;
           transform:translate(-50%,-50%);background:#5b9bf6;border:4px solid #b3d5ff;
           box-shadow:0 5px 18px #0007;pointer-events:none}
    .buttons{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    button{border:0;border-radius:13px;padding:17px 8px;color:white;background:#30588d;
           font-size:1rem;font-weight:700;touch-action:manipulation}
    button:active{filter:brightness(1.25)}
    button.harvest{background:#735ba3}button.stop{background:#bd3148}
    .note{font-size:.83rem;line-height:1.5;color:#9db2d1;margin-top:20px}
  </style>
</head>
<body>
<main>
  <h1>MAGNESIS 🎮</h1>
  <p class="subtitle">Wi-Fi differential-drive controller</p>
  <div class="status"><div id="connection">Menghubungkan...</div><div>Mode: <strong id="mode">IDLE</strong></div></div>
  <div class="pad" id="pad" aria-label="Joystick virtual"><div class="stick" id="stick"></div></div>
  <div class="buttons">
    <button data-action="manual">MANUAL (O)</button>
    <button data-action="auto">AUTO (△)</button>
    <button data-action="harvest" class="harvest">HARVEST (□)</button>
    <button data-action="stop" class="stop">STOP (X)</button>
  </div>
  <p class="note">Lepas joystick untuk berhenti. Setelah masuk mode MANUAL, joystick harus netral dulu.
  Jika data joystick terputus, motor berhenti otomatis.</p>
</main>
<script>
(() => {
  const pad = document.getElementById('pad');
  const stick = document.getElementById('stick');
  const connection = document.getElementById('connection');
  const mode = document.getElementById('mode');
  let x = 0, y = 0, pointerId = null, joystickInFlight = false;

  function render() {
    const radius = Math.max(1, pad.clientWidth / 2 - stick.clientWidth / 2 - 5);
    stick.style.left = `calc(50% + ${x / 7 * radius}px)`;
    stick.style.top = `calc(50% - ${y / 7 * radius}px)`;
  }

  function move(e) {
    const rect = pad.getBoundingClientRect();
    const radius = Math.max(1, rect.width / 2 - stick.clientWidth / 2 - 5);
    let dx = (e.clientX - rect.left - rect.width / 2) / radius;
    let dy = (rect.top + rect.height / 2 - e.clientY) / radius;
    const magnitude = Math.hypot(dx, dy);
    if (magnitude > 1) { dx /= magnitude; dy /= magnitude; }
    x = dx * 7;
    y = dy * 7;
    render();
  }

  function neutral() { x = 0; y = 0; render(); }

  pad.addEventListener('pointerdown', e => {
    if (pointerId !== null) return;
    pointerId = e.pointerId;
    pad.setPointerCapture(pointerId);
    move(e);
  });
  pad.addEventListener('pointermove', e => {
    if (e.pointerId === pointerId) move(e);
  });
  function release(e) {
    if (e.pointerId !== pointerId) return;
    pointerId = null;
    neutral();
  }
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
  pad.addEventListener('lostpointercapture', release);

  async function sendJoystick() {
    if (document.hidden || joystickInFlight) return;
    joystickInFlight = true;
    try {
      const body = new URLSearchParams({x:x.toFixed(3), y:y.toFixed(3)});
      const response = await fetch('/api/joystick', {method:'POST', body, cache:'no-store'});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      connection.textContent = 'Terhubung ke robot';
    } catch (err) {
      connection.textContent = 'Koneksi joystick terputus';
    } finally {
      joystickInFlight = false;
    }
  }

  async function sendButton(action) {
    if (action === 'stop') neutral();
    try {
      const response = await fetch('/api/button', {
        method:'POST',
        body:new URLSearchParams({action}),
        cache:'no-store'
      });
      if (!response.ok) throw new Error('HTTP ' + response.status);
    } catch (err) {
      connection.textContent = 'Gagal mengirim tombol';
    }
  }

  document.querySelectorAll('[data-action]').forEach(button => {
    button.addEventListener('click', () => sendButton(button.dataset.action));
  });

  async function refreshMode() {
    try {
      const response = await fetch('/api/status', {cache:'no-store'});
      if (!response.ok) return;
      const status = await response.json();
      mode.textContent = status.mode;
    } catch (err) { /* Joystick timeout tetap menangani kegagalan koneksi. */ }
  }

  function suspend() {
    neutral();
    // Upaya best-effort mengirim netral; failsafe ESP32 tetap wajib.
    if (navigator.sendBeacon) {
      navigator.sendBeacon('/api/joystick', new URLSearchParams({x:'0', y:'0'}));
    }
  }
  window.addEventListener('blur', suspend);
  window.addEventListener('pagehide', suspend);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) suspend();
  });
  window.addEventListener('resize', render);

  render();
  sendJoystick();
  refreshMode();
  setInterval(sendJoystick, 70);
  setInterval(refreshMode, 500);
})();
</script>
</body>
</html>
)MAGNESIS_HTML";

void enqueueButton(WifiButton button) {
    if (button == WifiButton::NONE) return;

    // STOP menghapus perintah lama yang belum diproses.
    if (button == WifiButton::STOP) {
        buttonHead = 0;
        buttonCount = 0;
    }

    if (buttonCount >= BUTTON_QUEUE_CAPACITY) {
        // Jangan timpa tombol lama diam-diam ketika antrean penuh.
        Serial.println("[WIFI] Button queue full");
        return;
    }

    const uint8_t tail =
        (buttonHead + buttonCount) % BUTTON_QUEUE_CAPACITY;
    buttonQueue[tail] = button;
    ++buttonCount;
}

bool parseAxis(const char* name, float& result) {
    if (!server.hasArg(name)) return false;
    const String text = server.arg(name);
    char* end = nullptr;
    const float value = strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !isfinite(value)) {
        return false;
    }
    result = constrain(value, -7.0f, 7.0f);
    return true;
}

void handleJoystick() {
    float x = 0.0f;
    float y = 0.0f;
    if (!parseAxis("x", x) || !parseAxis("y", y)) {
        server.send(400, "text/plain", "Invalid joystick data");
        return;
    }

    joystick.x = x;
    joystick.y = y;
    joystick.receivedAt = millis();
    joystick.hasReceived = true;
    ++joystick.sequence;
    server.send(200, "text/plain", "OK");
}

void handleButton() {
    const String action = server.arg("action");
    WifiButton button = WifiButton::NONE;

    if (action == "manual") button = WifiButton::MANUAL;
    else if (action == "auto") button = WifiButton::AUTO;
    else if (action == "harvest") button = WifiButton::HARVEST;
    else if (action == "stop") button = WifiButton::STOP;
    else {
        server.send(400, "text/plain", "Unknown action");
        return;
    }

    enqueueButton(button);
    server.send(200, "text/plain", "OK");
}

void handleStatus() {
    String json = "{\"mode\":\"";
    json += currentModeLabel; // hanya string literal dari robotModeToString()
    json += "\",\"clients\":";
    json += WiFi.softAPgetStationNum();
    json += "}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

} // namespace

bool wifiControlInit() {
    if (serverStarted) return true;

    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.println("[WIFI] Failed to start Access Point");
        return false;
    }

    server.on("/", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html; charset=utf-8", PAGE);
    });
    server.on("/api/joystick", HTTP_POST, handleJoystick);
    server.on("/api/button", HTTP_POST, handleButton);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });

    server.begin();
    serverStarted = true;

    Serial.print("[WIFI] SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WIFI] Open http://");
    Serial.println(WiFi.softAPIP());
    return true;
}

void wifiControlUpdate() {
    if (serverStarted) server.handleClient();
}

WifiJoystickState wifiControlGetJoystick() {
    return joystick;
}

WifiButton wifiControlPopButton() {
    if (buttonCount == 0) return WifiButton::NONE;

    const WifiButton button = buttonQueue[buttonHead];
    buttonHead = (buttonHead + 1) % BUTTON_QUEUE_CAPACITY;
    --buttonCount;
    return button;
}

void wifiControlSetModeLabel(const char* modeLabel) {
    currentModeLabel = modeLabel ? modeLabel : "UNKNOWN";
}
