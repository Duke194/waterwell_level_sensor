#include "web.h"

#include "config.h"
#include "diag.h"
#include "log.h"
#include "mqtt.h"
#include "ota_boot.h"
#include "ota_image.h"
#include "sensor.h"
#include "settings.h"
#include "status_led.h"

#include <ArduinoJson.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <string.h>
#include <strings.h>

namespace
{
WebServer webServer(80);
bool webMonitorRunning = false;
bool monitorForcedByFailure = false;

enum class OtaHttpResult
{
  None,
  Unauthorized,
  BadImage,
  FlashFailed,
  Reboot
};

OtaUploadGate otaGate;
OtaHttpResult otaHttpResult = OtaHttpResult::None;

void appendLabel(String &html, const char *name, const char *tip)
{
  html += "<label class='field-label'>";
  html += name;
  html += "<span class='tip' tabindex='0' data-tip='";
  html += tip;
  html += "'>?</span></label>";
}

void handleRoot()
{
  String html;
  html.reserve(16000);
  html +=
      "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>" MQTT_DEVICE_NAME " Monitor</title>"
      "<style>body{font-family:Arial,Helvetica,sans-serif;background:#0c111d;color:#f4f5f7;margin:0;padding:24px;}"
      "h1{margin-top:0;}h2{font-size:1.1rem;margin-top:0;}"
      ".card{background:#141b2d;border-radius:12px;padding:16px;box-shadow:0 4px 12px rgba(0,0,0,0.4);max-width:780px;margin-bottom:16px;}"
      ".status{margin-top:12px;font-size:0.92rem;color:#9dc6ff;}"
      ".kv{display:grid;grid-template-columns:180px 1fr;gap:8px;margin-top:10px;font-size:0.95rem;}"
      ".kv .k{color:#9ab;}.kv .v{font-weight:600;}"
      ".form label{display:block;margin-top:8px;color:#9ab;font-size:0.9rem;}"
      ".form .field-label{display:flex;align-items:center;margin-top:8px;color:#9ab;font-size:0.9rem;}"
      ".tip{position:relative;display:inline-flex;align-items:center;justify-content:center;width:16px;height:16px;margin-left:6px;border:1px solid #7f93b0;border-radius:50%;font-size:11px;line-height:1;color:#9dc6ff;cursor:help;flex:0 0 auto;}"
      ".tip:hover::after,.tip:focus::after{content:attr(data-tip);position:absolute;left:22px;top:-4px;z-index:5;width:260px;padding:8px 10px;background:#0a0f1c;border:1px solid #334155;border-radius:6px;color:#f4f5f7;font-size:12px;font-weight:400;line-height:1.35;white-space:normal;}"
      ".form input{width:100%;padding:8px;margin-top:4px;background:#1b2336;border:1px solid #22283b;color:#f4f5f7;border-radius:4px;box-sizing:border-box;}"
      ".checkbox{display:flex;align-items:center;gap:8px;margin-top:10px;}"
      ".checkbox input{width:auto;margin:0;}"
      ".checkbox .field-label{margin-top:0;}"
      ".form button{margin-top:12px;padding:8px 16px;background:#2563eb;color:#fff;border:none;border-radius:6px;cursor:pointer;}"
      ".form button:hover{background:#1d4ed8;}.msg{margin-left:8px;font-size:0.9rem;}"
      ".dev-row{display:flex;align-items:center;}"
      ".inline-btn{margin-left:auto;padding:4px 12px;background:#2563eb;color:#fff;border:none;border-radius:6px;cursor:pointer;font-size:0.85rem;font-weight:600;}"
      ".inline-btn:hover{background:#1d4ed8;}.inline-btn:disabled{background:#334155;cursor:default;}"
      "pre{background:#0a0f1c;border-radius:8px;padding:12px;overflow:auto;max-height:320px;white-space:pre-wrap;word-break:break-word;}</style>"
      "</head><body>"
      "<div class='card'><h1>" MQTT_DEVICE_NAME " Sensor Monitor</h1>";
  html += "<div class='status dev-row'>Firmware " FW_VERSION;
  const char *previous = otaOtherVersion();
  if (previous != nullptr)
  {
    html += "<button type='button' class='inline-btn' id='rollbackBtn'>Rollback ";
    html += previous;
    html += "</button>";
  }
  html += "</div>"
      "<div class='status' id='status'>Loading status...</div>"
      "<div class='kv'>"
      "<div class='k'>Boot count</div><div class='v' id='bootCount'>-</div>"
      "<div class='k'>Distance (raw)</div><div class='v' id='distanceRaw'>-</div>"
      "<div class='k'>Distance (effective)</div><div class='v' id='distanceEffective'>-</div>"
      "<div class='k'>IP address</div><div class='v' id='ip'>-</div>"
      "<div class='k'>Web monitor</div><div class='v' id='monitor'>-</div>"
      "<div class='k'>Battery mode</div><div class='v' id='batteryMode'>-</div>"
      "<div class='k'>DEV</div><div class='v dev-row'><span id='devMode'>-</span>"
      "<span class='tip' tabindex='0' data-tip='Misst alle 5 s und schreibt das ins Live-Log. MQTT bleibt beim eingestellten Sleep-Intervall.'>?</span>"
      "<button type='button' class='inline-btn' id='devToggle' disabled>...</button></div>"
      "</div>"
      "</div>"
      "<div class='card'><h2>Sensor Settings</h2><form class='form' id='settingsForm'>";

  appendLabel(html, "Area (m2)", "Grundfläche der Zisterne in m². Zusammen mit der Distanz ergibt das das Volumen.");
  html += "<input name='area' id='area' type='number' step='0.01' value='";
  html += String(settings.area, 2);
  html += "'>";

  appendLabel(html, "Well volume (L)", "Volumen in Litern, wenn die Zisterne bis zur Max-Füllhöhe voll ist.");
  html += "<input name='volume_well' id='volumeWell' type='number' step='0.01' value='";
  html += String(settings.volume_well, 2);
  html += "'>";

  appendLabel(html, "Sleep (s)", "Sekunden zwischen zwei Messungen. Im Batteriebetrieb die Deep-Sleep-Dauer, im Wachbetrieb auch das MQTT-Intervall.");
  html += "<input name='time_to_sleep' id='sleepSec' type='number' value='";
  html += String(settings.time_to_sleep);
  html += "'>";

  appendLabel(html, "Heartbeat (s)", "Spätestens nach dieser Zeit wird der letzte Wert gesendet, auch wenn sich der Pegel nicht geändert hat.");
  html += "<input name='heartbeat' id='heartbeatSec' type='number' value='";
  html += String(settings.heartbeat);
  html += "'>";

  appendLabel(html, "Average sample count", "So viele gültige Einzelmessungen werden zu einem Wert gemittelt.");
  html += "<input name='buffer_size' id='bufferSize' type='number' min='1' max='100' value='";
  html += String(settings.buffer_size);
  html += "'>";

  appendLabel(html, "Sensor range min (cm)", "Echo diese Distanz oder darunter gilt als zu nah und damit als ungültig.");
  html += "<input name='sensor_range_min' id='sensorRangeMin' type='number' min='0' max='1000' value='";
  html += String(settings.sensor_range_min);
  html += "'>";

  appendLabel(html, "Sensor range max (cm)", "Echo diese Distanz oder darüber wird verworfen.");
  html += "<input name='sensor_range_max' id='sensorRangeMax' type='number' min='1' max='2000' value='";
  html += String(settings.sensor_range_max);
  html += "'>";

  appendLabel(html, "Sensor to max fill (cm)", "Abstand vom Sensor bis zum Wasserspiegel bei voller Zisterne. Wird von der Rohdistanz abgezogen.");
  html += "<input name='sensor_to_max_fill_cm' id='sensorToMaxFillCm' type='number' min='0' max='1000' step='0.01' value='";
  html += String(settings.sensor_to_max_fill_cm, 2);
  html += "'>";

  appendLabel(html, "Calibration offset (cm)", "Fester Zuschlag in cm auf jede gemessene Distanz.");
  html += "<input name='calibration_offset' id='calibrationOffset' type='number' step='0.01' value='";
  html += String(settings.calibration_offset, 2);
  html += "'>";

  html += "<div class='checkbox'><input type='checkbox' id='batteryModeInput' name='battery_mode' value='1' ";
  html += settings.battery_mode ? "checked" : "";
  html += "><label class='field-label' for='batteryModeInput'>Battery mode"
          "<span class='tip' tabindex='0' data-tip='An schläft das Gerät zwischen den Messungen. Aus bleibt es wach und nutzt das Sleep-Intervall.'>?</span></label></div>";
  html += "<button type='submit'>Save settings</button><span class='msg' id='settingsMsg'></span>"
          "</form></div>"
          "<div class='card'><h2>MQTT Settings</h2><form class='form' id='mqttForm'>"
          "<label>Host</label><input name='host' id='mqttHost' type='text' value='";
  html += String(settings.mqtt_host);
  html += "'>"
          "<label>Port</label><input name='port' id='mqttPort' type='number' value='";
  html += String(settings.mqtt_port);
  html += "'>"
          "<label>User</label><input name='user' id='mqttUser' type='text' value='";
  html += String(settings.mqtt_user);
  html += "'>"
          "<label>Password</label><input name='pass' id='mqttPass' type='password' placeholder='Leave empty to keep current password'>"
          "<button type='submit'>Save MQTT</button><span class='msg' id='mqttMsg'></span>"
          "</form></div>";
  // Password must stay before the file input. It is read when the upload starts,
  // which only sees multipart fields that arrived earlier.
  html +=
          "<div class='card'><h2>Firmware Update</h2><form class='form' method='POST' action='/ota/upload' enctype='multipart/form-data'>"
          "<label>Password</label><input name='pw' type='password'>"
          "<label>Firmware (.bin)</label><input type='file' name='update' accept='.bin'>"
          "<button type='submit'>Flash firmware</button></form></div>"
          "<div class='card'><h2>Live Logs</h2><pre id='logs'>Loading logs...</pre></div>"
          "<script>"
          "let initialLoaded=false;let devOn=false;"
          "function updateDevUi(on){devOn=on;document.getElementById('devMode').textContent=on?'ON':'OFF';"
          "const b=document.getElementById('devToggle');b.textContent=on?'Turn DEV off':'Turn DEV on';b.disabled=false;}"
          "function setMsg(id,text,ok){const el=document.getElementById(id);el.textContent=text;el.style.color=ok?'#4ade80':'#f87171';}"
          "async function refreshStatus(){"
          "try{const res=await fetch('/status/json',{cache:'no-store'});if(!res.ok)throw new Error('status http '+res.status);"
          "const data=await res.json();document.getElementById('status').textContent='Last update: '+new Date().toLocaleTimeString();"
          "document.getElementById('bootCount').textContent=(data.bootCount!==undefined&&data.bootCount!==null)?data.bootCount:'-';"
          "document.getElementById('distanceRaw').textContent=((data.raw_distance_cm!==undefined&&data.raw_distance_cm!==null)?data.raw_distance_cm:'-')+' cm';"
          "document.getElementById('distanceEffective').textContent=((data.effective_distance_cm!==undefined&&data.effective_distance_cm!==null)?data.effective_distance_cm:'-')+' cm';"
          "document.getElementById('ip').textContent=data.ip||'-';"
          "document.getElementById('monitor').textContent=data.monitor?'ON':'OFF';"
          "document.getElementById('batteryMode').textContent=data.settings&&data.settings.battery_mode?'ON':'OFF';"
          "updateDevUi(!!data.dev_mode);"
          "if(!initialLoaded&&data.settings&&data.mqtt){"
          "document.getElementById('area').value=(data.settings.area!==undefined&&data.settings.area!==null)?data.settings.area:'';"
          "document.getElementById('volumeWell').value=(data.settings.volume_well!==undefined&&data.settings.volume_well!==null)?data.settings.volume_well:'';"
          "document.getElementById('sleepSec').value=(data.settings.time_to_sleep!==undefined&&data.settings.time_to_sleep!==null)?data.settings.time_to_sleep:'';"
          "document.getElementById('heartbeatSec').value=(data.settings.heartbeat!==undefined&&data.settings.heartbeat!==null)?data.settings.heartbeat:'';"
          "document.getElementById('bufferSize').value=(data.settings.buffer_size!==undefined&&data.settings.buffer_size!==null)?data.settings.buffer_size:'';"
          "document.getElementById('sensorRangeMin').value=(data.settings.sensor_range_min!==undefined&&data.settings.sensor_range_min!==null)?data.settings.sensor_range_min:'';"
          "document.getElementById('sensorRangeMax').value=(data.settings.sensor_range_max!==undefined&&data.settings.sensor_range_max!==null)?data.settings.sensor_range_max:'';"
          "document.getElementById('sensorToMaxFillCm').value=(data.settings.sensor_to_max_fill_cm!==undefined&&data.settings.sensor_to_max_fill_cm!==null)?data.settings.sensor_to_max_fill_cm:'';"
          "document.getElementById('calibrationOffset').value=(data.settings.calibration_offset!==undefined&&data.settings.calibration_offset!==null)?data.settings.calibration_offset:'';"
          "document.getElementById('batteryModeInput').checked=!!data.settings.battery_mode;"
          "document.getElementById('mqttHost').value=(data.mqtt.host!==undefined&&data.mqtt.host!==null)?data.mqtt.host:'';"
          "document.getElementById('mqttPort').value=(data.mqtt.port!==undefined&&data.mqtt.port!==null)?data.mqtt.port:'';"
          "document.getElementById('mqttUser').value=(data.mqtt.user!==undefined&&data.mqtt.user!==null)?data.mqtt.user:'';"
          "initialLoaded=true;}"
          "}catch(e){document.getElementById('status').textContent='Status refresh failed';}}"
          "async function refreshLogs(){"
          "try{const res=await fetch('/logs',{cache:'no-store'});document.getElementById('logs').textContent=await res.text();}"
          "catch(e){document.getElementById('logs').textContent='Failed to load logs';}}"
          "document.getElementById('devToggle').addEventListener('click',async function(){"
          "const btn=this;btn.disabled=true;const p=new URLSearchParams();p.set('enabled',devOn?'0':'1');"
          "try{const r=await fetch('/dev',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});"
          "if(r.ok){updateDevUi(!devOn);setTimeout(refreshStatus,500);}else{btn.disabled=false;}}"
          "catch(e){btn.disabled=false;}});"
          "const rollbackBtn=document.getElementById('rollbackBtn');"
          "if(rollbackBtn){rollbackBtn.addEventListener('click',async function(){"
          "if(!confirm(this.textContent+'?'))return;"
          "this.disabled=true;"
          "try{const r=await fetch('/ota/rollback',{method:'POST'});"
          "if(!r.ok){this.disabled=false;this.textContent='Rollback failed';}}"
          "catch(e){this.disabled=false;this.textContent='Rollback failed';}});}"
          "document.getElementById('settingsForm').addEventListener('submit',async function(e){e.preventDefault();"
          "const p=new URLSearchParams();p.set('area',document.getElementById('area').value);p.set('volume_well',document.getElementById('volumeWell').value);"
          "p.set('time_to_sleep',document.getElementById('sleepSec').value);p.set('heartbeat',document.getElementById('heartbeatSec').value);"
          "p.set('buffer_size',document.getElementById('bufferSize').value);"
          "p.set('sensor_range_min',document.getElementById('sensorRangeMin').value);"
          "p.set('sensor_range_max',document.getElementById('sensorRangeMax').value);"
          "p.set('sensor_to_max_fill_cm',document.getElementById('sensorToMaxFillCm').value);"
          "p.set('calibration_offset',document.getElementById('calibrationOffset').value);"
          "if(document.getElementById('batteryModeInput').checked){p.set('battery_mode','1');}"
          "const r=await fetch('/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});"
          "setMsg('settingsMsg',r.ok?'Saved':'Save failed',r.ok);if(r.ok){refreshStatus();}});"
          "document.getElementById('mqttForm').addEventListener('submit',async function(e){e.preventDefault();"
          "const p=new URLSearchParams();p.set('host',document.getElementById('mqttHost').value);p.set('port',document.getElementById('mqttPort').value);"
          "p.set('user',document.getElementById('mqttUser').value);const pass=document.getElementById('mqttPass').value;if(pass){p.set('pass',pass);}"
          "const r=await fetch('/mqtt/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});"
          "setMsg('mqttMsg',r.ok?'Saved (reconnect)...':'Save failed',r.ok);if(r.ok){document.getElementById('mqttPass').value='';setTimeout(refreshStatus,1500);}});"
          "refreshStatus();refreshLogs();setInterval(refreshStatus,5000);setInterval(refreshLogs,3000);"
          "</script></body></html>";
  webServer.send(200, "text/html", html);
}

void handleStatusJson()
{
  StaticJsonDocument<768> doc;
  doc["bootCount"] = currentBootCount();
  doc["ip"] = WiFi.localIP().toString();
  doc["monitor"] = webMonitorRunning;
  doc["dev_mode"] = devModeEnabled();
  if (hasValidDistance())
  {
    const int distanceCm = lastValidDistanceCm();
    doc["distance_cm"] = distanceCm;
    doc["raw_distance_cm"] = distanceCm;
    doc["effective_distance_cm"] = computeEffectiveDistanceCm(distanceCm);
  }
  else
  {
    doc["distance_cm"] = nullptr;
    doc["raw_distance_cm"] = nullptr;
    doc["effective_distance_cm"] = nullptr;
  }
  JsonObject sensorSettings = doc.createNestedObject("settings");
  sensorSettings["time_to_sleep"] = settings.time_to_sleep;
  sensorSettings["heartbeat"] = settings.heartbeat;
  sensorSettings["wifi_timeout"] = settings.wifi_timeout;
  sensorSettings["sensor_timeout"] = settings.sensor_timeout;
  sensorSettings["buffer_size"] = settings.buffer_size;
  sensorSettings["sensor_range_min"] = settings.sensor_range_min;
  sensorSettings["sensor_range_max"] = settings.sensor_range_max;
  sensorSettings["sensor_to_max_fill_cm"] = settings.sensor_to_max_fill_cm;
  sensorSettings["calibration_offset"] = settings.calibration_offset;
  sensorSettings["area"] = settings.area;
  sensorSettings["volume_well"] = settings.volume_well;
  sensorSettings["battery_mode"] = settings.battery_mode;
  JsonObject mqtt = doc.createNestedObject("mqtt");
  mqtt["host"] = settings.mqtt_host;
  mqtt["port"] = settings.mqtt_port;
  mqtt["user"] = settings.mqtt_user;
  mqtt["pass"] = "***";
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleLogs()
{
  webServer.send(200, "text/plain", getLogs());
}

void handleSettingsPost()
{
  if (webServer.hasArg("area"))
    settings.area = webServer.arg("area").toFloat();
  if (webServer.hasArg("volume_well"))
    settings.volume_well = webServer.arg("volume_well").toFloat();
  if (webServer.hasArg("time_to_sleep"))
    settings.time_to_sleep = webServer.arg("time_to_sleep").toInt();
  if (webServer.hasArg("heartbeat"))
    settings.heartbeat = webServer.arg("heartbeat").toInt();
  if (webServer.hasArg("buffer_size"))
    settings.buffer_size = webServer.arg("buffer_size").toInt();
  if (webServer.hasArg("sensor_range_min"))
    settings.sensor_range_min = webServer.arg("sensor_range_min").toInt();
  if (webServer.hasArg("sensor_range_max"))
    settings.sensor_range_max = webServer.arg("sensor_range_max").toInt();
  if (webServer.hasArg("sensor_to_max_fill_cm"))
    settings.sensor_to_max_fill_cm = webServer.arg("sensor_to_max_fill_cm").toFloat();
  if (webServer.hasArg("calibration_offset"))
    settings.calibration_offset = webServer.arg("calibration_offset").toFloat();

  if (settings.sensor_range_min < 0)
    settings.sensor_range_min = 0;
  if (settings.sensor_range_max <= settings.sensor_range_min)
    settings.sensor_range_max = settings.sensor_range_min + 1;
  if (settings.sensor_to_max_fill_cm < 0.0f)
    settings.sensor_to_max_fill_cm = 0.0f;
  if (settings.buffer_size < 1)
    settings.buffer_size = 1;
  settings.battery_mode = webServer.hasArg("battery_mode");
  saveSettingsToNvs();
  publishLastValidState();
  webServer.send(200, "text/plain", "OK");
}

void handleDevPost()
{
  bool enable = false;
  if (webServer.hasArg("enabled"))
  {
    const String value = webServer.arg("enabled");
    enable = value == "1" || value.equalsIgnoreCase("on") || value.equalsIgnoreCase("true");
  }
  setDevMode(enable);
  webServer.send(200, "text/plain", "OK");
}

void handleMqttConfigPost()
{
  if (!webServer.hasArg("host") || !webServer.hasArg("port") || !webServer.hasArg("user"))
  {
    webServer.send(400, "text/plain", "host, port, user required");
    return;
  }
  strlcpy(settings.mqtt_host, webServer.arg("host").c_str(), sizeof(settings.mqtt_host));
  settings.mqtt_port = webServer.arg("port").toInt();
  strlcpy(settings.mqtt_user, webServer.arg("user").c_str(), sizeof(settings.mqtt_user));
  if (webServer.hasArg("pass") && webServer.arg("pass").length() > 0)
  {
    strlcpy(settings.mqtt_pass, webServer.arg("pass").c_str(), sizeof(settings.mqtt_pass));
  }
  saveSettingsToNvs();
  mqttDisconnect();
  webServer.send(200, "text/plain", "OK");
}

void rejectOta(OtaHttpResult result)
{
  otaHttpResult = result;
  diagSetError(ErrorSource::Ota, "OTA: Update abgelehnt");
  Update.abort();
  logLine("OTA rejected: %s", otaGateReason(&otaGate));
}

void handleOtaUpload()
{
  HTTPUpload &upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    otaHttpResult = OtaHttpResult::None;
    diagClearError(ErrorSource::Ota);
    const bool passwordOk = webServer.hasArg("pw") && webServer.arg("pw") == OTA_SHARED_SECRET;
    if (!otaGateStart(&otaGate, passwordOk, otaNextSlotSize()))
    {
      rejectOta(passwordOk ? OtaHttpResult::BadImage : OtaHttpResult::Unauthorized);
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    {
      otaGateNoteFlashBegin(&otaGate, false);
      Update.printError(Serial);
      rejectOta(OtaHttpResult::FlashFailed);
      return;
    }
    otaGateNoteFlashBegin(&otaGate, true);
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (otaHttpResult != OtaHttpResult::None)
    {
      return;
    }
    if (!otaGateWrite(&otaGate, upload.buf, upload.currentSize))
    {
      rejectOta(OtaHttpResult::BadImage);
      return;
    }
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
    {
      otaGateNoteFlashWrite(&otaGate, false);
      Update.printError(Serial);
      rejectOta(OtaHttpResult::FlashFailed);
      return;
    }
    otaGateNoteFlashWrite(&otaGate, true);
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (otaHttpResult != OtaHttpResult::None)
    {
      Update.abort();
      return;
    }
    const bool accepted = otaGateFinish(&otaGate);
    // end(true) is what switches the boot partition, and it does that even
    // when the file is shorter than the slot. Only a fully checked image
    // may get that far. A failed end must not reboot.
    if (!otaMayReboot(accepted, true))
    {
      rejectOta(otaGate.password_ok ? OtaHttpResult::BadImage : OtaHttpResult::Unauthorized);
      return;
    }
    const bool ended = Update.end(true);
    if (!otaMayReboot(accepted, ended))
    {
      Update.printError(Serial);
      otaHttpResult = OtaHttpResult::FlashFailed;
      diagSetError(ErrorSource::Ota, "OTA: Flashen fehlgeschlagen");
      logLine("OTA rejected: flash end failed");
      return;
    }
    otaHttpResult = OtaHttpResult::Reboot;
    logLine("OTA image accepted, rebooting");
  }
}
} // namespace

void handleOtaRollback()
{
  const char *previous = otaOtherVersion();
  if (previous == nullptr || !otaRollbackToOther())
  {
    webServer.send(409, "text/plain", "no previous firmware");
    return;
  }
  logLine("Rolling back to %s", previous);
  webServer.send(200, "text/plain", "OK rebooting");
  delay(300);
  ESP.restart();
}

static void registerWebHandlers()
{
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/status/json", HTTP_GET, handleStatusJson);
  webServer.on("/logs", HTTP_GET, handleLogs);
  webServer.on("/settings", HTTP_POST, handleSettingsPost);
  webServer.on("/dev", HTTP_POST, handleDevPost);
  webServer.on("/ota/rollback", HTTP_POST, handleOtaRollback);
  webServer.on("/mqtt/config", HTTP_POST, handleMqttConfigPost);
  webServer.on(
      "/ota/upload", HTTP_POST,
      []()
      {
        if (otaHttpResult == OtaHttpResult::Reboot)
        {
          webServer.send(200, "text/plain", "OK rebooting");
          statusLedBlinkFor(StatusLed::Online, 5000);
          ESP.restart();
          return;
        }
        if (otaHttpResult == OtaHttpResult::Unauthorized)
        {
          webServer.send(401, "text/plain", "invalid password");
          return;
        }
        if (otaHttpResult == OtaHttpResult::FlashFailed)
        {
          webServer.send(500, "text/plain", "OTA failed");
          return;
        }
        const char *reason = otaHttpResult == OtaHttpResult::BadImage ? otaGateReason(&otaGate) : "no firmware";
        webServer.send(400, "text/plain", reason);
      },
      handleOtaUpload);
}

void startWebMonitor()
{
  static bool handlersRegistered = false;
  if (!handlersRegistered)
  {
    handlersRegistered = true;
    registerWebHandlers();
  }
  // begin() drops a stale listener first, so every start reopens the socket.
  // That keeps the MQTT switch reliable even if the flag says "running" but
  // the listener died after a Wi-Fi reconnect.
  webServer.begin();
  logLine("OTA marker %s", otaImageMarker());
  webMonitorRunning = true;
  mqttPublishMonitorState();
  logLine("Web monitor started on http://%s/", WiFi.localIP().toString().c_str());
}

void stopWebMonitor()
{
  if (!webMonitorRunning)
    return;
  webServer.stop();
  webMonitorRunning = false;
  mqttPublishMonitorState();
  logLine("Web monitor stopped");
}

void webHandleClient()
{
  webServer.handleClient();
}

bool webMonitorIsRunning()
{
  return webMonitorRunning;
}

bool webMonitorForcedByFailure()
{
  return monitorForcedByFailure;
}

void setWebMonitorEnabled(bool enable, bool persist)
{
  if (!enable)
  {
    // If user explicitly disables monitor, clear fallback latch.
    monitorForcedByFailure = false;
  }
  settings.web_monitor_enabled = enable;
  if (enable)
  {
    startWebMonitor();
  }
  else
  {
    stopWebMonitor();
  }
  if (persist)
  {
    saveSettingsToNvs();
  }
  mqttPublishMonitorState();
}

void webForceMonitorOnMqttFailure()
{
  // Fallback only: do not touch the stored monitor setting.
  monitorForcedByFailure = true;
  if (!webMonitorRunning)
  {
    startWebMonitor();
  }
}

void webReopenListener()
{
  if (webMonitorRunning)
  {
    webServer.begin();
    logLine("Web monitor listener reopened");
  }
}
