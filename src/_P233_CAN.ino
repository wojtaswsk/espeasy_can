#include "_Plugin_Helper.h"
#include <ACAN2515.h>
#include <SPI.h>
#include <Preferences.h>

#ifdef USES_P233
#define PLUGIN_233
#define PLUGIN_ID_233 233
#define PLUGIN_NAME_233 "CAN 2.0"

// ---------------- PCONFIG ----------------
#define P233_CONFIG_SEND        PCONFIG(0) // 1=TX, 0=RX
#define P233_CONFIG_GPIO        PCONFIG(1)
#define P233_CONFIG_DEVICE_ID   PCONFIG(2) // RX
#define P233_CONFIG_TARGET_ID   PCONFIG(3) // TX
#define P233_CONFIG_DOMO_IDX    PCONFIG(4) // TX: Domoticz idx
// PCONFIG(5) – wolne (kiedyś invert TX)
#define P233_CONFIG_INVERT_RX   PCONFIG(6) // RX: invert output logic (dla przekaźników aktywnych niskim)

// -------------- MCP2515 pins -------------
#define MCP2515_CS  10
#define MCP2515_INT 14   // unikaj pinów strap/boot

// -------------- Includes Net -------------
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

#include "src/ESPEasyCore/Controller.h"
#include "src/DataStructs/ControllerSettingsStruct.h"
#include "src/Helpers/ESPEasy_Storage.h"

// -------------- CAN setup ----------------
static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL;
static ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);
static ACAN2515Settings settings(QUARTZ_FREQUENCY, 125UL * 1000UL);

// -------------- Global state -------------
static bool     canInitialized = false;
static uint8_t  rxMasterTask   = 255;

static int             lastGpioState[ TASKS_MAX ];       // surowy odczyt pinu (TX) / ostatni zapis (RX)
static int             toggleStateArr[ TASKS_MAX ];
static unsigned long   lastDebounceTimeArr[ TASKS_MAX ];
static const unsigned  debounceDelay = 50; // ms

Preferences prefs;

// -------- Controller #0 cache ------------
static IPAddress g_ctrlIp;           
static uint16_t  g_ctrlPort = 0;
static bool      g_ctrlReady = false; 
static String    g_ctrlHost;          
static bool      g_ctrlHaveHost = false;
static uint32_t  g_dnsNextTryMs = 0;  

// -------- Domoticz raw queue -------------
static volatile bool     domoBusy          = false;
static volatile int8_t   domoPendingCmd    = -1; // -1 brak, 0=Off, 1=On
static volatile uint16_t domoPendingIdx    = 0;
static volatile uint32_t domoNotBeforeMs   = 0;  // grace po starcie

// --- Circuit breaker & throttle dla Domoticza
static uint32_t domoBreakerUntil = 0;
static uint8_t  domoFailStreak   = 0;
static uint32_t domoNextTryMs    = 0;

// --- Priorytet dla CAN i szybka ścieżka gdy Domoticz OK
static volatile uint32_t lastCanEdgeMs    = 0;
static uint32_t          lastDomoticzOkMs = 0;

// --- Echo guard x2 (per TX task) ---
#define ECHO_GUARD_MS    900
#define ECHO_GUARD_SLOTS 2
static uint32_t domoEchoGuardUntil [ TASKS_MAX ][ ECHO_GUARD_SLOTS ];
static int8_t   domoEchoGuardState [ TASKS_MAX ][ ECHO_GUARD_SLOTS ]; // -1 brak, 0=LOW, 1=HIGH
static uint8_t  domoEchoGuardWptr  [ TASKS_MAX ];

// --------- Log buffer (prosty) ----------
static char p233_logbuf[192];

static inline void P233_updateValue(uint8_t ti, int state) {
  if (ti >= TASKS_MAX) return;
  float val = (state == HIGH) ? 1.0f : 0.0f;
  UserVar.setFloat(ti, 0, val);
  EventStruct ev;
  ev.TaskIndex = ti;
  sendData(&ev);
}




// --------------- CAN ISR -----------------
static void canISR() { can.isr(); }

// ---------- Helpers / logging ------------
static inline bool IPv4Ready() {
#ifdef ESP32
  if (WiFi.status() != WL_CONNECTED) return false;
#endif
  IPAddress ip = WiFi.localIP();
  return !(ip[0]==0 && ip[1]==0 && ip[2]==0 && ip[3]==0);
}
static void logI(const char* msg) { addLog(LOG_LEVEL_INFO,  String(msg)); }
static void logE(const char* msg) { addLog(LOG_LEVEL_ERROR, String(msg)); }
static void logD(const char* msg) { addLog(LOG_LEVEL_DEBUG, String(msg)); }

// --- Wczytaj Controller #0 bez żadnej sieci/DNS ---
static void ReadController0ConfigOnly(String &hostOut, IPAddress &ipOut, uint16_t &portOut, bool &haveHost) {
  ControllerSettingsStruct cs;
  const String err = LoadControllerSettings((controllerIndex_t)0, cs);
  ipOut  = IPAddress(0,0,0,0);
  portOut = 0;
  hostOut = "";
  haveHost = false;

  if (err.length()) {
    snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: LoadControllerSettings#0 ERR: %s", err.c_str());
    logE(p233_logbuf);
    return;
  }
  portOut = cs.Port;
  ipOut   = cs.getIP();
  hostOut = cs.getHost();
  haveHost = hostOut.length() > 0;

  if (ipOut != IPAddress(0,0,0,0)) {
    snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: Ctrl0 IP=%u.%u.%u.%u port=%u",
             (unsigned)ipOut[0], (unsigned)ipOut[1], (unsigned)ipOut[2], (unsigned)ipOut[3], (unsigned)portOut);
    logI(p233_logbuf);
  } else if (haveHost) {
    snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: Ctrl0 host=%s port=%u (DNS później)",
             hostOut.c_str(), (unsigned)portOut);
    logI(p233_logbuf);
  } else {
    logI("P233: Ctrl0 not configured");
  }
}

// --- Spróbuj rozwiązać host→IP gdy Wi-Fi gotowe (throttlowane) ---
static void TryResolveControllerHostIfNeeded() {
  if (g_ctrlReady) return;
  if (!g_ctrlHaveHost || g_ctrlHost.isEmpty()) return;
  if (!IPv4Ready()) return;
  const uint32_t now = millis();
  if ((int32_t)(now - g_dnsNextTryMs) < 0) return;

  IPAddress resolved;
  if (WiFi.hostByName(g_ctrlHost.c_str(), resolved)) {
    g_ctrlIp = resolved;
    g_ctrlReady = (g_ctrlPort != 0);
    snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: DNS %s -> %u.%u.%u.%u",
             g_ctrlHost.c_str(), (unsigned)resolved[0], (unsigned)resolved[1], (unsigned)resolved[2], (unsigned)resolved[3]);
    logI(p233_logbuf);
  } else {
    snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: DNS resolve failed for %s", g_ctrlHost.c_str());
    logE(p233_logbuf);
  }
  g_dnsNextTryMs = now + 2000;
}

// ------------- CAN RX dispatchers -------------
static void p233_dispatchRx_0x123(uint8_t stateBit, uint8_t targetDevId, uint8_t senderId) {
  int setVal = stateBit ? HIGH : LOW;

  for (uint8_t ti2 = 0; ti2 < TASKS_MAX; ++ti2) {
    if (Settings.TaskDeviceNumber[ti2] != PLUGIN_ID_233) continue;
    if (!Settings.TaskDeviceEnabled[ti2]) continue;

    const bool isSend   = Settings.TaskDevicePluginConfig[ti2][0] != 0;
    const uint8_t gpio  = (uint8_t)Settings.TaskDevicePluginConfig[ti2][1];
    const uint8_t devId = (uint8_t)Settings.TaskDevicePluginConfig[ti2][2];

    if (isSend) continue;
    if (devId != targetDevId) continue;

    // RX invert?
    if (Settings.TaskDevicePluginConfig[ti2][6] != 0) {
      setVal = (setVal == HIGH) ? LOW : HIGH;
    }

    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, setVal);
    lastGpioState[ti2] = setVal;
    P233_updateValue(ti2, setVal);


    snprintf(p233_logbuf, sizeof(p233_logbuf),
             "P233 RX: from=%u to=%u task=%u GPIO=%u <- %s (invertRX=%s)",
             (unsigned)senderId, (unsigned)devId, (unsigned)ti2, (unsigned)gpio,
             (setVal==HIGH)?"HIGH":"LOW",
             (Settings.TaskDevicePluginConfig[ti2][6]!=0)?"ON":"OFF");
    logI(p233_logbuf);
  }
}

static void p233_dispatchRx_0x125(uint8_t helloDevId) {
  for (uint8_t ti2 = 0; ti2 < TASKS_MAX; ++ti2) {
    if (Settings.TaskDeviceNumber[ti2] != PLUGIN_ID_233) continue;
    if (!Settings.TaskDeviceEnabled[ti2]) continue;

    const bool isSend      = Settings.TaskDevicePluginConfig[ti2][0] != 0;
    const uint8_t targetId = (uint8_t)Settings.TaskDevicePluginConfig[ti2][3];
    if (!isSend || targetId != helloDevId) continue;

    const uint8_t senderId = (uint8_t)Settings.Unit;
    CANMessage msg;
    msg.id      = 0x123;
    msg.len     = 3;
    msg.data[0] = (uint8_t)toggleStateArr[ti2];
    msg.data[1] = (uint8_t)targetId;
    msg.data[2] = senderId;
    (void)can.tryToSend(msg);
  }
}

// ---- Lokalny loopback (sam-do-siebie) ----
static void p233_localLoop_applyIfLocal(uint8_t targetDevId, uint8_t stateBit, uint8_t senderId) {
  int setValBase = stateBit ? HIGH : LOW;

  for (uint8_t ti2 = 0; ti2 < TASKS_MAX; ++ti2) {
    if (!Settings.TaskDeviceEnabled[ti2]) continue;
    if (Settings.TaskDeviceNumber[ti2] != PLUGIN_ID_233) continue;

    const bool isSend = Settings.TaskDevicePluginConfig[ti2][0] != 0;
    if (isSend) continue;

    const uint8_t devId = (uint8_t)Settings.TaskDevicePluginConfig[ti2][2];
    if (devId != targetDevId) continue;

    const uint8_t gpio  = (uint8_t)Settings.TaskDevicePluginConfig[ti2][1];

    int setVal = setValBase;
    if (Settings.TaskDevicePluginConfig[ti2][6] != 0) { // RX invert
      setVal = (setVal == HIGH) ? LOW : HIGH;
    }

    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, setVal);
    lastGpioState[ti2] = setVal;
    P233_updateValue(ti2, setVal);


    snprintf(p233_logbuf, sizeof(p233_logbuf),
             "P233 LOCAL-LOOP: from=%u to=%u task=%u GPIO=%u <- %s (invertRX=%s)",
             (unsigned)senderId, (unsigned)devId, (unsigned)ti2, (unsigned)gpio,
             (setVal==HIGH)?"HIGH":"LOW",
             (Settings.TaskDevicePluginConfig[ti2][6]!=0)?"ON":"OFF");
    logI(p233_logbuf);
  }
}

// ---- Echo guard helpers ----
static inline void echo_guard_push(uint8_t ti, int8_t state01) {
  uint8_t w = domoEchoGuardWptr[ti] % ECHO_GUARD_SLOTS;
  domoEchoGuardState[ti][w] = state01;                  
  domoEchoGuardUntil[ti][w] = millis() + ECHO_GUARD_MS; 
  domoEchoGuardWptr[ti]     = (w + 1) & (ECHO_GUARD_SLOTS - 1);
}

static inline bool echo_guard_match_and_consume(uint8_t ti, int8_t incomingState01, uint8_t &hitSlot) {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < ECHO_GUARD_SLOTS; ++i) {
    if (domoEchoGuardState[ti][i] < 0) continue;
    if ((int32_t)(now - domoEchoGuardUntil[ti][i]) >= 0) {
      domoEchoGuardState[ti][i] = -1;
      domoEchoGuardUntil[ti][i] = 0;
      continue;
    }
    if (domoEchoGuardState[ti][i] == incomingState01) {
      hitSlot = i;
      domoEchoGuardState[ti][i] = -1;
      domoEchoGuardUntil[ti][i] = 0;
      return true;
    }
  }
  return false;
}

// ---- TX helper: ustaw/zmień stan jak przy zboczu (bez echo do Domoticza) ----
static void p233_tx_apply(uint8_t ti, int setMode /*-1 toggle, 0 off, 1 on*/) {
  if (ti >= TASKS_MAX) return;
  if (Settings.TaskDeviceNumber[ti] != PLUGIN_ID_233) return;
  if (!Settings.TaskDeviceEnabled[ti]) return;
  const bool isSend = Settings.TaskDevicePluginConfig[ti][0] != 0;
  if (!isSend) return;

  int newState = toggleStateArr[ti];
  if (setMode == -1)      newState = (toggleStateArr[ti] == HIGH) ? LOW : HIGH;
  else if (setMode == 0)  newState = LOW;
  else if (setMode == 1)  newState = HIGH;

  if (newState == toggleStateArr[ti]) return;

  toggleStateArr[ti] = newState;
  P233_updateValue(ti, toggleStateArr[ti]);


  // zapis NVS
  prefs.begin("plugin233", false);
  String key2 = String(F("lastState_")) + String(ti);
  prefs.putUChar(key2.c_str(), (uint8_t)toggleStateArr[ti]);
  prefs.end();

  // wyślij CAN do targetu
  const uint8_t senderId = (uint8_t)Settings.Unit;
  const uint8_t targetId = (uint8_t)Settings.TaskDevicePluginConfig[ti][3];

  CANMessage msg;
  msg.id      = 0x123;
  msg.len     = 3;
  msg.data[0] = (uint8_t)toggleStateArr[ti];
  msg.data[1] = (uint8_t)targetId;
  msg.data[2] = senderId;
  bool ok = can.tryToSend(msg);

  snprintf(p233_logbuf, sizeof(p233_logbuf),
           "P233 TX(write): ti=%u newState=%s target=%u sender=%u send=%s",
           (unsigned)ti, toggleStateArr[ti] ? "HIGH" : "LOW", (unsigned)targetId, (unsigned)senderId, ok ? "OK" : "FAIL");
  (ok ? logI : logE)(p233_logbuf);

  // Lokalny loopback (sam-do-siebie)
  p233_localLoop_applyIfLocal(targetId, (uint8_t)toggleStateArr[ti], senderId);

  lastCanEdgeMs = millis();
}

// --- helper: znajdź nadajnik po Domoticz idx ---
static bool P233_findTaskByIdx(uint16_t idx, uint8_t &outTi) {
  for (uint8_t ti = 0; ti < TASKS_MAX; ++ti) {
    if (!Settings.TaskDeviceEnabled[ti]) continue;
    if (Settings.TaskDeviceNumber[ti] != PLUGIN_ID_233) continue;
    if (Settings.TaskDevicePluginConfig[ti][0] == 0) continue; // nie TX
    if ((uint16_t)Settings.TaskDevicePluginConfig[ti][4] == idx) { outTi = ti; return true; }
  }
  return false;
}

// --------- RAW WiFiClient Domoticz --------
static void P233_rawDomoticzOnce(uint16_t idx, bool turnOn) {
  const uint32_t now = millis();

  if ((int32_t)(now - domoNextTryMs) < 0) { domoPendingCmd = -1; return; }
  if (domoBreakerUntil && (int32_t)(now - domoBreakerUntil) < 0) { domoPendingCmd = -1; return; }
  if (idx == 0) { domoPendingCmd = -1; return; }
  if (!g_ctrlReady || g_ctrlPort == 0) { domoPendingCmd = -1; return; }
  if (!IPv4Ready()) { return; } // keep pending

  WiFiClient client;
  client.setTimeout(75);

  bool connected = false;
#ifdef ESP32
  connected = client.connect(g_ctrlIp, g_ctrlPort, 100); // connect timeout 100ms
#else
  connected = client.connect(g_ctrlIp, g_ctrlPort);
#endif
  if (!connected) {
    domoPendingCmd = -1;
    domoFailStreak = (domoFailStreak < 250) ? (domoFailStreak + 1) : domoFailStreak;
    uint32_t pauseMs = (domoFailStreak == 1) ? 2000 : (domoFailStreak == 2 ? 5000 : 10000);
    domoBreakerUntil = millis() + pauseMs;
    domoNextTryMs    = millis() + 1000;
    client.stop();
    return;
  }

  const char* cmd = turnOn ? "On" : "Off";
  char req[320];
  int n = snprintf(req, sizeof(req),
                   "GET /json.htm?type=command&param=switchlight&idx=%u&switchcmd=%s HTTP/1.0\r\n"
                   "Host: %u.%u.%u.%u\r\n"
                   "Connection: close\r\n"
                   "\r\n",
                   (unsigned)idx, cmd,
                   (unsigned)g_ctrlIp[0], (unsigned)g_ctrlIp[1], (unsigned)g_ctrlIp[2], (unsigned)g_ctrlIp[3]);
  if (n <= 0 || (size_t)n >= sizeof(req)) {
    domoPendingCmd = -1;
    domoNextTryMs  = millis() + 1000;
    client.stop();
    return;
  }

  size_t w = client.write((const uint8_t*)req, (size_t)n);
  if (w != (size_t)n) {
    domoPendingCmd = -1;
    domoNextTryMs  = millis() + 1000;
    client.stop();
    return;
  }

  // szybki budżet czytania (10ms)
  size_t total = 0;
  const uint32_t rb = millis();
  for (;;) {
    int avail = client.available();
    if (avail > 0) {
      int toRead = (avail > 128) ? 128 : avail;
      uint8_t b[128];
      int got = client.read(b, toRead);
      if (got > 0) total += (size_t)got;
    } else if (!client.connected()) {
      break;
    }
    if ((int32_t)(millis() - rb) >= 10) break;
    delay(0);
  }

  domoFailStreak   = 0;
  domoBreakerUntil = 0;
  domoNextTryMs    = millis() + 100;
  lastDomoticzOkMs = millis();

  snprintf(p233_logbuf, sizeof(p233_logbuf), "P233 domo: GET %s bytes=%u", cmd, (unsigned)total);
  logI(p233_logbuf);

  domoPendingCmd = -1;
  client.stop();
}

// Przetwórz 1 żądanie jeśli kolejka i zaszły warunki
static void P233_domoProcessOnce() {
  if (domoBusy) return;
  if (domoPendingCmd < 0) return;
  if (domoNotBeforeMs != 0 && (int32_t)(millis() - domoNotBeforeMs) < 0) return;

  domoBusy = true;
  const bool turnOn  = (domoPendingCmd == 1);
  const uint16_t idx = domoPendingIdx;

  P233_rawDomoticzOnce(idx, turnOn);
  domoBusy = false;
}

// --------------- Main plugin ---------------
boolean Plugin_233(uint8_t function, struct EventStruct *event, String& string) {
  boolean success = false;

  switch (function) {

    case PLUGIN_DEVICE_ADD: {
      auto& dev = Device[++deviceCount];
      dev.Number      = PLUGIN_ID_233;
      dev.Type        = DEVICE_TYPE_DUMMY;
      dev.VType       = Sensor_VType::SENSOR_TYPE_SINGLE;
      dev.ValueCount  = 1;
      dev.TimerOption = false;
      success = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME: {
      string = F(PLUGIN_NAME_233);
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_LOAD: {
      addFormCheckBox(F("Send (TX)"), F("p233_send"), P233_CONFIG_SEND);
      addFormPinSelect(PinSelectPurpose::Generic, F("GPIO Pin"), F("p233_gpio"), P233_CONFIG_GPIO);

      if (!P233_CONFIG_SEND) {
        addFormNumericBox(F("Device ID (receiver)"), F("p233_devid"), P233_CONFIG_DEVICE_ID, 0, 255);
        addFormCheckBox(F("Invert output (RX)"), F("p233_invert_rx"), P233_CONFIG_INVERT_RX);
      } else {
        addFormNumericBox(F("Target DeviceID (CAN)"), F("p233_targetid"), P233_CONFIG_TARGET_ID, 0, 255);
        addFormNumericBox(F("Domoticz idx (0=off)"), F("p233_domo_idx"), P233_CONFIG_DOMO_IDX, 0, 65535);
        // (bez opcji invert TX)
      }
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE: {
      PCONFIG(0) = isFormItemChecked(F("p233_send"));
      PCONFIG(1) = getFormItemInt(F("p233_gpio"));
      if (!P233_CONFIG_SEND) {
        PCONFIG(2) = getFormItemInt(F("p233_devid"));
        PCONFIG(6) = isFormItemChecked(F("p233_invert_rx"));
      } else {
        PCONFIG(3) = getFormItemInt(F("p233_targetid"));
        PCONFIG(4) = getFormItemInt(F("p233_domo_idx"));
        // PCONFIG(5) – nieużywany
      }
      success = true;
      break;
    }

    case PLUGIN_INIT: {
      const uint8_t ti = event->TaskIndex;

      // echo guard init
      domoEchoGuardWptr[ti] = 0;
      for (uint8_t i=0;i<ECHO_GUARD_SLOTS;++i){
        domoEchoGuardState[ti][i] = -1;
        domoEchoGuardUntil[ti][i] = 0;
      }

      // CAN init (raz)
      if (!canInitialized) {
        SPI.begin();
        settings.mRequestedMode = ACAN2515Settings::NormalMode;
        const uint16_t ec = can.begin(settings, canISR);
        if (ec != 0) {
          snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: CAN init FAIL err=%u", (unsigned)ec);
          logE(p233_logbuf);
          success = true;
          break;
        }
        canInitialized = true;
        logI("P233: CAN initialized");
      }

      // RX master
      if (rxMasterTask == 255 || ti < rxMasterTask) {
        rxMasterTask = ti;
        snprintf(p233_logbuf, sizeof(p233_logbuf), "P233: RX master task=%u", (unsigned)rxMasterTask);
        logI(p233_logbuf);
      }

      // GPIO direction
      // ZMIANA: dla TX używamy INPUT_PULLDOWN (domyślnie niski), RX pozostaje OUTPUT
      pinMode(P233_CONFIG_GPIO, P233_CONFIG_SEND ? INPUT_PULLDOWN : OUTPUT);

      // NVS state per-task
      prefs.begin("plugin233", false);
      String key = String(F("lastState_")) + String(ti);
      if (prefs.isKey(key.c_str())) {
        toggleStateArr[ti] = prefs.getUChar(key.c_str(), HIGH);
      } else {
        toggleStateArr[ti] = HIGH;
        prefs.putUChar(key.c_str(), (uint8_t)toggleStateArr[ti]);
      }
      prefs.end();

      if (P233_CONFIG_SEND) {
        // Controller #0 config only
        String h; IPAddress ip; uint16_t port; bool haveHost;
        ReadController0ConfigOnly(h, ip, port, haveHost);
        g_ctrlIp       = ip;
        g_ctrlPort     = port;
        g_ctrlHost     = h;
        g_ctrlHaveHost = haveHost;
        g_ctrlReady    = (g_ctrlPort != 0) && (g_ctrlIp != IPAddress(0,0,0,0));
        g_dnsNextTryMs = millis() + 3000;

        lastGpioState[ti] = digitalRead(P233_CONFIG_GPIO);

        // CAN: stan startowy do targetu
        const uint8_t senderId = (uint8_t)Settings.Unit;
        CANMessage msg;
        msg.id      = 0x123;
        msg.len     = 3;
        msg.data[0] = (uint8_t)toggleStateArr[ti];
        msg.data[1] = (uint8_t)P233_CONFIG_TARGET_ID;
        msg.data[2] = senderId;
        bool ok = can.tryToSend(msg);
        snprintf(p233_logbuf, sizeof(p233_logbuf),
                 "P233 TX init: state=%s target=%u sender=%u send=%s",
                 toggleStateArr[ti] ? "HIGH":"LOW", (unsigned)P233_CONFIG_TARGET_ID, (unsigned)senderId, ok?"OK":"FAIL");
        (ok ? logI : logE)(p233_logbuf);

        // Lokalny loopback (sam-do-siebie)
        p233_localLoop_applyIfLocal((uint8_t)P233_CONFIG_TARGET_ID, (uint8_t)toggleStateArr[ti], senderId);

        // Domoticz: jeśli idx>0, zakolejkuj po 5 s + echo-guard
        if (P233_CONFIG_DOMO_IDX > 0) {
          domoPendingIdx  = (uint16_t)P233_CONFIG_DOMO_IDX;
          domoPendingCmd  = (toggleStateArr[ti] == HIGH) ? 1 : 0;
          domoNotBeforeMs = millis() + 5000;
          echo_guard_push(ti, (int8_t)((toggleStateArr[ti]==HIGH)?1:0));
        }
      } else {
        // RX: ustaw stan początkowy z ewentualną inwersją
        int outVal = toggleStateArr[ti];
        if (P233_CONFIG_INVERT_RX) outVal = (outVal==HIGH)?LOW:HIGH;

        lastGpioState[ti] = outVal;
        digitalWrite(P233_CONFIG_GPIO, outVal);

        // RX: HELLO po starcie
        CANMessage hello;
        hello.id  = 0x125;
        hello.len = 1;
        hello.data[0] = (uint8_t)P233_CONFIG_DEVICE_ID;
        (void)can.tryToSend(hello);

        snprintf(p233_logbuf, sizeof(p233_logbuf),
                 "P233 RX init: GPIO=%u set=%s (logical=%s, invertRX=%s)",
                 (unsigned)P233_CONFIG_GPIO,
                 (outVal==HIGH)?"HIGH":"LOW",
                 (toggleStateArr[ti]==HIGH)?"HIGH":"LOW",
                 P233_CONFIG_INVERT_RX?"ON":"OFF");
        logI(p233_logbuf);
      }

      success = true;
      break;
    }

    case PLUGIN_TEN_PER_SECOND: {
      const uint8_t ti = event->TaskIndex;
      if (!canInitialized) { success = true; break; }

      // TX: debounce + toggle + CAN + enqueue Domoticz (raw)
      if (P233_CONFIG_SEND) {
        if (!g_ctrlReady && g_ctrlHaveHost) {
          TryResolveControllerHostIfNeeded();
        }

        const int raw = digitalRead(P233_CONFIG_GPIO);
        const unsigned long now = millis();

        // ZMIANA: wykrywanie ZBOCZA NARASTAJĄCEGO w surowym odczycie
        if (lastGpioState[ti] == LOW && raw == HIGH) {
          if (now - lastDebounceTimeArr[ti] > debounceDelay) {
            lastDebounceTimeArr[ti] = now;

            // toggle state
            toggleStateArr[ti] = (toggleStateArr[ti] == HIGH) ? LOW : HIGH;
            P233_updateValue(ti, toggleStateArr[ti]);


            // save NVS
            prefs.begin("plugin233", false);
            String key2 = String(F("lastState_")) + String(ti);
            prefs.putUChar(key2.c_str(), (uint8_t)toggleStateArr[ti]);
            prefs.end();

            // CAN notify
            const uint8_t senderId = (uint8_t)Settings.Unit;
            const uint8_t targetId = (uint8_t)P233_CONFIG_TARGET_ID;
            CANMessage msg;
            msg.id      = 0x123;
            msg.len     = 3;
            msg.data[0] = (uint8_t)toggleStateArr[ti];
            msg.data[1] = (uint8_t)targetId;
            msg.data[2] = senderId;
            (void)can.tryToSend(msg);

            // Lokalny loopback (sam-do-siebie)
            p233_localLoop_applyIfLocal(targetId, (uint8_t)toggleStateArr[ti], senderId);

            lastCanEdgeMs = now;

            // enqueue Domoticz + echo guard
            if (P233_CONFIG_DOMO_IDX > 0) {
              domoPendingIdx = (uint16_t)P233_CONFIG_DOMO_IDX;
              domoPendingCmd = (toggleStateArr[ti] == HIGH) ? 1 : 0;
              echo_guard_push(ti, (int8_t)((toggleStateArr[ti]==HIGH)?1:0));
            }
          }
        }
        // zapamiętaj surowy stan pinu
        lastGpioState[ti] = raw;
      }

      // RX master: drain CAN
      if (ti == rxMasterTask) {
        CANMessage rx;
        while (can.receive(rx)) {
          if (rx.id == 0x123 && rx.len >= 2) {
            const uint8_t stateBit    = rx.data[0];
            const uint8_t targetDevId = rx.data[1];
            const uint8_t senderId    = (rx.len >= 3) ? rx.data[2] : 255;
            p233_dispatchRx_0x123(stateBit, targetDevId, senderId);
          } else if (rx.id == 0x125 && rx.len >= 1) {
            p233_dispatchRx_0x125(rx.data[0]);
          }
        }
      }

      // Domoticz: JEDNA próba jeśli jest w kolejce.
      if (domoPendingCmd >= 0 && !domoBusy) {
        const uint32_t now2 = millis();
        bool fastPath = (domoFailStreak == 0) && ((int32_t)(now2 - lastDomoticzOkMs) < 10000);

        if (fastPath || (int32_t)(now2 - lastCanEdgeMs) >= 150) {
          P233_domoProcessOnce();
        }
      }

      success = true;
      break;
    }

    case PLUGIN_WRITE: {
      // /control?cmd=p233,idx=NUM,on|off|toggle
      String s = string; s.trim();
      int c1 = s.indexOf(',');
      String cmd = (c1 >= 0) ? s.substring(0, c1) : s;
      cmd.trim(); cmd.toLowerCase();

      if (cmd == F("p233")) {
        String rest = (c1 >= 0) ? s.substring(c1 + 1) : F("");
        int c2 = rest.indexOf(',');
        String sel = (c2 >= 0) ? rest.substring(0, c2) : rest;
        String act = (c2 >= 0) ? rest.substring(c2 + 1) : F("toggle");
        sel.trim(); act.trim(); act.toLowerCase();

        uint8_t ti = 0xFF; bool resolved = false;
        if (sel.startsWith(F("idx="))) {
          uint16_t idx = (uint16_t) sel.substring(4).toInt();
          resolved = P233_findTaskByIdx(idx, ti);
        }

        if (resolved) {
          int mode = -1; 
          if (act == F("on"))  mode = 1;
          if (act == F("off")) mode = 0;

          // Echo-guard x2 (supresja odbitego HTTP z Domoticza)
          uint8_t hit=255;
          if (echo_guard_match_and_consume(ti, (int8_t)(mode==1?1:0), hit)) {
            snprintf(p233_logbuf, sizeof(p233_logbuf),
                     "P233 WRITE: echo-suppressed[slot=%u] ti=%u act=%s",
                     (unsigned)hit, (unsigned)ti, act.c_str());
            logI(p233_logbuf);
            success = true;
            break;
          }

          p233_tx_apply(ti, mode);
          lastCanEdgeMs = millis();
          success = true;
        }
      }
      break;
    }

    default: break;
  }

  return success;
}

#endif // USES_P233
