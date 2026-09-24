// ============================================================
// DE TAI 8 - KHOA CUA RFID | TUAN 3 - FIRMWARE V2.0
// Non-blocking state machine (millis), debounce, whitelist, lockout,
// event log co cau truc, cong nhan lenh (Serial hien tai -> MQTT tuan 4)
// ============================================================
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <strings.h>

// ============================================================
// KHOI A1 - PHAN CUNG (giu nguyen tu Tuan 2)
// ============================================================
constexpr uint8_t RC522_SS_PIN  = 5;
constexpr uint8_t RC522_RST_PIN = 21;
constexpr uint8_t REED_PIN      = 33;
constexpr uint8_t RELAY_PIN     = 27;
constexpr uint8_t SERVO_PIN     = 26;
constexpr uint8_t BUZZER_PIN    = 25;

// Muc logic (doi 1 cho neu diagram.json/phan cung thay doi)
constexpr uint8_t RELAY_ON_LEVEL  = HIGH;  // muc IN lam relay "bat" (xem muc kiem tra relay)
constexpr uint8_t DOOR_OPEN_LEVEL = LOW;   // slide switch: LOW = cua mo
constexpr int SERVO_LOCKED_DEG    = 0;
constexpr int SERVO_UNLOCKED_DEG  = 90;

// ============================================================
// KHOI A2 - THONG SO THRESHOLD / TIMER (gia tri khoi diem - se tinh chinh khi test)
// ============================================================
namespace Cfg {
  constexpr unsigned long DOOR_DEBOUNCE_MS   = 50;     // loc nhieu switch cua
  constexpr unsigned long RFID_POLL_MS       = 100;    // chu ky hoi RC522
  constexpr unsigned long CARD_COOLDOWN_MS   = 1500;   // bo qua doc lap lai cung 1 the
  constexpr unsigned long UNLOCK_WINDOW_MS   = 5000;   // thoi gian cho mo cua sau khi mo khoa
  constexpr unsigned long RELOCK_DELAY_MS    = 1500;   // cua dong on dinh bao lau thi khoa lai
  constexpr unsigned long LEFT_OPEN_MS       = 15000;  // cua mo hop le qua lau -> nhac nho
  constexpr unsigned long ALARM_MAX_SOUND_MS = 10000;  // toi da thoi gian coi bao dong keu
  constexpr uint8_t       FAIL_LIMIT         = 3;      // so lan quet sai lien tiep de khoa tam
  constexpr unsigned long FAIL_WINDOW_MS     = 30000;  // qua ngan nay khong sai -> dem lai tu 0
  constexpr unsigned long LOCKOUT_MS         = 10000;  // thoi gian khoa tam RFID
  constexpr bool          ALARM_LATCHED      = false;  // true: bao dong giu den khi the/lenh hop le
}

// ============================================================
// KHOI A3 - DANH SACH THE HOP LE (whitelist)
// ============================================================
struct UidEntry { byte bytes[4]; const char* name; };
const UidEntry WHITELIST[] = {
  { {0x01, 0x02, 0x03, 0x04}, "blue"  },   // Blue Card preset cua Wokwi
  { {0x11, 0x22, 0x33, 0x44}, "green" },   // Green Card preset cua Wokwi
};
constexpr size_t WHITELIST_SIZE = sizeof(WHITELIST) / sizeof(WHITELIST[0]);

// ============================================================
// KHOI A4 - KIEU DU LIEU + BIEN TRANG THAI
// ============================================================
enum class LockState : uint8_t { LOCKED, UNLOCKED_WAIT_OPEN, DOOR_OPEN_AUTH, ALARM };
enum class Source    : uint8_t { RFID, SERIAL_TEST, REMOTE };

struct BeepPattern { uint16_t onMs; uint16_t offMs; uint8_t repeat; };  // repeat 0 = vo han
constexpr BeepPattern PAT_OK       {120,    0, 1};
constexpr BeepPattern PAT_DENIED   {120,  120, 2};
constexpr BeepPattern PAT_LOCKOUT  {600,    0, 1};
constexpr BeepPattern PAT_LEFTOPEN {100, 1900, 0};
constexpr BeepPattern PAT_ALARM    {300,  200, 0};

MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);
Servo lockServo;

LockState state = LockState::LOCKED;
unsigned long stateSince = 0;

bool doorRaw = false, doorStable = false;          // false = dong, true = mo
unsigned long doorRawChangedAt = 0, doorStableSince = 0;

uint8_t failCount = 0;
unsigned long lastFailAt = 0;
bool lockoutActive = false;
unsigned long lockoutStart = 0;

byte lastUid[10];
byte lastUidSize = 0;
unsigned long lastCardAt = 0;
bool haveLastCard = false;
unsigned long lastRfidPoll = 0;

bool alarmMuted = false;
bool leftOpenWarned = false;

const BeepPattern* bzPattern = nullptr;
uint8_t bzCount = 0;
bool bzPhaseOn = false, bzLevel = false;
unsigned long bzPhaseStart = 0, bzLastToggleUs = 0;

unsigned long loopMaxUs = 0;

// ============================================================
// KHOI B - TIEN ICH: thoi gian, log su kien, UID
// ============================================================
// An toan khi millis() tran so (~50 ngay): luon tru, khong cong
inline bool elapsed(unsigned long now, unsigned long since, unsigned long ms) {
  return (now - since) >= ms;
}

const char* stateName(LockState s) {
  switch (s) {
    case LockState::LOCKED:             return "LOCKED";
    case LockState::UNLOCKED_WAIT_OPEN: return "UNLOCKED_WAIT_OPEN";
    case LockState::DOOR_OPEN_AUTH:     return "DOOR_OPEN_AUTH";
    case LockState::ALARM:              return "ALARM";
  }
  return "?";
}

// DIEM DUY NHAT phat su kien. Tuan 4: them 1 dong publish MQTT o day.
void logEvent(const char* evt, const char* detail = "") {
  Serial.printf("[%lu] EVT=%s state=%s %s\r\n", millis(), evt, stateName(state), detail);
}

void uidToString(const byte* uid, byte size, char* out, size_t outLen) {
  size_t pos = 0;
  out[0] = '\0';
  for (byte i = 0; i < size && pos + 2 < outLen; i++) {
    pos += snprintf(out + pos, outLen - pos, "%02X", uid[i]);
  }
}

const UidEntry* findUid(const byte* uid, byte size) {
  if (size != 4) return nullptr;
  for (size_t i = 0; i < WHITELIST_SIZE; i++) {
    if (memcmp(uid, WHITELIST[i].bytes, 4) == 0) return &WHITELIST[i];
  }
  return nullptr;
}

// ============================================================
// KHOI C - BUZZER KHONG CHAN (mau beep, khong dung delay)
// ============================================================
void buzzerStop() {
  bzPattern = nullptr;
  bzLevel = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerPlay(const BeepPattern& p) {
  bzPattern = &p;
  bzCount = 0;
  bzPhaseOn = true;
  bzLevel = false;
  bzPhaseStart = millis();
  bzLastToggleUs = micros();
}

void buzzerUpdate() {
  if (bzPattern == nullptr) return;
  unsigned long nowMs = millis();
  unsigned long phaseLen = bzPhaseOn ? bzPattern->onMs : bzPattern->offMs;

  if (elapsed(nowMs, bzPhaseStart, phaseLen)) {
    bzPhaseStart = nowMs;
    if (bzPhaseOn) {
      bzPhaseOn = false;
      bzLevel = false;
      digitalWrite(BUZZER_PIN, LOW);
    } else {
      bzCount++;
      if (bzPattern->repeat != 0 && bzCount >= bzPattern->repeat) {
        buzzerStop();
        return;
      }
      bzPhaseOn = true;
    }
    return;
  }

  if (bzPhaseOn) {                               // song vuong ~1 kHz bang micros()
    unsigned long nowUs = micros();
    if (nowUs - bzLastToggleUs >= 500) {
      bzLastToggleUs = nowUs;
      bzLevel = !bzLevel;
      digitalWrite(BUZZER_PIN, bzLevel ? HIGH : LOW);
    }
  }
}

// ============================================================
// KHOI D - ACTUATOR + CUA (debounce)
// ============================================================
void setLock(bool unlocked) {
  digitalWrite(RELAY_PIN, unlocked ? RELAY_ON_LEVEL : (RELAY_ON_LEVEL == HIGH ? LOW : HIGH));
  lockServo.write(unlocked ? SERVO_UNLOCKED_DEG : SERVO_LOCKED_DEG);
}

// Tra ve true dung 1 lan khi trang thai cua ON DINH doi (da qua debounce)
bool doorUpdate(unsigned long now) {
  bool raw = (digitalRead(REED_PIN) == DOOR_OPEN_LEVEL);
  if (raw != doorRaw) {
    doorRaw = raw;
    doorRawChangedAt = now;
  }
  if (doorRaw != doorStable && elapsed(now, doorRawChangedAt, Cfg::DOOR_DEBOUNCE_MS)) {
    doorStable = doorRaw;
    doorStableSince = now;
    return true;
  }
  return false;
}

// ============================================================
// KHOI E - STATE MACHINE
// ============================================================
void enterState(LockState s, unsigned long now) {
  Serial.printf("[%lu] STATE %s -> %s\r\n", now, stateName(state), stateName(s));
  state = s;
  stateSince = now;
  switch (s) {
    case LockState::LOCKED:             setLock(false); buzzerStop(); break;
    case LockState::UNLOCKED_WAIT_OPEN: setLock(true);  buzzerStop(); break;
    case LockState::DOOR_OPEN_AUTH:     setLock(true);  buzzerStop(); leftOpenWarned = false; break;
    case LockState::ALARM:              setLock(false); alarmMuted = false; buzzerPlay(PAT_ALARM); break;
  }
}

// CONG VAO DUY NHAT de mo khoa: RFID, Serial (Tuan 3), MQTT/Telegram/Voice (Tuan 4+)
void requestUnlock(Source src, const char* detail) {
  unsigned long now = millis();
  const char* srcName = (src == Source::RFID) ? "RFID" : (src == Source::SERIAL_TEST ? "SERIAL" : "REMOTE");
  char msg[80];
  snprintf(msg, sizeof(msg), "src=%s %s", srcName, detail);
  if (src == Source::RFID) failCount = 0;
  enterState(doorStable ? LockState::DOOR_OPEN_AUTH : LockState::UNLOCKED_WAIT_OPEN, now);
  buzzerPlay(PAT_OK);
  logEvent("UNLOCK_GRANTED", msg);
}

void handleCard(const byte* uid, byte size, unsigned long now) {
  bool repeat = haveLastCard && size == lastUidSize && memcmp(uid, lastUid, size) == 0 &&
                !elapsed(now, lastCardAt, Cfg::CARD_COOLDOWN_MS);
  lastCardAt = now;                       // the con nam tren dau doc -> keo dai cooldown
  if (repeat) return;

  if (size > sizeof(lastUid)) size = sizeof(lastUid);
  memcpy(lastUid, uid, size);
  lastUidSize = size;
  haveLastCard = true;

  char uidStr[24];
  uidToString(uid, size, uidStr, sizeof(uidStr));
  char msg[64];

  if (lockoutActive) {
    snprintf(msg, sizeof(msg), "uid=%s", uidStr);
    logEvent("CARD_IGNORED_LOCKOUT", msg);
    return;
  }

  const UidEntry* e = findUid(uid, size);
  if (e != nullptr) {
    snprintf(msg, sizeof(msg), "uid=%s name=%s", uidStr, e->name);
    requestUnlock(Source::RFID, msg);
    return;
  }

  if (elapsed(now, lastFailAt, Cfg::FAIL_WINDOW_MS)) failCount = 0;
  failCount++;
  lastFailAt = now;
  snprintf(msg, sizeof(msg), "uid=%s fail=%u/%u", uidStr, failCount, Cfg::FAIL_LIMIT);
  logEvent("CARD_DENIED", msg);
  if (state != LockState::ALARM) buzzerPlay(PAT_DENIED);

  if (failCount >= Cfg::FAIL_LIMIT) {
    lockoutActive = true;
    lockoutStart = now;
    failCount = 0;
    logEvent("LOCKOUT_START", "too many invalid cards");
    if (state != LockState::ALARM) buzzerPlay(PAT_LOCKOUT);
  }
}

void lockoutUpdate(unsigned long now) {
  if (lockoutActive && elapsed(now, lockoutStart, Cfg::LOCKOUT_MS)) {
    lockoutActive = false;
    logEvent("LOCKOUT_END");
  }
}

void fsmUpdate(unsigned long now, bool doorChanged) {
  switch (state) {

    case LockState::LOCKED:
      if (doorChanged && doorStable) {                    // cua mo khi chua duoc phep
        enterState(LockState::ALARM, now);
        logEvent("DOOR_FORCED", "door opened without authorization");
      }
      break;

    case LockState::UNLOCKED_WAIT_OPEN:
      if (doorChanged && doorStable) {
        enterState(LockState::DOOR_OPEN_AUTH, now);
        logEvent("DOOR_OPENED", "authorized");
      } else if (!doorStable && elapsed(now, stateSince, Cfg::UNLOCK_WINDOW_MS)) {
        enterState(LockState::LOCKED, now);
        logEvent("RELOCKED", "timeout no entry");
      }
      break;

    case LockState::DOOR_OPEN_AUTH:
      if (doorChanged && doorStable) {                    // mo lai trong luc chua khoa
        stateSince = now;
        leftOpenWarned = false;
        buzzerStop();
        logEvent("DOOR_REOPENED");
      }
      if (doorChanged && !doorStable) {
        buzzerStop();
        leftOpenWarned = false;
        logEvent("DOOR_CLOSED");
      }
      if (!doorStable && elapsed(now, doorStableSince, Cfg::RELOCK_DELAY_MS)) {
        enterState(LockState::LOCKED, now);
        logEvent("RELOCKED", "door closed");
      } else if (doorStable && !leftOpenWarned && elapsed(now, stateSince, Cfg::LEFT_OPEN_MS)) {
        leftOpenWarned = true;
        buzzerPlay(PAT_LEFTOPEN);
        logEvent("LEFT_OPEN_WARN", "door open too long");
      }
      break;

    case LockState::ALARM:
      if (doorChanged && !doorStable) logEvent("DOOR_CLOSED");
      if (!alarmMuted && elapsed(now, stateSince, Cfg::ALARM_MAX_SOUND_MS)) {
        buzzerStop();
        alarmMuted = true;
        logEvent("ALARM_MUTED", "max sound time reached");
      }
      if (!Cfg::ALARM_LATCHED && !doorStable && elapsed(now, doorStableSince, Cfg::RELOCK_DELAY_MS)) {
        enterState(LockState::LOCKED, now);
        logEvent("ALARM_CLEARED", "door closed");
      }
      break;
  }
}

// ============================================================
// KHOI F - RFID + LENH (Serial hien tai; Tuan 4 goi handleCommand tu MQTT callback)
// ============================================================
void rfidPoll(unsigned long now) {
  if (!elapsed(now, lastRfidPoll, Cfg::RFID_POLL_MS)) return;
  lastRfidPoll = now;
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;
  handleCard(rfid.uid.uidByte, rfid.uid.size, now);
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void printStatus() {
  Serial.printf("[%lu] STATUS state=%s door=%s lockout=%s fail=%u loopMaxUs=%lu\r\n",
                millis(), stateName(state), doorStable ? "OPEN" : "CLOSED",
                lockoutActive ? "YES" : "NO", failCount, loopMaxUs);
}

void handleCommand(const char* cmd, Source src) {
  if (strcasecmp(cmd, "unlock") == 0)         requestUnlock(src, "cmd=unlock");
  else if (strcasecmp(cmd, "status") == 0)    printStatus();
  else if (strcasecmp(cmd, "resetstat") == 0) loopMaxUs = 0;
  else                                        logEvent("CMD_UNKNOWN", cmd);
}

void serialPoll() {
  static char buf[24];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf, Source::SERIAL_TEST);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ============================================================
// SETUP + LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  SPI.begin();
  rfid.PCD_Init();

  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  lockServo.attach(SERVO_PIN);

  unsigned long now = millis();
  doorRaw = doorStable = (digitalRead(REED_PIN) == DOOR_OPEN_LEVEL);
  doorRawChangedAt = doorStableSince = now;

  state = LockState::LOCKED;
  stateSince = now;
  setLock(false);

  Serial.println("=== FW v2.0 (Tuan 3): state machine khong chan ===");
  if (doorStable) logEvent("BOOT_DOOR_OPEN", "door open at boot - no alarm until next open");
  printStatus();
}

void loop() {
  unsigned long t0 = micros();
  unsigned long now = millis();

  bool doorChanged = doorUpdate(now);
  fsmUpdate(now, doorChanged);
  rfidPoll(now);
  serialPoll();
  lockoutUpdate(now);
  buzzerUpdate();

  unsigned long dur = micros() - t0;
  if (dur > loopMaxUs) loopMaxUs = dur;
}