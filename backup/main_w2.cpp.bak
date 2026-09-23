#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ============================================================
// KHAI BAO CHAN GPIO
// ============================================================

#define RC522_SS_PIN   5
#define RC522_RST_PIN  21
#define REED_PIN       33
#define RELAY_PIN      27
#define SERVO_PIN      26
#define BUZZER_PIN     25

// ============================================================
// CAU HINH BUZZER
// Khong dung tone() de tranh xung dot PWM/LEDC voi Servo
// ============================================================



// ============================================================
// DOI TUONG
// ============================================================

MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);
Servo lockServo;

// UID hop le: 01 02 03 04
byte validUID[4] = {01, 02, 03, 04};

// Danh dau cua dang duoc mo boi the hop le
bool doorWasUnlockedRecently = false;

// Luu trang thai cua o vong lap truoc
bool lastDoorOpen = false;

// ============================================================
// KHAI BAO HAM
// ============================================================

bool checkValidUID(byte *uid, byte size);
void unlockDoor();
void beepBuzzer();

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // Khoi dong SPI + RFID
  SPI.begin();
  rfid.PCD_Init();

  // ----------------------------------------------------------
  // Cau hinh GPIO
  // ----------------------------------------------------------

  pinMode(REED_PIN, INPUT_PULLUP);
  // ----------------------------------------------------------
  // Relay
  // ----------------------------------------------------------
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // ----------------------------------------------------------
  // Cau hinh Buzzer bang LEDC
  // Khong dung tone()
  // ----------------------------------------------------------
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // ----------------------------------------------------------
  // Cau hinh Servo
  // ----------------------------------------------------------

  lockServo.attach(SERVO_PIN);
  // 0 do = khoa
  lockServo.write(0);

  Serial.println("=== Tuan 2: Simulation Tang 1 - San sang ===");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // ==========================================================
  // PHAN 1: DOC RFID
  // ==========================================================

  if (rfid.PICC_IsNewCardPresent() &&
      rfid.PICC_ReadCardSerial()) {

    Serial.print("UID doc duoc:");

    for (byte i = 0; i < rfid.uid.size; i++) {

      Serial.print(
        rfid.uid.uidByte[i] < 0x10 ? " 0" : " "
      );

      Serial.print(rfid.uid.uidByte[i], HEX);
    }

    Serial.println();

    // --------------------------------------------------------
    // Kiem tra UID
    // --------------------------------------------------------

    if (checkValidUID(
          rfid.uid.uidByte,
          rfid.uid.size)) {

      Serial.println(">> The HOP LE -> MO KHOA");

      unlockDoor();

    } else {

      Serial.println(">> The KHONG hop le -> Tu choi");
    }

    // Bao RC522 da xu ly xong the
    rfid.PICC_HaltA();
  }

  // ==========================================================
  // PHAN 2: DOC TRANG THAI CUA
  // ==========================================================

  bool doorOpen = (digitalRead(REED_PIN) == LOW);

  // ----------------------------------------------------------
  // Chi phat hien canh bao khi cua VUA CHUYEN TU DONG -> MO
  // ----------------------------------------------------------

  if (doorOpen &&
      !doorWasUnlockedRecently) {

    Serial.println("!!! CANH BAO: Cua mo BAT THUONG !!!");

    beepBuzzer();
  }
  delay(200);

  // Cap nhat trang thai cua cho vong lap tiep theo
  lastDoorOpen = doorOpen;

  delay(200);
}

// ============================================================
// KIEM TRA UID
// ============================================================

bool checkValidUID(byte *uid, byte size) {

  // Chi chap nhan UID 4 byte
  if (size != 4)
    return false;

  for (byte i = 0; i < 4; i++) {

    if (uid[i] != validUID[i])
      return false;
  }

  return true;
}

// ============================================================
// BUZZER
// ============================================================

void beepBuzzer() {

  // Tao am thanh 1000 Hz bang cach bat/tat GPIO thu cong
  // Chu ky 1000 Hz = 1000 micro giay
  // Moi nua chu ky = 500 micro giay

  unsigned long startTime = millis();

  while (millis() - startTime < 500) {

    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(500);

    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(500);
  }

  // Dam bao buzzer tat
  digitalWrite(BUZZER_PIN, LOW);
}

// ============================================================
// MO KHOA
// ============================================================

void unlockDoor() {

  // ----------------------------------------------------------
  // Relay bat
  // ----------------------------------------------------------

  digitalWrite(RELAY_PIN, HIGH);

  // ----------------------------------------------------------
  // Servo mo cua
  // ----------------------------------------------------------

  lockServo.write(90);

  // Danh dau cua vua duoc mo bang the hop le
  doorWasUnlockedRecently = true;

  Serial.println(">> Relay ON");
  Serial.println(">> Servo -> 90 do");

  // ----------------------------------------------------------
  // Giu cua mo 3 giay
  // ----------------------------------------------------------

  delay(3000);

  // ----------------------------------------------------------
  // Dong cua
  // ----------------------------------------------------------

  lockServo.write(0);

  digitalWrite(RELAY_PIN, LOW);

  doorWasUnlockedRecently = false;

  Serial.println(">> Servo -> 0 do");
  Serial.println(">> Relay OFF");
}

