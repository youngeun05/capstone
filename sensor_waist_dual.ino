// ═══════════════════════════════════════════════════════════════
//  허리(골반+상부) 센서 펌웨어 - ESP32 보드 1개 + IMU 2개 (6축 x2)
// ═══════════════════════════════════════════════════════════════
//  보드 1개에 MPU 2개를 같은 I2C 버스에 물리고, AD0 핀으로 주소를 나눠
//  각각 다른 센서 ID 로 서버에 보낸다.
//
//    WAIST_LOW : 아래쪽 = 천골/L5 부근   -> 고관절 계산에 쓰인다
//    WAIST_UP  : 위쪽   = 허리 상부/흉요추 -> 지금은 기록만 (오프라인 분석용)
//
//  다리 보드는 sensor_leg_single.ino 를 쓴다 (MPU 1개). 이 스케치는
//  허리 보드 전용이다.
//
//  ⚠ 업로드 전 체크리스트
//    1) ADDR_LOW / ADDR_UP 확인.  <- 가장 중요. 아래 '주소 확인법' 참고
//    2) WIFI_SSID / WS_HOST 확인 (ipconfig 로 서버 PC IP 대조)
//    3) 오프셋 측정: 두 센서 각각 따로 재서 넣을 것 (복사 금지)
//
//  ── 주소 확인법 ──────────────────────────────────────────────
//  MPU6050 의 AD0 핀이 GND(또는 미연결)면 0x68, 3.3V 면 0x69 다.
//  배선을 모르면 그냥 업로드하고 시리얼 모니터(115200)를 열면 된다.
//  부팅할 때 I2C 버스를 스캔해서 응답한 주소를 전부 찍어준다.
//
//  그다음 '어느 쪽이 아래인지' 는 이렇게 확인한다:
//    1) 보드를 켜고 시리얼 모니터를 본다 (1초마다 두 센서 값이 찍힌다)
//    2) 아래쪽 센서만 손으로 살짝 기울인다
//    3) WAIST_LOW 줄의 숫자가 변하면 정상. WAIST_UP 이 변하면 배선이
//       반대이므로 아래 ADDR_LOW / ADDR_UP 값을 서로 바꿔서 다시 업로드.
//  ─────────────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define BOOT_BTN 0

// ⭐⭐ 두 센서의 I2C 주소 ⭐⭐
//   부팅 시 스캔 결과를 보고 맞는지 확인할 것. 반대면 두 값을 바꾸면 된다.
#define ADDR_LOW 0x68      // 아래쪽(천골/L5) = WAIST_LOW  -> 고관절 계산용
#define ADDR_UP  0x69      // 위쪽(허리 상부) = WAIST_UP   -> 기록만

// ⭐ 와이파이 / 서버 정보 (다리 보드와 동일하게)
const char* WIFI_SSID = "IT-301_MOB_2.4G";
const char* WIFI_PASS = "mobile2019";
const char* WS_HOST   = "172.20.103.181";   // 서버 PC IP (ipconfig 로 확인!)
const uint16_t WS_PORT = 8765;

// ── 필터 설정 (다리 펌웨어와 동일) ──
const float BETA_NORMAL = 0.25;
const float BETA_STATIC = 0.5;
const float GYRO_STATIC_THR = 8.0;
const float BIAS_ALPHA    = 0.0008;
const float BIAS_GYRO_THR = 2.5;
const float BIAS_ACC_TOL  = 0.10;
const float ACC_DEAD_ZONE = 0.03;
const float ACC_REJECT_K  = 4.0;

// ── 센서 1개분 상태 전체 ──
//   다리 펌웨어는 전역 변수 하나로 처리했지만, 여기선 센서가 2개라
//   오프셋·바이어스·accRef 까지 전부 센서별로 따로 가져가야 한다.
//   (하나로 공유하면 두 센서의 바이어스 보정이 서로를 망가뜨린다)
struct Sensor {
  uint8_t     addr;
  const char* id;
  bool        present;      // 부팅 때 응답했나
  float qw, qx, qy, qz;
  float accRef;
  int16_t axOff, ayOff, azOff;   // 가속도 오프셋 (mpu_calib 으로 실측)
  float   gxOff, gyOff, gzOff;   // 자이로 오프셋 (실행 중 자동 보정됨)
};

Sensor sensors[2] = {
  // addr,     id,          present, qw,  qx,  qy,  qz,  accRef, ax,ay,az, gx,  gy,  gz
  { ADDR_LOW, "WAIST_LOW", false,   1.0, 0.0, 0.0, 0.0, 0.0,    0, 0, 0,  0.0, 0.0, 0.0 },
  { ADDR_UP,  "WAIST_UP",  false,   1.0, 0.0, 0.0, 0.0, 0.0,    0, 0, 0,  0.0, 0.0, 0.0 },
};
const int N_SENSORS = 2;

unsigned long lastTime = 0;
unsigned long lastBtnTime = 0;
unsigned long lastFilterTime = 0;
unsigned long lastMpuRetry = 0;

WebSocketsClient webSocket;

// ── 연결 진단용 상태 ──
// 예전엔 접속 실패도 "연결 끊김" 으로만 찍혀서, '한 번도 못 붙은 것'인지
// '붙었다가 끊긴 것'인지 구분이 안 됐다. 원인이 완전히 다르므로 나눠서 찍는다.
bool wsEverConnected = false;
unsigned long wsFailCount = 0;
unsigned long wsConnectedAt = 0;

void printNetHelp() {
  Serial.println("   ┌─ 확인할 것 ────────────────────────────────");
  Serial.printf("   │ 1) 이 보드 IP    : %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("   │    게이트웨이     : %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.printf("   │ 2) 서버 주소 설정 : %s:%u\n", WS_HOST, WS_PORT);
  Serial.println("   │    -> 위 두 IP 의 앞 세 자리가 같아야 합니다.");
  Serial.println("   │       다르면 PC 를 보드와 같은 와이파이에 연결하거나");
  Serial.println("   │       WS_HOST 를 PC 의 ipconfig 값으로 고치세요.");
  Serial.println("   │ 3) 서버(server_knee_v1.py)가 켜져 있는지");
  Serial.println("   │ 4) 윈도우 방화벽에서 Python 을 허용했는지");
  Serial.println("   └────────────────────────────────────────────");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    if (!wsEverConnected) {
      Serial.printf("\n웹소켓 서버 연결됨! (%s:%u) — 데이터 전송을 시작합니다\n\n",
                    WS_HOST, WS_PORT);
    } else {
      Serial.printf("웹소켓 재연결됨 (%s:%u)\n", WS_HOST, WS_PORT);
    }
    wsEverConnected = true;
    wsFailCount = 0;
    wsConnectedAt = millis();

  } else if (type == WStype_DISCONNECTED) {
    if (!wsEverConnected) {
      // 한 번도 붙은 적 없음 = 서버 주소/방화벽/네트워크 문제
      wsFailCount++;
      if (wsFailCount == 1 || wsFailCount % 10 == 0) {
        Serial.printf("\n[접속 실패 %lu회] 서버 %s:%u 에 아직 한 번도 못 붙었습니다.\n",
                      wsFailCount, WS_HOST, WS_PORT);
        printNetHelp();
      }
    } else {
      // 붙었다가 끊김 = 신호 약함/서버 재시작/전원 문제
      unsigned long uptime = (millis() - wsConnectedAt) / 1000;
      Serial.printf("웹소켓 연결 끊김 (%lu초 유지됨) - 재연결 시도 중... "
                    "[WiFi RSSI %d dBm]\n", uptime, WiFi.RSSI());
      if (WiFi.RSSI() < -75)
        Serial.println("   ⚠ 신호가 약합니다(-75dBm 이하). 공유기와 가까이서 시험해보세요.");
    }
  }
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((byte)addr, (byte)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool writeRegVerified(uint8_t addr, uint8_t reg, uint8_t val) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
    delay(5);
    if (readReg(addr, reg) == val) return true;
  }
  return false;
}

// ── I2C 버스 스캔: 어떤 주소가 살아있는지 눈으로 확인용 ──
void scanI2C() {
  Serial.println("\n─── I2C 스캔 ───");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  주소 0x%02X 응답", a);
      if (a == ADDR_LOW)      Serial.print("   <- WAIST_LOW 로 설정됨");
      else if (a == ADDR_UP)  Serial.print("   <- WAIST_UP 으로 설정됨");
      else                    Serial.print("   <- ⚠ 코드에 없는 주소");
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  아무것도 응답 안 함! SDA/SCL 배선과 전원을 확인하세요.");
  } else if (found == 1) {
    Serial.println("  ⚠ 1개만 응답. 두 번째 MPU 의 AD0 핀 배선을 확인하세요.");
    Serial.println("     (AD0 = GND 또는 미연결 -> 0x68,  AD0 = 3.3V -> 0x69)");
  }
  Serial.println("────────────────\n");
}

void setupMPU(Sensor &s) {
  Wire.beginTransmission(s.addr);
  Wire.write(0x6B); Wire.write(0x00);   // 전원 켜기 (sleep 해제)
  if (Wire.endTransmission() != 0) {
    s.present = false;
    Serial.printf("MPU 0x%02X (%s) 응답 없음 - 이 센서는 건너뜁니다\n",
                  s.addr, s.id);
    return;
  }
  delay(20);

  bool ok = true;
  ok &= writeRegVerified(s.addr, 0x1A, 0x03);   // DLPF 41Hz
  ok &= writeRegVerified(s.addr, 0x1B, 0x08);   // gyro ±500 dps
  ok &= writeRegVerified(s.addr, 0x1C, 0x08);   // accel ±4g

  uint8_t acfg = readReg(s.addr, 0x1C);
  uint8_t gcfg = readReg(s.addr, 0x1B);
  const char* aR[] = {"±2g", "±4g", "±8g", "±16g"};
  const char* gR[] = {"±250dps", "±500dps", "±1000dps", "±2000dps"};
  Serial.printf("MPU 0x%02X (%s) 설정 %s", s.addr, s.id, ok ? "OK" : "실패!");
  if (acfg != 0xFF && gcfg != 0xFF)
    Serial.printf(" (가속도 %s, 자이로 %s)", aR[(acfg >> 3) & 3], gR[(gcfg >> 3) & 3]);
  Serial.println();
  s.present = ok;
}

// ── 6축 Madgwick (다리 펌웨어와 완전히 같은 수식) ──
void madgwickUpdate6(Sensor &s, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt) {
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float BETA = (gyroMag < GYRO_STATIC_THR) ? BETA_STATIC : BETA_NORMAL;

  float amag = sqrt(ax*ax + ay*ay + az*az);
  float dev = fabs(amag - s.accRef) - ACC_DEAD_ZONE;
  if (dev > 0) {
    float conf = 1.0 - dev * ACC_REJECT_K;
    if (conf < 0) conf = 0;
    BETA *= conf;
  }

  float qw = s.qw, qx = s.qx, qy = s.qy, qz = s.qz;
  gx *= PI/180.0; gy *= PI/180.0; gz *= PI/180.0;

  float qDotW = 0.5 * (-qx*gx - qy*gy - qz*gz);
  float qDotX = 0.5 * ( qw*gx + qy*gz - qz*gy);
  float qDotY = 0.5 * ( qw*gy - qx*gz + qz*gx);
  float qDotZ = 0.5 * ( qw*gz + qx*gy - qy*gx);

  float anorm = sqrt(ax*ax + ay*ay + az*az);
  if (anorm > 0.0001) {
    ax /= anorm; ay /= anorm; az /= anorm;
    float _4qw = 4.0*qw, _4qx = 4.0*qx, _4qy = 4.0*qy;
    float _8qx = 8.0*qx, _8qy = 8.0*qy;
    float qwqw = qw*qw, qxqx = qx*qx, qyqy = qy*qy, qzqz = qz*qz;
    float s0 = _4qw*qyqy + 2.0*qy*ax + _4qw*qxqx - 2.0*qx*ay;
    float s1 = _4qx*qzqz - 2.0*qz*ax + 4.0*qwqw*qx - 2.0*qw*ay - _4qx + _8qx*qxqx + _8qx*qyqy + _4qx*az;
    float s2 = 4.0*qwqw*qy + 2.0*qw*ax + _4qy*qzqz - 2.0*qz*ay - _4qy + _8qy*qxqx + _8qy*qyqy + _4qy*az;
    float s3 = 4.0*qxqx*qz - 2.0*qx*ax + 4.0*qyqy*qz - 2.0*qy*ay;
    float sNorm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm > 0.0001) {
      s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;
      qDotW -= BETA * s0; qDotX -= BETA * s1; qDotY -= BETA * s2; qDotZ -= BETA * s3;
    }
  }

  qw += qDotW * dt; qx += qDotX * dt; qy += qDotY * dt; qz += qDotZ * dt;
  float qNorm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  // qNorm 이 0 이면 NaN 이 되어 서버로 null 이 날아간다. 그럴 땐 갱신을 건너뛴다.
  if (qNorm > 0.0001) {
    s.qw = qw/qNorm; s.qx = qx/qNorm; s.qy = qy/qNorm; s.qz = qz/qNorm;
  }
}

void readAndUpdate(Sensor &s, float dt) {
  if (!s.present) return;

  Wire.beginTransmission(s.addr); Wire.write(0x3B);
  // 읽기에 실패하면 present 를 내려서 재시도 대상이 되게 한다
  if (Wire.endTransmission(false) != 0) { s.present = false; return; }
  Wire.requestFrom((byte)s.addr, (byte)14);
  if (Wire.available() < 14) return;

  // HANDOVER 실수 #8: Wire.read() 를 한 식 안에서 두 번 부르면
  // C++ 평가 순서가 보장되지 않는다. 버퍼로 먼저 받는다.
  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();
  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  // buf[6..7] = 온도 (버림)
  int16_t gx = (buf[8]  << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  float cgx = (gx - s.gxOff) / 65.5,   cgy = (gy - s.gyOff) / 65.5,   cgz = (gz - s.gzOff) / 65.5;
  float cax = (ax - s.axOff) / 8192.0, cay = (ay - s.ayOff) / 8192.0, caz = (az - s.azOff) / 8192.0;

  // 정지 상태에서 자이로 바이어스 천천히 자동 보정 (센서별로 따로)
  {
    float gm = sqrt(cgx*cgx + cgy*cgy + cgz*cgz);
    float am = sqrt(cax*cax + cay*cay + caz*caz);
    if (s.accRef <= 0.0) s.accRef = am;
    if (gm < BIAS_GYRO_THR && fabs(am - s.accRef) < BIAS_ACC_TOL) {
      s.gxOff += (gx - s.gxOff) * BIAS_ALPHA;
      s.gyOff += (gy - s.gyOff) * BIAS_ALPHA;
      s.gzOff += (gz - s.gzOff) * BIAS_ALPHA;
      s.accRef += (am - s.accRef) * BIAS_ALPHA * 10.0;
    }
  }

  madgwickUpdate6(s, cgx, cgy, cgz, cax, cay, caz, dt);
}

void sendQuat(Sensor &s) {
  if (!s.present) return;
  StaticJsonDocument<192> doc;
  doc["id"] = s.id;
  doc["z"] = 1;
  doc["qw"] = s.qw; doc["qx"] = s.qx; doc["qy"] = s.qy; doc["qz"] = s.qz;
  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  scanI2C();
  for (int i = 0; i < N_SENSORS; i++) setupMPU(sensors[i]);

  int nOk = 0;
  for (int i = 0; i < N_SENSORS; i++) if (sensors[i].present) nOk++;
  if (nOk == 0) {
    Serial.println("⚠ 두 센서 모두 실패. 배선을 확인하고 리셋하세요.");
  } else if (nOk < N_SENSORS) {
    Serial.println("⚠ 한 센서만 동작합니다. WAIST_LOW 가 살아있으면 고관절은 계산됩니다.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("WiFi 연결됨, IP: ");
  Serial.println(WiFi.localIP());

  Serial.printf("서버 접속 시도: ws://%s:%u\n", WS_HOST, WS_PORT);
  Serial.printf("  (이 보드 IP %s 와 앞 세 자리가 같아야 합니다)\n",
                WiFi.localIP().toString().c_str());
  webSocket.begin(WS_HOST, WS_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);

  lastFilterTime = micros();
  Serial.println(">>> [허리 보드] 준비 완료 (6축 x2) <<<");
  Serial.println(">>> BOOT 짧게 누르면 서버 전체 영점 (직립 자세에서) <<<");
  Serial.println(">>> 아래쪽 센서만 기울여서 WAIST_LOW 값이 변하는지 확인하세요 <<<");
}

void loop() {
  webSocket.loop();

  // WiFi 가 끊기면 웹소켓은 영원히 재시도만 한다. 이것도 구분해서 알린다.
  static bool wifiWasUp = true;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (wifiWasUp && !wifiUp) {
    Serial.println("\n[WiFi 끊김] 공유기 연결이 끊어졌습니다. 재연결 대기 중...");
    WiFi.reconnect();
  } else if (!wifiWasUp && wifiUp) {
    Serial.printf("[WiFi 복구] IP: %s\n", WiFi.localIP().toString().c_str());
  }
  wifiWasUp = wifiUp;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastFilterTime) / 1000000.0;

  if (dt >= 0.01) {          // 100Hz 필터 갱신
    lastFilterTime = nowMicros;
    if (dt > 0.02) dt = 0.02;

    for (int i = 0; i < N_SENSORS; i++) readAndUpdate(sensors[i], dt);

    unsigned long now = millis();

    // 못 읽는 센서가 있으면 5초마다 다시 붙여본다 (접촉 불량이면 복구된다)
    if (now - lastMpuRetry > 5000) {
      bool anyDown = false;
      for (int i = 0; i < N_SENSORS; i++) if (!sensors[i].present) anyDown = true;
      if (anyDown) {
        lastMpuRetry = now;
        for (int i = 0; i < N_SENSORS; i++) {
          if (sensors[i].present) continue;
          Serial.printf("MPU 0x%02X (%s) 재시도... ", sensors[i].addr, sensors[i].id);
          setupMPU(sensors[i]);
          if (sensors[i].present) Serial.printf(">>> %s 복구됨! <<<\n", sensors[i].id);
        }
      }
    }

    // BOOT 버튼 -> 서버에 영점 명령 (서버가 모든 보드 기준값을 동시에 잡음)
    bool btnPressed = (digitalRead(BOOT_BTN) == LOW && (now - lastBtnTime > 500));
    if (btnPressed && webSocket.isConnected()) {
      lastBtnTime = now;
      StaticJsonDocument<32> doc;
      doc["cmd"] = "zero";
      String out; serializeJson(doc, out);
      webSocket.sendTXT(out);
      Serial.println(">>> 영점 명령 서버로 전송 <<<");
    }

    if (webSocket.isConnected() && (now - lastTime > 100)) {   // 10Hz 전송
      lastTime = now;
      for (int i = 0; i < N_SENSORS; i++) sendQuat(sensors[i]);
    }

    // 디버그: 1초마다 두 센서 쿼터니언 출력
    // (아래쪽만 기울였을 때 WAIST_LOW 줄만 변해야 정상)
    static unsigned long lastDbg = 0;
    if (now - lastDbg > 1000) {
      lastDbg = now;
      const char* wsState = webSocket.isConnected() ? "서버연결됨"
                          : (wsEverConnected ? "끊김-재시도" : "미연결");
      for (int i = 0; i < N_SENSORS; i++) {
        Sensor &s = sensors[i];
        if (s.present)
          Serial.printf("%-10s (0x%02X) q=[%.3f,%.3f,%.3f,%.3f]  [%s]\n",
                        s.id, s.addr, s.qw, s.qx, s.qy, s.qz, wsState);
        else
          Serial.printf("%-10s (0x%02X) 없음  [%s]\n", s.id, s.addr, wsState);
      }
    }
  }
}
