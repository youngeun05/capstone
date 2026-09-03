// ═══════════════════════════════════════════════════════════════
//  하체(다리)용 센서 펌웨어 - ESP32 보드 1개 + IMU 1개 (6축)
// ═══════════════════════════════════════════════════════════════
//  팀의 sensor_elbow/shoulder 펌웨어에서 "MPU 1개" 구성으로 줄인 것.
//  같은 스케치를 BOARD_SELECT 만 바꿔서 각 보드에 업로드한다.
//
//    BOARD_SELECT 1 = R_THIGH (오른 허벅지)
//    BOARD_SELECT 2 = R_CALF  (오른 종아리)
//    BOARD_SELECT 3 = L_THIGH (왼 허벅지)
//    BOARD_SELECT 4 = L_CALF  (왼 종아리)
//
//  허리 보드는 이 스케치가 아니라 sensor_waist_dual.ino 를 쓴다.
//  (보드 1개에 MPU 2개 = WAIST_LOW / WAIST_UP 구성이라 코드가 다름)
//
//  ⚠ 업로드 전 체크리스트
//    1) BOARD_SELECT 확인 (허벅지/종아리 뒤바뀌면 각도가 이상해짐.
//       확인법: 무릎만 굽혔다 폈을 때 THIGH 값은 거의 안 변해야 정상)

//    2) WIFI_SSID / WS_HOST 확인 (인수인계 문서의 실수 #네트워크:
//       보드 불이 켜져도 서버 IP 가 다르면 접속 못 함. ipconfig 로 대조)
//    3) 오프셋 측정: mpu_calib_single.ino 로 이 보드의 값을 재서 입력.
//       (인수인계 문서의 실수 #1: 다른 보드 값 복사 금지)
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68        // AD0 핀을 GND(또는 미연결)로 = 0x68
#define BOOT_BTN 0

// ⭐⭐ 이 보드가 몸 어디에 붙는지 선택 ⭐⭐
#define BOARD_SELECT 4

#if BOARD_SELECT == 1
  #define SENSOR_ID "R_THIGH"
#elif BOARD_SELECT == 2
  #define SENSOR_ID "R_CALF"
#elif BOARD_SELECT == 3
  #define SENSOR_ID "L_THIGH"
#elif BOARD_SELECT == 4
  #define SENSOR_ID "L_CALF"
#else
  // 예전엔 #else 가 곧 L_CALF 라서, BOARD_SELECT 에 오타를 내면
  // 조용히 L_CALF 보드가 하나 더 생겼다. 이제는 컴파일이 멈춘다.
  #error "BOARD_SELECT 는 1~4 중 하나여야 합니다 (1=R_THIGH 2=R_CALF 3=L_THIGH 4=L_CALF). 허리 보드는 sensor_waist_dual.ino 를 쓰세요."
#endif

// ⭐ 와이파이 / 서버 정보 (팀과 동일 네트워크)
const char* WIFI_SSID = "IT-301_MOB_2.4G";
const char* WIFI_PASS = "mobile2019";
const char* WS_HOST   = "172.20.103.181";   // 서버 PC IP (ipconfig 로 확인!)
const uint16_t WS_PORT = 8765;

// ⭐ 이 보드의 가속도/자이로 오프셋 (mpu_calib_single.ino 로 실측 후 입력)
//   HANDOVER: "다리 4개 전부 미측정(0)" -> 반드시 측정할 것.
//   0 인 채로도 돌아가긴 하지만 영점/기울기 정확도가 떨어진다.
const int16_t AX_OFFSET = 0;
const int16_t AY_OFFSET = 0;
const int16_t AZ_OFFSET = 0;
float GX_OFFSET = 0.0;
float GY_OFFSET = 0.0;
float GZ_OFFSET = 0.0;

// ── 필터 설정 (팀 펌웨어와 동일) ──
const float BETA_NORMAL = 0.25;
const float BETA_STATIC = 0.5;
const float GYRO_STATIC_THR = 8.0;
const float BIAS_ALPHA    = 0.0008;
const float BIAS_GYRO_THR = 2.5;
const float BIAS_ACC_TOL  = 0.10;
const float ACC_DEAD_ZONE = 0.03;
const float ACC_REJECT_K  = 4.0;

struct IMUState {
  float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};
IMUState imu;
float accRef = 0.0;

unsigned long lastTime = 0;
unsigned long lastBtnTime = 0;
unsigned long lastFilterTime = 0;

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("웹소켓 서버 연결됨!");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("웹소켓 연결 끊김, 재연결 시도 중...");
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

bool mpuOk = false;              // MPU 설정 성공 여부
unsigned long lastMpuRetry = 0;

bool setupMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B); Wire.write(0x00);   // 전원 켜기 (sleep 해제)
  Wire.endTransmission();
  delay(20);

  bool ok = true;
  ok &= writeRegVerified(addr, 0x1A, 0x03);   // DLPF 41Hz
  ok &= writeRegVerified(addr, 0x1B, 0x08);   // gyro ±500 dps
  ok &= writeRegVerified(addr, 0x1C, 0x08);   // accel ±4g

  uint8_t acfg = readReg(addr, 0x1C);
  uint8_t gcfg = readReg(addr, 0x1B);
  const char* aR[] = {"±2g", "±4g", "±8g", "±16g"};
  const char* gR[] = {"±250dps", "±500dps", "±1000dps", "±2000dps"};
  Serial.printf("MPU 0x%02X 설정 %s", addr, ok ? "OK" : "실패!");
  if (acfg != 0xFF && gcfg != 0xFF)
    Serial.printf(" (가속도 %s, 자이로 %s)", aR[(acfg >> 3) & 3], gR[(gcfg >> 3) & 3]);
  Serial.println();
  return ok;
}

// ── 6축 Madgwick (팀 펌웨어의 6축 폴백 경로와 동일한 수식) ──
void madgwickUpdate6(IMUState &s, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt) {
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float BETA = (gyroMag < GYRO_STATIC_THR) ? BETA_STATIC : BETA_NORMAL;

  float amag = sqrt(ax*ax + ay*ay + az*az);
  float dev = fabs(amag - accRef) - ACC_DEAD_ZONE;
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
  s.qw = qw/qNorm; s.qx = qx/qNorm; s.qy = qy/qNorm; s.qz = qz/qNorm;
}

void readAndUpdate(float dt) {
  if (!mpuOk) return;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { mpuOk = false; return; }
  Wire.requestFrom((byte)MPU_ADDR, (byte)14);
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

  float cgx = (gx - GX_OFFSET) / 65.5,   cgy = (gy - GY_OFFSET) / 65.5,   cgz = (gz - GZ_OFFSET) / 65.5;
  float cax = (ax - AX_OFFSET) / 8192.0, cay = (ay - AY_OFFSET) / 8192.0, caz = (az - AZ_OFFSET) / 8192.0;

  // 정지 상태에서 자이로 바이어스 천천히 자동 보정 (팀 펌웨어와 동일)
  {
    float gm = sqrt(cgx*cgx + cgy*cgy + cgz*cgz);
    float am = sqrt(cax*cax + cay*cay + caz*caz);
    if (accRef <= 0.0) accRef = am;
    if (gm < BIAS_GYRO_THR && fabs(am - accRef) < BIAS_ACC_TOL) {
      GX_OFFSET += (gx - GX_OFFSET) * BIAS_ALPHA;
      GY_OFFSET += (gy - GY_OFFSET) * BIAS_ALPHA;
      GZ_OFFSET += (gz - GZ_OFFSET) * BIAS_ALPHA;
      accRef += (am - accRef) * BIAS_ALPHA * 10.0;
    }
  }

  madgwickUpdate6(imu, cgx, cgy, cgz, cax, cay, caz, dt);
}

void sendQuat() {
  StaticJsonDocument<192> doc;
  doc["id"] = SENSOR_ID;
  doc["z"] = 1;
  doc["qw"] = imu.qw; doc["qx"] = imu.qx; doc["qy"] = imu.qy; doc["qz"] = imu.qz;
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

  mpuOk = setupMPU(MPU_ADDR);
  if (!mpuOk) {
    Serial.println("⚠⚠ MPU 를 못 읽습니다. 이 상태로도 서버에는 접속하지만");
    Serial.println("   쿼터니언이 [1,0,0,0] 에서 안 움직입니다.");
    Serial.println("   (서버 콘솔에서 'sensors' 를 치면 '값 고정' 으로 나옵니다)");
    Serial.println("   SDA/SCL 배선, 전원, 납땜, AD0 핀을 확인하세요.");
    Serial.println("   5초마다 자동으로 다시 시도합니다.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("WiFi 연결됨, IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin(WS_HOST, WS_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);

  lastFilterTime = micros();
  Serial.printf(">>> [%s] 준비 완료 (6축, MPU 1개) <<<\n", SENSOR_ID);
  Serial.println(">>> BOOT 짧게 누르면 서버 전체 영점 (직립 자세에서) <<<");
}

void loop() {
  webSocket.loop();

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastFilterTime) / 1000000.0;

  if (dt >= 0.01) {          // 100Hz 필터 갱신
    lastFilterTime = nowMicros;
    if (dt > 0.02) dt = 0.02;

    readAndUpdate(dt);

    unsigned long now = millis();

    // MPU 를 못 읽는 상태면 5초마다 다시 붙여본다 (접촉 불량이면 복구된다)
    if (!mpuOk && now - lastMpuRetry > 5000) {
      lastMpuRetry = now;
      Serial.print("MPU 재시도... ");
      mpuOk = setupMPU(MPU_ADDR);
      if (mpuOk) Serial.println(">>> MPU 복구됨! <<<");
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
      sendQuat();
    }

    // 디버그: 1초마다 쿼터니언 출력
    static unsigned long lastDbg = 0;
    if (now - lastDbg > 1000) {
      lastDbg = now;
      Serial.printf("%s q=[%.3f,%.3f,%.3f,%.3f]%s\n",
        SENSOR_ID, imu.qw, imu.qx, imu.qy, imu.qz,
        mpuOk ? "" : "   ⚠ MPU 안 읽힘 (값이 안 변합니다)");
    }
  }
}
