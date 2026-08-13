/*
 * 견갑골 어깨 센서 보드 - 9축 (MPU 1개짜리 보드 2개 공용 펌웨어)
 *
 * ┌─ 왜 보드를 둘로 쪼갰나 ────────────────────────────────────────────┐
 * │ AK8963 지자계는 I2C 주소가 0x0C 로 고정이다.                       │
 * │ 한 보드에 MPU 두 개를 달면 둘 다 0x0C 라 동시에 못 쓴다.           │
 * │ 그래서 팔 보드는 "상완만 9축, 전완은 6축"으로 타협해야 했다.        │
 * │                                                                    │
 * │ 보드를 나누면 각 보드에 MPU가 하나뿐이라 충돌 자체가 없다.          │
 * │ 좌우 견갑골 둘 다 온전히 9축으로 쓸 수 있다.                       │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * ⭐ 업로드할 때 아래 BOARD_SELECT 숫자만 바꿔서 각 보드에 올리면 된다.
 *      1 = 왼쪽 견갑골  (L_SHOULDER)
 *      2 = 오른쪽 견갑골 (R_SHOULDER)
 *
 * I2C 주소는 자동으로 찾는다. AD0 를 GND 에 물리면 0x68, 3.3V 면 0x69 인데
 * 어느 쪽이든 부팅 때 스캔해서 응답하는 주소를 쓴다.
 * (두 보드가 서로 다른 ESP32 이므로 주소가 같아도 아무 문제 없다.
 *  새로 만드는 거라면 둘 다 AD0 -> GND 로 통일해서 0x68 로 두는 걸 권한다.
 *  그래야 mpu_calib_single.ino 를 수정 없이 쓸 수 있다.)
 *
 * 서버는 고칠 필요가 없다. 센서 ID(L_SHOULDER / R_SHOULDER)가 그대로라
 * 서버 입장에서는 "보드 하나가 둘로 늘어난 것"일 뿐이다.
 *
 * 부팅 로그에서 반드시 확인할 것:
 *   "지자계 OK  |m|=xx.x uT (정상)"  이 떠야 9축이 동작하는 것이다.
 *   |m| 이 25~65 를 벗어나면 hard-iron 보정값이 틀린 것.
 */
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ⭐⭐ 이 보드가 어느 쪽인지 선택 (1 = 왼쪽, 2 = 오른쪽)
#define BOARD_SELECT 1

#define SDA_PIN 8
#define SCL_PIN 9
#define AK8963_ADDR 0x0C
#define BOOT_BTN 0

// ⭐ 와이파이 / 서버 정보
const char* WIFI_SSID = "IT-301_MOB_2.4G";
const char* WIFI_PASS = "mobile2019";
const char* WS_HOST   = "192.168.0.65";
const uint16_t WS_PORT = 8765;

// ══════════════════════════════════════════════════════════════
//  보드별 설정
//
//  ⚠ 아래 가속도/자이로 오프셋은 '기존 어깨 보드'에서 측정한 값이다.
//    - 그 MPU 모듈을 그대로 새 보드로 옮겼다면 이 값을 쓰면 된다
//      (오프셋은 ESP32 가 아니라 MPU 칩에 딸린 값이다)
//    - 새 MPU 모듈을 쓴다면 mpu_calib_single.ino 로 다시 재야 한다
//
//  ⚠ 지자계 오프셋은 보드를 조립한 뒤 반드시 새로 재야 한다.
//    hard-iron 은 주변 부품(배터리, 배선, 납땜)이 만드는 자기장이라
//    보드 구성이 바뀌면 값도 완전히 달라진다.
// ══════════════════════════════════════════════════════════════
#if BOARD_SELECT == 1
  #define SENSOR_ID "L_SHOULDER"
  // 기존 어깨 보드의 MPU1(0x68) 측정값
  const int16_t AX_OFFSET = 2086, AY_OFFSET = 527, AZ_OFFSET = -1366;
  float GX_OFFSET = -73.9, GY_OFFSET = -101.5, GZ_OFFSET = 2.5;
  // ⭐⭐ mag_calibration.ino 로 측정해서 채우세요 (0 이면 6축으로 동작)
  float MX_OFFSET = 0.0, MY_OFFSET = 0.0, MZ_OFFSET = 0.0;
  float MX_SCALE = 1.0, MY_SCALE = 1.0, MZ_SCALE = 1.0;

#elif BOARD_SELECT == 2
  #define SENSOR_ID "R_SHOULDER"
  // 기존 어깨 보드의 MPU2(0x69) 측정값
  const int16_t AX_OFFSET = 438, AY_OFFSET = -81, AZ_OFFSET = 817;
  float GX_OFFSET = -99.6, GY_OFFSET = 31.8, GZ_OFFSET = -36.7;
  // ⭐⭐ mag_calibration.ino 로 측정해서 채우세요 (0 이면 6축으로 동작)
  float MX_OFFSET = 0.0, MY_OFFSET = 0.0, MZ_OFFSET = 0.0;
  float MX_SCALE = 1.0, MY_SCALE = 1.0, MZ_SCALE = 1.0;

#else
  #error "BOARD_SELECT 는 1 또는 2 여야 합니다"
#endif

uint8_t MPU_ADDR = 0x68;   // setup() 에서 자동 탐색해 덮어쓴다

struct IMUState {
  float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};
IMUState imu;

struct MagCal {
  float asaX = 1.0, asaY = 1.0, asaZ = 1.0;   // AK8963 ROM 감도보정계수
  float lastUT = 0.0;                         // 최근 자기장 크기 (uT)
  unsigned long lastOkMs = 0;                 // 마지막 성공 시각
  bool ready = false;                         // 보정값이 들어왔는지
};
MagCal magcal;

// 정지 시 가속도 크기. 첫 샘플로 학습한다.
// (보드마다 1.02~1.20g 로 편차가 커서 1.0 고정으로 시작하면 정지 판정이 안 될 수 있음)
float accRef = 0.0;

unsigned long lastTime = 0;
unsigned long lastBtnTime = 0;
unsigned long lastFilterTime = 0;

// ── 필터 설정 (팔 보드에서 드리프트 테스트로 검증한 값) ──
const float BETA_NORMAL = 0.25;     // 움직이는 중
const float BETA_STATIC = 0.5;      // 거의 정지 -> 중력으로 더 세게 끌어당김
const float GYRO_STATIC_THR = 8.0;  // deg/s
// 자이로 바이어스 실시간 재추정 (온도에 따라 변하므로 부팅 때 값만 쓰면 쌓인다)
const float BIAS_ALPHA    = 0.0008;
const float BIAS_GYRO_THR = 2.5;
const float BIAS_ACC_TOL  = 0.10;
// 동작 중 가속도 신뢰도 낮추기 (빠르게 움직이면 중력 대신 동작가속도를 잰다)
const float ACC_DEAD_ZONE = 0.03;
const float ACC_REJECT_K  = 4.0;

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

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

// 쓴 뒤 되읽어 확인하고 안 먹었으면 재시도.
// ⚠ 이 설정이 실패하면 칩이 기본값(±2g, ±250dps)으로 남아 나눗셈 상수와 2배 어긋난다.
bool writeRegVerified(uint8_t addr, uint8_t reg, uint8_t val) {
  for (int attempt = 0; attempt < 3; attempt++) {
    writeReg(addr, reg, val);
    delay(5);
    if (readReg(addr, reg) == val) return true;
  }
  return false;
}

// 0x68 / 0x69 중 응답하는 쪽을 찾는다.
bool findMPU() {
  for (uint8_t a : {0x68, 0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      MPU_ADDR = a;
      uint8_t who = readReg(a, 0x75);
      Serial.printf("MPU 발견: 0x%02X (WHO_AM_I=0x%02X %s)\n",
                    a, who, who == 0x71 ? "MPU9250" : "?");
      return true;
    }
  }
  return false;
}

void setupMPU(uint8_t addr) {
  writeReg(addr, 0x6B, 0x00);   // 슬립 해제
  delay(20);                    // 짧으면 이후 설정이 씹힌다

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
}

// ── 지자계(AK8963) 초기화 ──
// MPU9250 의 지자계는 칩 안에 있지만 별도 I2C 장치(0x0C)라
// MPU 를 bypass 모드로 열어야 ESP32 가 직접 읽을 수 있다.
// 이 보드는 MPU 가 하나뿐이라 주소 충돌 걱정 없이 그냥 열어두면 된다.
bool setupMag(uint8_t mpuAddr, MagCal &cal) {
  writeReg(mpuAddr, 0x6A, 0x00);   // USER_CTRL: I2C 마스터 끄기
  writeReg(mpuAddr, 0x37, 0x02);   // INT_PIN_CFG: BYPASS_EN
  delay(10);

  uint8_t wia = readReg(AK8963_ADDR, 0x00);   // WIA 는 항상 0x48
  if (wia != 0x48) {
    Serial.printf("AK8963 응답 없음 (WIA=0x%02X) -> 6축으로 동작\n", wia);
    return false;
  }

  writeReg(AK8963_ADDR, 0x0B, 0x01); delay(10);   // CNTL2 soft reset
  writeReg(AK8963_ADDR, 0x0A, 0x00); delay(10);   // CNTL1 power down

  // fuse ROM 에서 축별 감도보정계수(ASA) 읽기
  writeReg(AK8963_ADDR, 0x0A, 0x0F); delay(10);
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x10);
  Wire.endTransmission(false);
  Wire.requestFrom((byte)AK8963_ADDR, (byte)3);
  uint8_t asa[3] = {128, 128, 128};
  if (Wire.available() >= 3) { asa[0] = Wire.read(); asa[1] = Wire.read(); asa[2] = Wire.read(); }
  cal.asaX = (asa[0] - 128) / 256.0 + 1.0;   // ASA -> 배율
  cal.asaY = (asa[1] - 128) / 256.0 + 1.0;
  cal.asaZ = (asa[2] - 128) / 256.0 + 1.0;
  delay(10);
  writeReg(AK8963_ADDR, 0x0A, 0x00); delay(10);   // power down

  // 연속측정 모드2 (100Hz) + 16bit.  CNTL1 = 16bit(bit4) | mode 0110
  writeReg(AK8963_ADDR, 0x0A, 0x16);
  delay(10);

  Serial.printf("AK8963 초기화됨  ASA=[%.3f, %.3f, %.3f]\n",
                cal.asaX, cal.asaY, cal.asaZ);
  return true;
}

// 지자계 읽기. 성공 시 true, 값은 uT 단위로 mx,my,mz 에 채움.
bool readMag(MagCal &cal, float &mx, float &my, float &mz) {
  if (!cal.ready) return false;   // 미보정이면 아예 안 쓴다 (6축 폴백)

  uint8_t st1 = readReg(AK8963_ADDR, 0x02);
  if (!(st1 & 0x01)) return false;    // DRDY 아직 아님

  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);   // HXL 부터
  Wire.endTransmission(false);
  Wire.requestFrom((byte)AK8963_ADDR, (byte)7);   // 6바이트 + ST2
  if (Wire.available() < 7) return false;

  // ⚠ a | (b << 8) 을 한 줄에 쓰면 Wire.read() 두 번의 평가 순서가
  //   보장되지 않아 바이트가 뒤집힐 수 있다. 반드시 따로 받는다.
  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();
  uint8_t st2 = Wire.read();
  if (st2 & 0x08) return false;   // HOFL 자기장 오버플로

  int16_t rx = (int16_t)((uint16_t)xl | ((uint16_t)xh << 8));   // 리틀엔디안
  int16_t ry = (int16_t)((uint16_t)yl | ((uint16_t)yh << 8));
  int16_t rz = (int16_t)((uint16_t)zl | ((uint16_t)zh << 8));

  // ⚠ 단위 일치: mag_calibration.ino 가 RAW 정수에서 min/max 를 잡으므로
  //   hard-iron 오프셋도 RAW 단위. 반드시 RAW 에서 먼저 뺀다.
  float fx = rx - MX_OFFSET, fy = ry - MY_OFFSET, fz = rz - MZ_OFFSET;
  const float LSB_TO_UT = 0.15;   // 16bit 모드
  mx = fx * MX_SCALE * LSB_TO_UT * cal.asaX;
  my = fy * MY_SCALE * LSB_TO_UT * cal.asaY;
  mz = fz * MZ_SCALE * LSB_TO_UT * cal.asaZ;

  cal.lastUT = sqrt(mx*mx + my*my + mz*mz);
  cal.lastOkMs = millis();
  return true;
}

// ── 9축 Madgwick (MARG) ──
// 지자계가 없으면(읽기 실패/미보정) 자동으로 6축 업데이트로 폴백한다.
void madgwickUpdate9(IMUState &imu, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz, bool haveMag,
                     float dt, float accRef) {
  // 정지에 가까울수록 중력 쪽으로 더 세게 끌어당겨 드리프트를 털어낸다.
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float BETA = (gyroMag < GYRO_STATIC_THR) ? BETA_STATIC : BETA_NORMAL;

  // 가속도가 순수 중력에서 벗어난 만큼 보정 강도를 깎는다.
  // 데드존을 둬서 센서 스케일 오차(1.02~1.04g)로는 깎이지 않게 한다.
  float amag = sqrt(ax*ax + ay*ay + az*az);
  float dev = fabs(amag - accRef) - ACC_DEAD_ZONE;
  if (dev > 0) {
    float conf = 1.0 - dev * ACC_REJECT_K;
    if (conf < 0) conf = 0;
    BETA *= conf;
  }

  float qw = imu.qw, qx = imu.qx, qy = imu.qy, qz = imu.qz;
  gx *= PI/180.0; gy *= PI/180.0; gz *= PI/180.0;

  float qDotW = 0.5 * (-qx*gx - qy*gy - qz*gz);
  float qDotX = 0.5 * ( qw*gx + qy*gz - qz*gy);
  float qDotY = 0.5 * ( qw*gy - qx*gz + qz*gx);
  float qDotZ = 0.5 * ( qw*gz + qx*gy - qy*gx);

  float anorm = sqrt(ax*ax + ay*ay + az*az);
  float mnorm = sqrt(mx*mx + my*my + mz*mz);
  bool useMag = haveMag && mnorm > 0.0001;

  if (anorm > 0.0001) {
    ax /= anorm; ay /= anorm; az /= anorm;
    float s0, s1, s2, s3;

    if (useMag) {
      mx /= mnorm; my /= mnorm; mz /= mnorm;

      float _2qw = 2.0*qw, _2qx = 2.0*qx, _2qy = 2.0*qy, _2qz = 2.0*qz;
      float qwqw = qw*qw, qwqx = qw*qx, qwqy = qw*qy, qwqz = qw*qz;
      float qxqx = qx*qx, qxqy = qx*qy, qxqz = qx*qz;
      float qyqy = qy*qy, qyqz = qy*qz, qzqz = qz*qz;

      // 측정한 자기장을 월드로 돌려 기준 방향(_2bx 수평, _2bz 수직) 추정
      float _2qwmx = 2.0*qw*mx, _2qwmy = 2.0*qw*my, _2qwmz = 2.0*qw*mz;
      float _2qxmx = 2.0*qx*mx;
      float hx = mx*qwqw - _2qwmy*qz + _2qwmz*qy + mx*qxqx + _2qx*my*qy + _2qx*mz*qz - mx*qyqy - mx*qzqz;
      float hy = _2qwmx*qz + my*qwqw - _2qwmz*qx + _2qxmx*qy - my*qxqx + my*qyqy + _2qy*mz*qz - my*qzqz;
      float _2bx = sqrt(hx*hx + hy*hy);
      float _2bz = -_2qwmx*qy + _2qwmy*qx + mz*qwqw + _2qxmx*qz - mz*qxqx + _2qy*my*qz - mz*qyqy + mz*qzqz;
      float _4bx = 2.0*_2bx, _4bz = 2.0*_2bz;

      // 목적함수의 그래디언트 (가속도 + 지자계 결합)
      s0 = -_2qy*(2.0*(qxqz - qwqy) - ax)
           + _2qx*(2.0*(qwqx + qyqz) - ay)
           - _2bz*qy*(_2bx*(0.5 - qyqy - qzqz) + _2bz*(qxqz - qwqy) - mx)
           + (-_2bx*qz + _2bz*qx)*(_2bx*(qxqy - qwqz) + _2bz*(qwqx + qyqz) - my)
           + _2bx*qy*(_2bx*(qwqy + qxqz) + _2bz*(0.5 - qxqx - qyqy) - mz);
      s1 = _2qz*(2.0*(qxqz - qwqy) - ax)
           + _2qw*(2.0*(qwqx + qyqz) - ay)
           - 4.0*qx*(1.0 - 2.0*(qxqx + qyqy) - az)
           + _2bz*qz*(_2bx*(0.5 - qyqy - qzqz) + _2bz*(qxqz - qwqy) - mx)
           + (_2bx*qy + _2bz*qw)*(_2bx*(qxqy - qwqz) + _2bz*(qwqx + qyqz) - my)
           + (_2bx*qz - _4bz*qx)*(_2bx*(qwqy + qxqz) + _2bz*(0.5 - qxqx - qyqy) - mz);
      s2 = -_2qw*(2.0*(qxqz - qwqy) - ax)
           + _2qz*(2.0*(qwqx + qyqz) - ay)
           - 4.0*qy*(1.0 - 2.0*(qxqx + qyqy) - az)
           + (-_4bx*qy - _2bz*qw)*(_2bx*(0.5 - qyqy - qzqz) + _2bz*(qxqz - qwqy) - mx)
           + (_2bx*qx + _2bz*qz)*(_2bx*(qxqy - qwqz) + _2bz*(qwqx + qyqz) - my)
           + (_2bx*qw - _4bz*qy)*(_2bx*(qwqy + qxqz) + _2bz*(0.5 - qxqx - qyqy) - mz);
      s3 = _2qx*(2.0*(qxqz - qwqy) - ax)
           + _2qy*(2.0*(qwqx + qyqz) - ay)
           + (-_4bx*qz + _2bz*qx)*(_2bx*(0.5 - qyqy - qzqz) + _2bz*(qxqz - qwqy) - mx)
           + (-_2bx*qw + _2bz*qy)*(_2bx*(qxqy - qwqz) + _2bz*(qwqx + qyqz) - my)
           + _2bx*qx*(_2bx*(qwqy + qxqz) + _2bz*(0.5 - qxqx - qyqy) - mz);
    } else {
      // 6축 폴백
      float _4qw = 4.0*qw, _4qx = 4.0*qx, _4qy = 4.0*qy;
      float _8qx = 8.0*qx, _8qy = 8.0*qy;
      float qwqw = qw*qw, qxqx = qx*qx, qyqy = qy*qy, qzqz = qz*qz;
      s0 = _4qw*qyqy + 2.0*qy*ax + _4qw*qxqx - 2.0*qx*ay;
      s1 = _4qx*qzqz - 2.0*qz*ax + 4.0*qwqw*qx - 2.0*qw*ay - _4qx + _8qx*qxqx + _8qx*qyqy + _4qx*az;
      s2 = 4.0*qwqw*qy + 2.0*qw*ax + _4qy*qzqz - 2.0*qz*ay - _4qy + _8qy*qxqx + _8qy*qyqy + _4qy*az;
      s3 = 4.0*qxqx*qz - 2.0*qx*ax + 4.0*qyqy*qz - 2.0*qy*ay;
    }

    float sNorm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm > 0.0001) {
      s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;
      qDotW -= BETA * s0; qDotX -= BETA * s1; qDotY -= BETA * s2; qDotZ -= BETA * s3;
    }
  }

  qw += qDotW * dt; qx += qDotX * dt; qy += qDotY * dt; qz += qDotZ * dt;
  float qNorm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  imu.qw = qw/qNorm; imu.qx = qx/qNorm; imu.qy = qy/qNorm; imu.qz = qz/qNorm;
}

void readAndUpdate(float dt) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((byte)MPU_ADDR, (byte)14);
  if (Wire.available() < 14) return;

  // ⚠ a << 8 | b 를 한 줄에 쓰면 평가 순서가 보장되지 않는다. 버퍼로 먼저 받는다.
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  int16_t ax = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);   // 가속도는 빅엔디안
  int16_t ay = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
  int16_t az = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
  // b[6],b[7] = 온도 (안 씀)
  int16_t gx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
  int16_t gy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  int16_t gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

  float cgx = (gx - GX_OFFSET) / 65.5,   cgy = (gy - GY_OFFSET) / 65.5,   cgz = (gz - GZ_OFFSET) / 65.5;
  float cax = (ax - AX_OFFSET) / 8192.0, cay = (ay - AY_OFFSET) / 8192.0, caz = (az - AZ_OFFSET) / 8192.0;

  float mx = 0, my = 0, mz = 0;
  bool haveMag = readMag(magcal, mx, my, mz);

  // 축 정렬: AK8963 은 x/y 가 바뀌고 z 가 뒤집힌 프레임을 쓴다.
  float mxA = my, myA = mx, mzA = -mz;

  // 확실히 정지일 때만 자이로 바이어스와 가속도 기준크기를 갱신한다.
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

  madgwickUpdate9(imu, cgx, cgy, cgz, cax, cay, caz, mxA, myA, mzA, haveMag, dt, accRef);
}

void sendQuat(const char* id, float qw, float qx, float qy, float qz) {
  StaticJsonDocument<192> doc;
  doc["id"] = id;
  doc["z"] = 1;
  doc["qw"] = qw; doc["qx"] = qx; doc["qy"] = qy; doc["qz"] = qz;
  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
}

// 부팅 직후 자기장 크기를 재서 보정이 제대로 됐는지 알려준다.
void reportMagHealth() {
  Serial.println("\n[지자계 상태] 지구 자기장은 25~65 uT. 벗어나면 hard-iron 값이 틀린 것.");
  if (!magcal.ready) {
    Serial.println("  6축 (지자계 미사용)");
    return;
  }
  float sum = 0; int cnt = 0;
  for (int k = 0; k < 40 && cnt < 10; k++) {
    float mx, my, mz;
    if (readMag(magcal, mx, my, mz)) { sum += sqrt(mx*mx + my*my + mz*mz); cnt++; }
    delay(15);
  }
  if (cnt == 0) {
    Serial.println("  ⚠ 데이터 안 옴 -> 6축으로 폴백");
    magcal.ready = false;
  } else {
    float m = sum / cnt;
    Serial.printf("  지자계 OK  |m|=%.1f uT  %s\n", m,
                  (m > 20 && m < 75) ? "(정상)" : "⚠ 범위 벗어남 - 재캘리브레이션 필요");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  Serial.printf("\n=== 어깨 보드 (9축) : %s ===\n", SENSOR_ID);

  if (!findMPU()) {
    Serial.println("MPU 를 못 찾았습니다. 배선을 확인하세요 (SDA=8, SCL=9).");
    Serial.println("i2c_scan_pure.ino 로 주소가 뜨는지 먼저 보세요.");
    while (true) delay(1000);
  }
  setupMPU(MPU_ADDR);

  // 지자계: hard-iron 보정값이 없으면 쓰지 않는다.
  // 보정 안 된 지자계는 yaw 를 통째로 틀어버려 6축보다 나쁘다.
  bool magOk = setupMag(MPU_ADDR, magcal);
  if (!magOk) {
    magcal.ready = false;
  } else if (MX_OFFSET == 0.0 && MY_OFFSET == 0.0 && MZ_OFFSET == 0.0) {
    magcal.ready = false;
    Serial.println("⚠ hard-iron 보정값이 0 입니다. 6축으로 동작합니다.");
    Serial.println("  mag_calibration.ino 로 측정해서 M*_OFFSET 에 넣으세요.");
  } else {
    magcal.ready = true;
  }
  reportMagHealth();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("\nWiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nWiFi 연결됨, IP: %s\n", WiFi.localIP().toString().c_str());

  webSocket.begin(WS_HOST, WS_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);

  lastFilterTime = micros();
  Serial.printf(">>> [%s 9축] 준비 완료 <<<\n", SENSOR_ID);
  Serial.println(">>> BOOT 짧게 누르면 영점 (차렷 자세에서) <<<");
}

void loop() {
  webSocket.loop();

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastFilterTime) / 1000000.0;
  if (dt < 0.01) return;

  lastFilterTime = nowMicros;
  if (dt > 0.02) dt = 0.02;

  readAndUpdate(dt);

  unsigned long now = millis();

  // BOOT 버튼 -> 서버에 영점 명령 (영점은 서버가 관리한다)
  if (digitalRead(BOOT_BTN) == LOW && (now - lastBtnTime > 500) && webSocket.isConnected()) {
    lastBtnTime = now;
    StaticJsonDocument<32> doc;
    doc["cmd"] = "zero";
    String out; serializeJson(doc, out);
    webSocket.sendTXT(out);
    Serial.println(">>> 영점 명령 서버로 전송 <<<");
  }

  if (webSocket.isConnected() && (now - lastTime > 100)) {
    lastTime = now;
    sendQuat(SENSOR_ID, imu.qw, imu.qx, imu.qy, imu.qz);
  }

  // 1초마다 상태 출력. |m| 이 계속 25~65 uT 안이면 지자계가 정상 동작 중.
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 1000) {
    lastDbg = now;
    bool magAlive = (now - magcal.lastOkMs) < 500;
    Serial.printf("%s q=[%.3f,%.3f,%.3f,%.3f]  |m|=%.1f uT %s\n",
      SENSOR_ID, imu.qw, imu.qx, imu.qy, imu.qz,
      magAlive ? magcal.lastUT : 0.0,
      !magAlive ? "<- 지자계 안 읽힘! 6축으로 동작 중"
      : (magcal.lastUT < 20 || magcal.lastUT > 75)
                ? "<- 범위 벗어남! hard-iron 재측정 필요"
                : "(정상)");
  }
}
