#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU1_ADDR 0x68
#define MPU2_ADDR 0x69
#define AK8963_ADDR 0x0C      // 지자계 (bypass 모드에서 직접 접근)
#define BOOT_BTN 0

// ⭐⭐ 센서 ID (실제 연결: 0x68=전완, 0x69=상완)
//   실측 근거: 팔꿈치만 90도 굽혔을 때 0x68 이 99.6도, 0x69 가 4.9도 움직였다.
//   상완은 팔꿈치를 굽혀도 제자리여야 하므로 0x69 가 상완이다.
#define SENSOR_ID_1 "R_FOREARM"    // MPU1(0x68) = 전완
#define SENSOR_ID_2 "R_UPPERARM"   // MPU2(0x69) = 상완

// ⭐ 와이파이 / 서버 정보
const char* WIFI_SSID = "IT-301_MOB_2.4G";
const char* WIFI_PASS = "mobile2019";
const char* WS_HOST   = "192.168.0.65";
const uint16_t WS_PORT = 8765;

// ⭐ 오른팔 보드 오프셋 (가속도/자이로)
const int16_t MPU1_AX_OFFSET = -375;
const int16_t MPU1_AY_OFFSET = -593;
const int16_t MPU1_AZ_OFFSET = 37;
float MPU1_GX_OFFSET = -7.3;
float MPU1_GY_OFFSET = -75.7;
float MPU1_GZ_OFFSET = 21.6;

const int16_t MPU2_AX_OFFSET = 317;
const int16_t MPU2_AY_OFFSET = -290;
const int16_t MPU2_AZ_OFFSET = -536;
float MPU2_GX_OFFSET = 11.3;
float MPU2_GY_OFFSET = 113.8;
float MPU2_GZ_OFFSET = -33.8;

// ⭐ 지자계 hard-iron 오프셋 (8자 캘리브레이션 측정값, raw 단위)
//   9축은 상완=MPU2(0x69). 상완 센서로 측정한 값.
float MPU1_MX_OFFSET = 0.0, MPU1_MY_OFFSET = 0.0, MPU1_MZ_OFFSET = 0.0;
float MPU2_MX_OFFSET = -27.0, MPU2_MY_OFFSET = 443.5, MPU2_MZ_OFFSET = -345.5;
// soft-iron 스케일
float MPU1_MX_SCALE = 1.0, MPU1_MY_SCALE = 1.0, MPU1_MZ_SCALE = 1.0;
float MPU2_MX_SCALE = 1.023, MPU2_MY_SCALE = 0.991, MPU2_MZ_SCALE = 0.987;

struct IMUState {
  float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};
IMUState imu1, imu2;

// 지자계 감도 보정계수 (AK8963 ROM의 ASA값, 축별)
struct MagCal {
  float asaX = 1.0, asaY = 1.0, asaZ = 1.0;
  // 진단용: 지자계가 실제로 읽히고 있는지 확인하기 위한 값.
  // 읽기에 실패해도 readMag 는 조용히 false 를 반환하고 6축으로 떨어지므로,
  // 이 값이 없으면 9축이 동작 중인지 아닌지 알 방법이 없다.
  float lastUT = 0.0;            // 최근 자기장 크기 (uT). 25~65 면 정상
  unsigned long lastOkMs = 0;    // 마지막 성공 시각. 오래되면 지자계가 죽은 것
};
MagCal mag1cal, mag2cal;
// 캘리브레이션은 별도 스케치(mag_calibration.ino)에서 수행 → 위 오프셋에 입력

float accRef1 = 0.0, accRef2 = 0.0;

unsigned long lastTime = 0;
unsigned long lastBtnTime = 0;
unsigned long lastFilterTime = 0;

// ── 필터 설정 ──
const float BETA_NORMAL = 0.25;
const float BETA_STATIC = 0.5;
const float GYRO_STATIC_THR = 8.0;
const float BIAS_ALPHA    = 0.0008;
const float BIAS_GYRO_THR = 2.5;
const float BIAS_ACC_TOL  = 0.10;
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

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

// ── 지자계(AK8963) 초기화 ──
// MPU9250의 지자계는 칩 내부에 있지만 별도 I2C 장치(0x0C)라,
// MPU를 bypass 모드로 열어야 ESP32가 직접 읽을 수 있다.
void setupMag(uint8_t mpuAddr, MagCal &cal) {
  // 0) 다른 MPU의 bypass를 먼저 닫아 지자계(0x0C) 충돌 방지
  uint8_t otherMpu = (mpuAddr == MPU1_ADDR) ? MPU2_ADDR : MPU1_ADDR;
  writeReg(otherMpu, 0x6B, 0x00); delay(5);
  writeReg(otherMpu, 0x37, 0x00);  // 다른 MPU bypass OFF
  delay(5);
  // 1) 이 MPU bypass 활성화: INT_PIN_CFG(0x37)의 BYPASS_EN 비트(0x02)
  writeReg(mpuAddr, 0x37, 0x02);
  // USER_CTRL(0x6A) I2C_MST 비활성 (bypass 쓰려면 마스터 꺼야 함)
  writeReg(mpuAddr, 0x6A, 0x00);
  delay(10);

  // 2) AK8963 리셋 & 전원
  writeReg(AK8963_ADDR, 0x0B, 0x01);  // CNTL2 soft reset
  delay(10);
  writeReg(AK8963_ADDR, 0x0A, 0x00);  // CNTL1 power down
  delay(10);

  // 3) ROM 액세스 모드로 감도보정계수(ASA) 읽기
  writeReg(AK8963_ADDR, 0x0A, 0x0F);  // fuse ROM access
  delay(10);
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x10);  // ASAX
  Wire.endTransmission(false);
  Wire.requestFrom((byte)AK8963_ADDR, (byte)3);
  uint8_t asa[3] = {128,128,128};
  if (Wire.available() >= 3) { asa[0]=Wire.read(); asa[1]=Wire.read(); asa[2]=Wire.read(); }
  // ASA → 실제 배율: (asa-128)/256 + 1
  cal.asaX = (asa[0]-128)/256.0 + 1.0;
  cal.asaY = (asa[1]-128)/256.0 + 1.0;
  cal.asaZ = (asa[2]-128)/256.0 + 1.0;
  delay(10);
  writeReg(AK8963_ADDR, 0x0A, 0x00);  // power down
  delay(10);

  // 4) 연속측정 모드2 (100Hz) + 16bit 출력
  //    CNTL1(0x0A): 0x16 = 16bit(bit4) + mode 0110(100Hz)
  writeReg(AK8963_ADDR, 0x0A, 0x16);
  delay(10);

  Serial.printf("AK8963(via 0x%02X) ASA=[%.2f, %.2f, %.2f]\n",
                mpuAddr, cal.asaX, cal.asaY, cal.asaZ);
}

// 지자계 읽기. 성공 시 true, 값은 uT 단위로 mx,my,mz에 채움.
bool readMag(MagCal &cal, float mxO, float myO, float mzO,
             float mxS, float myS, float mzS,
             float &mx, float &my, float &mz) {
  // 데이터 준비 확인 (ST1 0x02 DRDY 비트)
  uint8_t st1 = readReg(AK8963_ADDR, 0x02);
  if (!(st1 & 0x01)) return false;

  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);  // HXL부터
  Wire.endTransmission(false);
  Wire.requestFrom((byte)AK8963_ADDR, (byte)7);  // 6바이트 + ST2
  if (Wire.available() < 7) return false;
  int16_t rx = Wire.read() | (Wire.read() << 8);  // 리틀엔디안!
  int16_t ry = Wire.read() | (Wire.read() << 8);
  int16_t rz = Wire.read() | (Wire.read() << 8);
  uint8_t st2 = Wire.read();
  if (st2 & 0x08) return false;  // HOFL 자기장 오버플로

  // ⚠ 단위 일치: 캘리브레이션(mag_calibration.ino)은 RAW 정수값에서
  //   min/max를 잡으므로, 오프셋도 RAW 단위. 따라서 RAW에서 먼저 빼야 함.
  float rxf = rx, ryf = ry, rzf = rz;
  // hard-iron 제거 (raw 단위)
  rxf -= mxO; ryf -= myO; rzf -= mzO;
  // soft-iron 스케일 + ASA 감도보정 + LSB→uT (0.15)
  float scale = 0.15;
  mx = rxf * mxS * scale * cal.asaX;
  my = ryf * myS * scale * cal.asaY;
  mz = rzf * mzS * scale * cal.asaZ;

  // 진단용 기록 (9축이 실제로 먹고 있는지 확인용)
  cal.lastUT = sqrt(mx*mx + my*my + mz*mz);
  cal.lastOkMs = millis();
  return true;
}

void setupMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B); Wire.write(0x00);
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
}

// ── 9축 Madgwick ──
// 6축(가속도+자이로)에 지자계(mx,my,mz)를 추가해 yaw를 절대적으로 잡는다.
// 지자계가 없으면(읽기 실패) 자동으로 6축 업데이트로 폴백.
void madgwickUpdate9(IMUState &imu, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz, bool haveMag,
                     float dt, float accRef) {
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float BETA = (gyroMag < GYRO_STATIC_THR) ? BETA_STATIC : BETA_NORMAL;

  float amag = sqrt(ax*ax + ay*ay + az*az);
  float dev = fabs(amag - accRef) - ACC_DEAD_ZONE;
  if (dev > 0) {
    float conf = 1.0 - dev * ACC_REJECT_K;
    if (conf < 0) conf = 0;
    BETA *= conf;
  }

  float qw = imu.qw, qx = imu.qx, qy = imu.qy, qz = imu.qz;
  gx *= PI/180.0; gy *= PI/180.0; gz *= PI/180.0;

  // 자이로 기반 변화율
  float qDotW = 0.5 * (-qx*gx - qy*gy - qz*gz);
  float qDotX = 0.5 * ( qw*gx + qy*gz - qz*gy);
  float qDotY = 0.5 * ( qw*gy - qx*gz + qz*gx);
  float qDotZ = 0.5 * ( qw*gz + qx*gy - qy*gx);

  float anorm = sqrt(ax*ax + ay*ay + az*az);
  bool useMag = haveMag && (fabs(mx)+fabs(my)+fabs(mz) > 0.01);
  float mnorm = sqrt(mx*mx + my*my + mz*mz);
  if (useMag && mnorm < 0.0001) useMag = false;

  if (anorm > 0.0001) {
    ax /= anorm; ay /= anorm; az /= anorm;

    float s0, s1, s2, s3;

    if (useMag) {
      mx /= mnorm; my /= mnorm; mz /= mnorm;

      // 보조 변수 (Madgwick AHRS 표준 유도식)
      float _2qw = 2.0*qw, _2qx = 2.0*qx, _2qy = 2.0*qy, _2qz = 2.0*qz;
      float _2qwmx, _2qwmy, _2qwmz, _2qxmx;
      float qwqw = qw*qw, qwqx = qw*qx, qwqy = qw*qy, qwqz = qw*qz;
      float qxqx = qx*qx, qxqy = qx*qy, qxqz = qx*qz;
      float qyqy = qy*qy, qyqz = qy*qz, qzqz = qz*qz;

      // 지구 자기장 기준 방향 추정 (자기장을 현재 자세로 회전)
      _2qwmx = 2.0*qw*mx; _2qwmy = 2.0*qw*my; _2qwmz = 2.0*qw*mz; _2qxmx = 2.0*qx*mx;
      float hx = mx*qwqw - _2qwmy*qz + _2qwmz*qy + mx*qxqx + _2qx*my*qy + _2qx*mz*qz - mx*qyqy - mx*qzqz;
      float hy = _2qwmx*qz + my*qwqw - _2qwmz*qx + _2qxmx*qy - my*qxqx + my*qyqy + _2qy*mz*qz - my*qzqz;
      float _2bx = sqrt(hx*hx + hy*hy);
      float _2bz = -_2qwmx*qy + _2qwmy*qx + mz*qwqw + _2qxmx*qz - mz*qxqx + _2qy*my*qz - mz*qyqy + mz*qzqz;
      float _4bx = 2.0*_2bx, _4bz = 2.0*_2bz;

      // 목적함수의 그래디언트 (가속도+지자계 결합)
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
      // 6축 폴백 (지자계 없음)
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

void readAndUpdate(uint8_t addr, IMUState &imu, MagCal &cal, bool useMagSensor,
                   int16_t axO, int16_t ayO, int16_t azO,
                   float &gxO, float &gyO, float &gzO,
                   float mxO, float myO, float mzO,
                   float mxS, float myS, float mzS,
                   float dt, float &accRef) {
  Wire.beginTransmission(addr); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((byte)addr, (byte)14);
  if (Wire.available() >= 14) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    float cgx = (gx - gxO) / 65.5,   cgy = (gy - gyO) / 65.5,   cgz = (gz - gzO) / 65.5;
    float cax = (ax - axO) / 8192.0, cay = (ay - ayO) / 8192.0, caz = (az - azO) / 8192.0;

    // 지자계 읽기 (상완만. 전완은 useMagSensor=false → 6축 폴백)
    float mx=0, my=0, mz=0;
    bool haveMag = false;
    if (useMagSensor) {
      haveMag = readMag(cal, mxO, myO, mzO, mxS, myS, mzS, mx, my, mz);
    }

    // 축 정렬: AK8963 x/y 교환, z 부호 반전 (Madgwick 프레임에 맞춤)
    float mx_aligned =  my;
    float my_aligned =  mx;
    float mz_aligned = -mz;

    {
      float gm = sqrt(cgx*cgx + cgy*cgy + cgz*cgz);
      float am = sqrt(cax*cax + cay*cay + caz*caz);
      if (accRef <= 0.0) accRef = am;
      if (gm < BIAS_GYRO_THR && fabs(am - accRef) < BIAS_ACC_TOL) {
        gxO += (gx - gxO) * BIAS_ALPHA;
        gyO += (gy - gyO) * BIAS_ALPHA;
        gzO += (gz - gzO) * BIAS_ALPHA;
        accRef += (am - accRef) * BIAS_ALPHA * 10.0;
      }
    }

    madgwickUpdate9(imu, cgx, cgy, cgz, cax, cay, caz,
                    mx_aligned, my_aligned, mz_aligned, haveMag, dt, accRef);
  }
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  setupMPU(MPU1_ADDR);
  setupMPU(MPU2_ADDR);

  // 지자계 초기화: 진짜 상완인 MPU2(0x69)에 9축 적용
  setupMag(MPU2_ADDR, mag2cal);
  Serial.println(">>> 지자계는 상완(MPU2=0x69)만 사용. 전완(MPU1=0x68)은 6축 <<<");

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
  Serial.println(">>> [오른팔 9축] 준비 완료 <<<");
  Serial.println(">>> BOOT 짧게 누르면 영점 (팔 펴고). 캘리브레이션은 mag_calibration.ino로 별도 <<<");
}

void loop() {
  webSocket.loop();

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastFilterTime) / 1000000.0;

  if (dt >= 0.01) {
    lastFilterTime = nowMicros;
    if (dt > 0.02) dt = 0.02;

    // 전완(MPU1=0x68): 6축. 지자계 안 읽음.
    readAndUpdate(MPU1_ADDR, imu1, mag1cal, false,
                  MPU1_AX_OFFSET, MPU1_AY_OFFSET, MPU1_AZ_OFFSET,
                  MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET,
                  MPU1_MX_OFFSET, MPU1_MY_OFFSET, MPU1_MZ_OFFSET,
                  MPU1_MX_SCALE, MPU1_MY_SCALE, MPU1_MZ_SCALE,
                  dt, accRef1);

    // 상완(MPU2=0x69): 9축. 지자계 사용 (진짜 상완이라 방향 정확).
    readAndUpdate(MPU2_ADDR, imu2, mag2cal, true,
                  MPU2_AX_OFFSET, MPU2_AY_OFFSET, MPU2_AZ_OFFSET,
                  MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET,
                  MPU2_MX_OFFSET, MPU2_MY_OFFSET, MPU2_MZ_OFFSET,
                  MPU2_MX_SCALE, MPU2_MY_SCALE, MPU2_MZ_SCALE,
                  dt, accRef2);

    unsigned long now = millis();

    // BOOT 버튼 → 영점 (서버가 기준값 잡고 재방송). 캘리브레이션은 별도 스케치.
    bool btnPressed = (digitalRead(BOOT_BTN) == LOW && (now - lastBtnTime > 500));
    if (btnPressed && webSocket.isConnected()) {
      lastBtnTime = now;
      StaticJsonDocument<32> doc;
      doc["cmd"] = "zero";
      String out; serializeJson(doc, out);
      webSocket.sendTXT(out);
      Serial.println(">>> 영점 명령 서버로 전송 <<<");
    }

    if (webSocket.isConnected() && (now - lastTime > 100)) {
      lastTime = now;
      sendQuat(SENSOR_ID_1, imu1.qw, imu1.qx, imu1.qy, imu1.qz);
      sendQuat(SENSOR_ID_2, imu2.qw, imu2.qx, imu2.qy, imu2.qz);
    }

    // 디버그: 1초마다 쿼터니언 + 지자계 상태 출력
    //   배치 확인 - 팔꿈치만 굽혔을 때 UA 값이 거의 안 변해야 맞게 붙은 것
    //   9축 확인 - |m| 이 25~65 uT 안에서 안정적이어야 지자계가 먹고 있는 것
    static unsigned long lastDbg = 0;
    if (now - lastDbg > 1000) {
      lastDbg = now;
      bool magAlive = (now - mag2cal.lastOkMs) < 500;
      Serial.printf("FA q=[%.3f,%.3f,%.3f,%.3f]  UA q=[%.3f,%.3f,%.3f,%.3f]  |m|=%.1f uT %s\n",
        imu1.qw,imu1.qx,imu1.qy,imu1.qz, imu2.qw,imu2.qx,imu2.qy,imu2.qz,
        magAlive ? mag2cal.lastUT : 0.0,
        !magAlive          ? "<- 지자계 안 읽힘! 6축으로 동작 중"
        : (mag2cal.lastUT < 20 || mag2cal.lastUT > 75)
                           ? "<- 범위 벗어남! hard-iron 재측정 필요"
                           : "(정상)");
    }
  }
}
