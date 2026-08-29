// ─────────────────────────────────────────────────────────────
//  어깨 보드 (L/R SHOULDER) — 순수 6축 + 동적 자이로 캘리브레이션
//  v1.3 : 찢어진 읽기(torn read) 차단, 중앙값/MAD 기반 견고한 통계,
//         실제 표류량 측정(drift) 명령 추가
//
//  v1.2 대비 바뀐 점
//   1. DATA_RDY 플래그를 확인하고 읽어 찢어진 읽기 차단
//      - MPU가 레지스터를 갱신하는 중에 읽으면 상위/하위 바이트가
//        서로 다른 샘플에서 와서 ±256 LSB(약 3.9도/초) 가짜 스파이크가 난다.
//        400개 중 하나만 있어도 표준편차가 12.8 LSB로 올라가 경고를 띄운다.
//   2. 표준편차 대신 중앙값 + MAD 사용 (이상치 하나에 무너지지 않음)
//   3. 이상치 개수를 따로 세어 '움직임'과 '배선 문제'를 구분
//   4. 결과를 LSB가 아니라 도/초로 표시
//   5. drift 명령: 자이로를 적분해 실제로 분당 몇 도 밀리는지 직접 측정
// ─────────────────────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU1_ADDR 0x68
#define MPU2_ADDR 0x69
#define BOOT_BTN 0

// ⭐⭐ 센서 ID (어깨 - 0x68이 왼쪽, 0x69가 오른쪽)
#define BOARD_NAME  "SHOULDER_BOARD"
#define FW_VERSION  "shoulder-6ax-1.4"
#define SENSOR_ID_1 "L_SHOULDER"
#define SENSOR_ID_2 "R_SHOULDER"

const char* WIFI_SSID = "IT-301_MOB_2.4G";
const char* WIFI_PASS = "mobile2019";
const char* WS_HOST   = "192.168.0.10";
const uint16_t WS_PORT = 8765;

// ⭐ 어깨 보드 가속도 오프셋 (기존 실측값 유지)
const int16_t MPU1_AX_OFFSET = 1800;
const int16_t MPU1_AY_OFFSET = 613;
const int16_t MPU1_AZ_OFFSET = -1398;

const int16_t MPU2_AX_OFFSET = -532;
const int16_t MPU2_AY_OFFSET = -815;
const int16_t MPU2_AZ_OFFSET = 900;

// ⭐ 동적 자이로 오프셋 (setup에서 자동 측정)
float MPU1_GX_OFFSET = 0.0, MPU1_GY_OFFSET = 0.0, MPU1_GZ_OFFSET = 0.0;
float MPU2_GX_OFFSET = 0.0, MPU2_GY_OFFSET = 0.0, MPU2_GZ_OFFSET = 0.0;

// ── 자이로 측정 설정 ─────────────────────────────────────────
const bool  AUTO_GYRO_CAL      = true;
const int   MAX_CAL_SAMPLES    = 400;
const int   GYRO_CAL_N         = 300;    // 100Hz 기준 약 3초
const int   GYRO_SETTLE_N      = 50;     // 측정 전 버리는 샘플

const float GYRO_LSB_PER_DPS   = 65.5f;  // ±500 dps 설정 기준
const float SHAKE_WARN_DPS     = 0.30f;  // 이보다 크면 실제로 움직이는 중
const float BIAS_WARN_DPS      = 0.50f;  // 이보다 크면 cal 재측정 권장
const float OUTLIER_DPS        = 2.0f;   // 이 이상 튀면 이상치로 간주
const float OUTLIER_WARN_PCT   = 1.0f;   // 이상치가 이 비율을 넘으면 배선 의심

// ⭐ 캘리브레이션 결과 보관 (서버 보고 / info 명령용)
struct CalInfo {
  float gx = 0, gy = 0, gz = 0;
  float sd = -1.0;      // 흔들림 (도/초)
  float outPct = 0.0;   // 읽기 오류 비율 (%)
  int   n  = 0;         // 유효 샘플 수
  bool  ok = false;     // 흔들림·이상치 모두 문턱 이하
};
CalInfo cal1, cal2;

struct IMUState {
  float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};
IMUState imu1, imu2;

// ⭐ 멀티태스킹 데이터 보호용 Mutex 락
portMUX_TYPE imuMutex = portMUX_INITIALIZER_UNLOCKED;

float accRef1 = 0.0, accRef2 = 0.0;
unsigned long lastBtnTime = 0;
unsigned long lastSendTime = 0;
unsigned long lastPrintTime = 0;
bool helloSent = false;          // 접속 후 0점 결과를 보고했는지
bool debugPrint = false;         // 쿼터니언 콘솔 출력 (print 명령으로 토글)

// ⭐ [v1.4] 서버에서 받은 명령. 콜백에서 바로 실행하면 cal(7초)/drift(60초)
//    동안 webSocket.loop() 가 멈춰 연결이 끊긴다. loop() 로 넘겨서 실행한다.
String pendingCmd = "";

// ⭐ [v1.4] check / drift 결과를 서버로 보고하기 위해 보관한다.
struct DiagInfo {
  float sd     = -1.0;   // 흔들림 (도/초)
  float bias   = -1.0;   // 잔여 바이어스 크기 (도/초)
  float perMin = -1.0;   // 예상 표류 (도/분)
  float outPct = -1.0;   // 읽기 오류 (%)
  float drift  = -1.0;   // 실측 표류 (도/분)
  bool  has    = false;
};
DiagInfo diag1, diag2;

// ⭐ 시리얼 명령용: IMU 태스크 일시정지 핸드셰이크
volatile bool imuPauseReq = false;
volatile bool imuPaused   = false;

// ⭐ 측정용 정적 버퍼 (스택 절약)
static int16_t g_buf[3][MAX_CAL_SAMPLES];
static int16_t g_tmp[MAX_CAL_SAMPLES];
static int16_t g_dev[MAX_CAL_SAMPLES];
static int     g_fallback = 0;   // DATA_RDY 폴링이 실패해 그냥 읽은 횟수

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

const char* CMD_HELP =
  "\n명령 목록 (줄바꿈 설정을 'Both NL & CR' 로 두세요)\n"
  "  check   흔들림 + 잔여 바이어스 진단 (오프셋 유지, 약 3초)\n"
  "  cal     자이로 0점 재측정 (재부팅 없이, 서버 교정 유지)\n"
  "  drift   30초간 실제 표류량 측정 (가장 믿을 수 있는 지표)\n"
  "  info    현재 오프셋 / 연결 상태\n"
  "  print   쿼터니언 실시간 출력 켜기·끄기\n"
  "  reboot  ESP32 소프트 리셋 (= reset)\n"
  "  (서버 콘솔의 bcal / bcheck / bdrift 로도 원격 실행됩니다)\n"
  "  zero    서버로 전체 영점 명령 전송\n";

// ═══════════════════════════════════════════════════════════════
//  1) I2C 기본 입출력
// ═══════════════════════════════════════════════════════════════
uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((byte)addr, (byte)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((byte)addr, n);
  if (Wire.available() < n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
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

void setupMPU(uint8_t addr) {
  Wire.beginTransmission(addr); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(20);
  writeRegVerified(addr, 0x1A, 0x03);   // DLPF 41Hz
  writeRegVerified(addr, 0x19, 0x09);   // 샘플레이트 100Hz
  writeRegVerified(addr, 0x1B, 0x08);   // 자이로 ±500 dps
  writeRegVerified(addr, 0x1C, 0x08);   // 가속도 ±4 g

  writeReg(addr, 0x37, 0x00);           // 지자계 bypass 차단
  writeRegVerified(addr, 0x38, 0x01);   // ⭐ DATA_RDY 상태비트 활성화 (핀은 미사용)
}

// ═══════════════════════════════════════════════════════════════
//  2) 자이로 측정 (찢어진 읽기 차단 + 견고한 통계)
// ═══════════════════════════════════════════════════════════════

// ── DATA_RDY 를 기다렸다가 한 샘플만 정확히 읽는다 ────────────
bool readGyroFresh(uint8_t addr, int16_t* g, uint32_t timeoutMs = 30) {
  uint8_t b[6];
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    uint8_t st = readReg(addr, 0x3A);          // INT_STATUS
    if (st != 0xFF && (st & 0x01)) {           // RAW_DATA_RDY
      if (!readRegs(addr, 0x43, b, 6)) return false;
      g[0] = (int16_t)(b[0] << 8 | b[1]);
      g[1] = (int16_t)(b[2] << 8 | b[3]);
      g[2] = (int16_t)(b[4] << 8 | b[5]);
      return true;
    }
    delayMicroseconds(300);
  }
  // 상태비트가 동작하지 않는 개체를 위한 대비책: 그냥 읽는다
  if (!readRegs(addr, 0x43, b, 6)) return false;
  g[0] = (int16_t)(b[0] << 8 | b[1]);
  g[1] = (int16_t)(b[2] << 8 | b[3]);
  g[2] = (int16_t)(b[4] << 8 | b[5]);
  g_fallback++;
  return true;
}

static int cmp16(const void* a, const void* b) {
  int16_t x = *(const int16_t*)a, y = *(const int16_t*)b;
  return (x > y) - (x < y);
}

// ── 중앙값 ────────────────────────────────────────────────────
float medianOf(int16_t* v, int n) {
  memcpy(g_tmp, v, n * sizeof(int16_t));
  qsort(g_tmp, n, sizeof(int16_t), cmp16);
  return (n & 1) ? (float)g_tmp[n/2] : 0.5f * (g_tmp[n/2 - 1] + g_tmp[n/2]);
}

// ── MAD 기반 견고한 표준편차 (이상치 하나에 무너지지 않음) ────
float robustSD(int16_t* v, int n, float med) {
  for (int i = 0; i < n; i++) g_dev[i] = (int16_t)fabsf(v[i] - med);
  float mad = medianOf(g_dev, n);
  return mad * 1.4826f;   // 정규분포에서 MAD -> 표준편차 환산 계수
}

// ── 샘플 수집. 반환값은 유효 샘플 수, outliers 에 이상치 개수 ──
int collectGyro(uint8_t addr, int n, int* outliers) {
  if (n > MAX_CAL_SAMPLES) n = MAX_CAL_SAMPLES;
  int got = 0;
  for (int i = 0; i < n; i++) {
    int16_t g[3];
    if (readGyroFresh(addr, g)) {
      g_buf[0][got] = g[0]; g_buf[1][got] = g[1]; g_buf[2][got] = g[2];
      got++;
    }
  }
  *outliers = 0;
  if (got >= 10) {
    float thr = OUTLIER_DPS * GYRO_LSB_PER_DPS;
    for (int ax = 0; ax < 3; ax++) {
      float med = medianOf(g_buf[ax], got);
      for (int i = 0; i < got; i++)
        if (fabsf(g_buf[ax][i] - med) > thr) (*outliers)++;
    }
  }
  return got;
}

// ── 자이로 0점 측정 ───────────────────────────────────────────
void calibrateGyro(uint8_t addr, float &gxO, float &gyO, float &gzO,
                   CalInfo &info, const char* name) {
  for (int i = 0; i < GYRO_SETTLE_N; i++) { int16_t d[3]; readGyroFresh(addr, d); }

  int outliers = 0;
  int n = collectGyro(addr, GYRO_CAL_N, &outliers);
  info.n = n;

  if (n < GYRO_CAL_N / 2) {
    info.ok = false;
    Serial.printf("  %-12s 측정 실패 (응답 %d/%d회). 배선과 주소를 확인하세요\n",
                  name, n, GYRO_CAL_N);
    return;
  }

  float med[3], sd[3];
  for (int ax = 0; ax < 3; ax++) {
    med[ax] = medianOf(g_buf[ax], n);
    sd[ax]  = robustSD(g_buf[ax], n, med[ax]);
  }
  float sdDps  = max(sd[0], max(sd[1], sd[2])) / GYRO_LSB_PER_DPS;
  float outPct = 100.0f * outliers / n;

  gxO = med[0]; gyO = med[1]; gzO = med[2];
  info.gx = med[0]; info.gy = med[1]; info.gz = med[2];
  info.sd = sdDps; info.outPct = outPct;
  info.ok = (sdDps <= SHAKE_WARN_DPS) && (outPct <= OUTLIER_WARN_PCT);

  Serial.printf("  %-12s 0점 [%+7.1f, %+7.1f, %+7.1f]  흔들림 %.3f 도/초  이상치 %.1f%%\n",
                name, gxO, gyO, gzO, sdDps, outPct);
  if (sdDps > SHAKE_WARN_DPS)
    Serial.println("     >> 측정 중 보드가 움직였습니다. 내려놓고 cal 을 다시 실행하세요");
  if (outPct > OUTLIER_WARN_PCT)
    Serial.printf("     >> 읽기 오류 %.1f%%. I2C 배선 길이, 풀업 저항, 접지를 확인하세요\n", outPct);
}

// ── 오프셋은 그대로 두고 현재 상태만 진단 ─────────────────────
void measureNoise(uint8_t addr, const char* name,
                  float gxO, float gyO, float gzO,
                  DiagInfo &info, int samples = 250) {
  int outliers = 0;
  int n = collectGyro(addr, samples, &outliers);
  if (n < 10) {
    Serial.printf("  %-12s 응답 없음 (배선 확인)\n", name);
    info.has = false;
    return;
  }

  float med[3], sd[3];
  for (int ax = 0; ax < 3; ax++) {
    med[ax] = medianOf(g_buf[ax], n);
    sd[ax]  = robustSD(g_buf[ax], n, med[ax]);
  }
  float sdDps  = max(sd[0], max(sd[1], sd[2])) / GYRO_LSB_PER_DPS;
  float outPct = 100.0f * outliers / n;

  // 지금 오프셋을 뺀 잔여 바이어스 = 실제로 흐르고 있는 각속도
  float rx = (med[0] - gxO) / GYRO_LSB_PER_DPS;
  float ry = (med[1] - gyO) / GYRO_LSB_PER_DPS;
  float rz = (med[2] - gzO) / GYRO_LSB_PER_DPS;
  float rmag = sqrtf(rx*rx + ry*ry + rz*rz);

  // ⭐ [v1.4] 서버 보고용으로 담아둔다
  info.sd = sdDps; info.bias = rmag; info.outPct = outPct;
  info.perMin = rmag * 60.0f; info.has = true;

  Serial.printf("  %-12s 샘플 %3d   흔들림 %.3f 도/초 %s\n", name, n, sdDps,
                (sdDps > SHAKE_WARN_DPS) ? "<< 지금 움직이는 중" : "(정지 양호)");
  Serial.printf("               잔여 바이어스 [%+.3f, %+.3f, %+.3f] 크기 %.3f 도/초 %s\n",
                rx, ry, rz, rmag, (rmag > BIAS_WARN_DPS) ? "<< cal 권장" : "(양호)");
  Serial.printf("               예상 표류 %.1f 도/분   읽기 오류 %.1f%% %s\n",
                rmag * 60.0f, outPct,
                (outPct > OUTLIER_WARN_PCT) ? "<< 배선 점검" : "");
}

// ── 실제 표류량 직접 측정 (가장 믿을 수 있는 지표) ────────────
void measureDrift(uint8_t addr, const char* name,
                  float gxO, float gyO, float gzO,
                  DiagInfo &info, int seconds = 30) {
  double ix = 0, iy = 0, iz = 0;
  uint32_t t0 = millis(), tPrev = micros();
  int n = 0;

  while (millis() - t0 < (uint32_t)seconds * 1000) {
    int16_t g[3];
    if (!readGyroFresh(addr, g)) continue;
    uint32_t tNow = micros();
    float dt = (tNow - tPrev) / 1000000.0f;
    tPrev = tNow;
    if (dt > 0.1f) continue;                 // 통신 지연 구간은 버린다

    ix += (g[0] - gxO) / GYRO_LSB_PER_DPS * dt;
    iy += (g[1] - gyO) / GYRO_LSB_PER_DPS * dt;
    iz += (g[2] - gzO) / GYRO_LSB_PER_DPS * dt;
    n++;
  }

  double mag = sqrt(ix*ix + iy*iy + iz*iz);
  float el = (millis() - t0) / 1000.0f;
  float perMin = mag / el * 60.0;
  info.drift = perMin; info.has = true;      // ⭐ [v1.4] 서버 보고용
  Serial.printf("  %-12s %.0f초 [%+.1f, %+.1f, %+.1f] 도, 크기 %.1f 도 (%.2f 도/분)\n",
                name, el, ix, iy, iz, mag, perMin);
  if (perMin > 10.0)
    Serial.println("     >> 분당 10도를 넘습니다. 앞/옆/대각 판정이 1~2분이면 무너집니다");
}

// ═══════════════════════════════════════════════════════════════
//  3) IMU 태스크 정지 / 재개
// ═══════════════════════════════════════════════════════════════
bool pauseIMU(uint32_t timeoutMs = 600) {
  imuPauseReq = true;
  uint32_t t0 = millis();
  while (!imuPaused && millis() - t0 < timeoutMs) delay(2);
  if (!imuPaused) {
    imuPauseReq = false;
    Serial.println("IMU 태스크 정지 실패 (명령 취소)");
    return false;
  }
  delay(20);   // 진행 중이던 I2C 전송이 끝날 여유
  return true;
}
void resumeIMU() { imuPauseReq = false; }

// ═══════════════════════════════════════════════════════════════
//  4) 시리얼 명령
// ═══════════════════════════════════════════════════════════════

// ⭐ [v1.4] check / drift 결과를 서버로 보고
void sendDiag(const char* kind) {
  StaticJsonDocument<512> doc;
  doc["type"]  = "diag";
  doc["board"] = BOARD_NAME;
  doc["fw"]    = FW_VERSION;
  doc["kind"]  = kind;

  JsonArray arr = doc.createNestedArray("d");
  const char* ids[2]   = { SENSOR_ID_1, SENSOR_ID_2 };
  DiagInfo*   infos[2] = { &diag1, &diag2 };
  for (int i = 0; i < 2; i++) {
    if (!infos[i]->has) continue;
    JsonObject o = arr.createNestedObject();
    o["id"] = ids[i];
    if (infos[i]->sd     >= 0) o["sd"]      = round(infos[i]->sd * 1000) / 1000.0;
    if (infos[i]->bias   >= 0) o["bias"]    = round(infos[i]->bias * 1000) / 1000.0;
    if (infos[i]->perMin >= 0) o["per_min"] = round(infos[i]->perMin * 10) / 10.0;
    if (infos[i]->outPct >= 0) o["out"]     = round(infos[i]->outPct * 10) / 10.0;
    if (infos[i]->drift  >= 0) o["drift"]   = round(infos[i]->drift * 100) / 100.0;
  }

  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
  Serial.printf(">>> %s 결과를 서버로 보고했습니다 <<<\n", kind);
}

// ⭐ [v1.4] 이 명령이 우리 보드 것인지 (target 이 없거나 ALL 이면 전체 대상)
bool targetMatches(const char* target) {
  if (target == nullptr || target[0] == '\0') return true;
  if (!strcmp(target, "ALL") || !strcmp(target, "all")) return true;
  return !strcmp(target, BOARD_NAME);
}

void runCmd(String c) {
  c.toLowerCase();

  if (c == "check") {
    Serial.println("\n[check] 흔들림 진단 (오프셋은 유지). 가만히 두세요");
    if (!pauseIMU()) return;
    g_fallback = 0;
    diag1 = DiagInfo(); diag2 = DiagInfo();
    measureNoise(MPU1_ADDR, SENSOR_ID_1, MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET, diag1);
    measureNoise(MPU2_ADDR, SENSOR_ID_2, MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET, diag2);
    resumeIMU();
    if (g_fallback > 0)
      Serial.printf("  (참고: DATA_RDY 대기 실패 %d회 — 상태비트가 동작하지 않는 개체일 수 있습니다)\n", g_fallback);
    if (webSocket.isConnected()) sendDiag("check");
    Serial.println("[check] 완료");

  } else if (c == "cal") {
    Serial.println("\n[cal] 자이로 0점 재측정. 보드를 내려놓고 움직이지 마세요");
    if (!pauseIMU()) return;
    g_fallback = 0;
    calibrateGyro(MPU1_ADDR, MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET, cal1, SENSOR_ID_1);
    calibrateGyro(MPU2_ADDR, MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET, cal2, SENSOR_ID_2);
    accRef1 = accRef2 = 0.0;
    resumeIMU();
    helloSent = false;   // 새 측정 결과를 서버에 다시 보고
    Serial.println("[cal] 완료 (자세는 유지되므로 서버 교정은 다시 안 해도 됩니다)");

  } else if (c == "drift") {
    Serial.println("\n[drift] 센서당 30초씩 실제 표류량 측정. 보드를 내려놓고 두세요");
    if (!pauseIMU()) return;
    diag1 = DiagInfo(); diag2 = DiagInfo();
    measureDrift(MPU1_ADDR, SENSOR_ID_1, MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET, diag1, 30);
    measureDrift(MPU2_ADDR, SENSOR_ID_2, MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET, diag2, 30);
    resumeIMU();
    if (webSocket.isConnected()) sendDiag("drift");
    Serial.println("[drift] 완료");

  } else if (c == "info") {
    Serial.printf("\n[info] 가동 %lu초\n", millis() / 1000);
    Serial.printf("  %-12s 오프셋 [%+8.1f, %+8.1f, %+8.1f]  흔들림 %.3f 도/초  이상치 %.1f%%  %s\n",
                  SENSOR_ID_1, MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET,
                  cal1.sd, cal1.outPct, cal1.ok ? "OK" : "불량");
    Serial.printf("  %-12s 오프셋 [%+8.1f, %+8.1f, %+8.1f]  흔들림 %.3f 도/초  이상치 %.1f%%  %s\n",
                  SENSOR_ID_2, MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET,
                  cal2.sd, cal2.outPct, cal2.ok ? "OK" : "불량");
    Serial.println("  (오프셋은 정지 구간마다 실시간으로 미세 조정되므로 cal 직후 값과 다를 수 있습니다)");
    Serial.printf("  WiFi %s / 서버 %s\n",
                  WiFi.status() == WL_CONNECTED ? "연결됨" : "끊김",
                  webSocket.isConnected() ? "연결됨" : "끊김");

  } else if (c == "print") {
    debugPrint = !debugPrint;
    Serial.printf("\n[print] 쿼터니언 출력 %s\n", debugPrint ? "켜짐" : "꺼짐");

  } else if (c == "reboot" || c == "reset") {
    Serial.println("\n[reboot] 2초 뒤 재부팅합니다. USB 시리얼이 잠시 끊겼다 돌아옵니다");
    Serial.flush();
    delay(2000);
    ESP.restart();

  } else if (c == "zero") {
    if (webSocket.isConnected()) {
      webSocket.sendTXT("{\"cmd\":\"zero\",\"arm\":\"ALL\"}");
      Serial.println("\n[zero] 서버로 전체 영점 명령 전송");
    } else {
      Serial.println("\n[zero] 서버에 연결되어 있지 않습니다");
    }

  } else if (c == "help" || c == "?") {
    Serial.println(CMD_HELP);

  } else {
    Serial.printf("\n알 수 없는 명령: %s\n", c.c_str());
    Serial.println(CMD_HELP);
  }
}

void handleSerial() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      buf.trim();
      if (buf.length()) runCmd(buf);
      buf = "";
    } else if (buf.length() < 32) {
      buf += ch;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  5) 자세 추정
// ═══════════════════════════════════════════════════════════════
void madgwickUpdate(IMUState &imu, float gx, float gy, float gz,
                    float ax, float ay, float az, float dt, float accRef) {
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
  gx *= PI / 180.0; gy *= PI / 180.0; gz *= PI / 180.0;

  float qDotW = 0.5 * (-qx*gx - qy*gy - qz*gz);
  float qDotX = 0.5 * ( qw*gx + qy*gz - qz*gy);
  float qDotY = 0.5 * ( qw*gy - qx*gz + qz*gx);
  float qDotZ = 0.5 * ( qw*gz + qx*gy - qy*gx);

  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm > 0.0001) {
    ax /= norm; ay /= norm; az /= norm;
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
  imu.qw = qw / qNorm; imu.qx = qx / qNorm; imu.qy = qy / qNorm; imu.qz = qz / qNorm;
}

void readAndUpdate(uint8_t addr, IMUState &imuState,
                   int16_t axO, int16_t ayO, int16_t azO,
                   float &gxO, float &gyO, float &gzO,
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

    float cgx = (gx - gxO) / GYRO_LSB_PER_DPS;
    float cgy = (gy - gyO) / GYRO_LSB_PER_DPS;
    float cgz = (gz - gzO) / GYRO_LSB_PER_DPS;
    float cax = (ax - axO) / 8192.0, cay = (ay - ayO) / 8192.0, caz = (az - azO) / 8192.0;

    float gm = sqrt(cgx*cgx + cgy*cgy + cgz*cgz);
    float am = sqrt(cax*cax + cay*cay + caz*caz);
    if (accRef <= 0.0) accRef = am;
    if (gm < BIAS_GYRO_THR && fabs(am - accRef) < BIAS_ACC_TOL) {
      gxO += (gx - gxO) * BIAS_ALPHA;
      gyO += (gy - gyO) * BIAS_ALPHA;
      gzO += (gz - gzO) * BIAS_ALPHA;
      accRef += (am - accRef) * BIAS_ALPHA * 10.0;
    }

    IMUState tempImu = imuState;
    madgwickUpdate(tempImu, cgx, cgy, cgz, cax, cay, caz, dt, accRef);

    portENTER_CRITICAL(&imuMutex);
    imuState = tempImu;
    portEXIT_CRITICAL(&imuMutex);
  }
}

// ── FreeRTOS IMU 전용 태스크 (Core 1 고정, 100Hz 보장) ──
void imuTask(void * pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  unsigned long lastValidTime = micros();

  for(;;) {
    // 시리얼 명령이 I2C를 쓸 동안 버스를 완전히 놓아준다
    if (imuPauseReq) {
      imuPaused = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      xLastWakeTime = xTaskGetTickCount();  // 재개 시 주기 기준 갱신
      lastValidTime = micros();             // dt 폭주 방지
      continue;
    }
    imuPaused = false;

    unsigned long now = micros();
    float dt = (now - lastValidTime) / 1000000.0f;
    if(dt > 0.05f) dt = 0.01f;
    lastValidTime = now;

    readAndUpdate(MPU1_ADDR, imu1, MPU1_AX_OFFSET, MPU1_AY_OFFSET, MPU1_AZ_OFFSET,
                  MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET, dt, accRef1);

    readAndUpdate(MPU2_ADDR, imu2, MPU2_AX_OFFSET, MPU2_AY_OFFSET, MPU2_AZ_OFFSET,
                  MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET, dt, accRef2);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ═══════════════════════════════════════════════════════════════
//  6) 통신
// ═══════════════════════════════════════════════════════════════
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("웹소켓 서버 연결됨!");
    helloSent = false;              // 접속할 때마다 0점 결과 재보고
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.println("웹소켓 연결 끊김, 재연결 시도 중...");
    helloSent = false;
    return;
  }
  if (type != WStype_TEXT) return;

  // ⭐ [v1.4] 서버 명령 수신. 여기서 바로 실행하면 cal(7초)/drift(60초) 동안
  //    webSocket.loop() 가 멈춰 서버가 이 보드를 끊긴 것으로 본다.
  //    pendingCmd 에 담아두고 loop() 에서 실행한다.
  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, payload, length)) return;   // 방송 등 다른 프레임 무시
  const char* c = doc["cmd"] | "";
  if (c[0] == '\0') return;
  if (!targetMatches(doc["target"] | "")) return;

  if      (!strcmp(c, "boardcal"))   pendingCmd = "cal";
  else if (!strcmp(c, "boardcheck")) pendingCmd = "check";
  else if (!strcmp(c, "boarddrift")) pendingCmd = "drift";
  else if (!strcmp(c, "boardinfo"))  pendingCmd = "info";
  else return;

  Serial.printf("\n[서버 명령 수신] %s\n", pendingCmd.c_str());
}

void sendHello() {
  StaticJsonDocument<512> doc;
  doc["type"]   = "hello";
  doc["board"]  = BOARD_NAME;
  doc["fw"]     = FW_VERSION;
  doc["uptime"] = millis();

  JsonArray arr = doc.createNestedArray("cal");
  const char* ids[2]   = { SENSOR_ID_1, SENSOR_ID_2 };
  CalInfo*    infos[2] = { &cal1, &cal2 };
  for (int i = 0; i < 2; i++) {
    JsonObject o = arr.createNestedObject();
    o["id"]  = ids[i];
    o["gx"]  = round(infos[i]->gx * 10) / 10.0;
    o["gy"]  = round(infos[i]->gy * 10) / 10.0;
    o["gz"]  = round(infos[i]->gz * 10) / 10.0;
    o["sd"]  = round(infos[i]->sd * 1000) / 1000.0;   // 도/초
    o["out"] = round(infos[i]->outPct * 10) / 10.0;   // %
    o["n"]   = infos[i]->n;
    o["ok"]  = infos[i]->ok;
  }

  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
  Serial.println(">>> 자이로 0점 결과를 서버로 보고했습니다 <<<");
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

// ═══════════════════════════════════════════════════════════════
//  7) setup / loop
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // ⚠ 어깨는 선이 길어 100kHz 유지

  setupMPU(MPU1_ADDR);
  setupMPU(MPU2_ADDR);

  if (AUTO_GYRO_CAL) {
    Serial.println("\n자이로 0점 측정 중... 보드를 움직이지 마세요 (약 7초)");
    Serial.println("(전원 인가 직후에는 온도가 오르며 바이어스가 흐릅니다.");
    Serial.println(" 1분쯤 지난 뒤 cal 을 한 번 더 실행하면 더 정확합니다)");
    calibrateGyro(MPU1_ADDR, MPU1_GX_OFFSET, MPU1_GY_OFFSET, MPU1_GZ_OFFSET, cal1, SENSOR_ID_1);
    calibrateGyro(MPU2_ADDR, MPU2_GX_OFFSET, MPU2_GY_OFFSET, MPU2_GZ_OFFSET, cal2, SENSOR_ID_2);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi 연결됨, IP: " + WiFi.localIP().toString());

  webSocket.begin(WS_HOST, WS_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);

  xTaskCreatePinnedToCore(imuTask, "IMU_Task", 4096, NULL, 3, NULL, 1);

  Serial.println(">>> [어깨 순수 6축 v1.4] 준비 완료 <<<");
  Serial.println(CMD_HELP);
}

void loop() {
  handleSerial();
  webSocket.loop();

  // ⭐ [v1.4] 서버에서 받은 명령은 여기서 실행한다 (웹소켓 콜백 밖이라 안전)
  if (pendingCmd.length()) {
    String c = pendingCmd;
    pendingCmd = "";
    runCmd(c);
  }

  unsigned long now = millis();

  if (webSocket.isConnected() && !helloSent) {
    sendHello();
    helloSent = true;
  }

  bool btnPressed = (digitalRead(BOOT_BTN) == LOW && (now - lastBtnTime > 500));
  if (btnPressed && webSocket.isConnected()) {
    lastBtnTime = now;
    webSocket.sendTXT("{\"cmd\":\"zero\",\"arm\":\"ALL\"}");
    Serial.println(">>> 전체 영점 명령 서버로 전송 <<<");
  }

  if (debugPrint && now - lastPrintTime > 200) {
    lastPrintTime = now;
    IMUState temp1, temp2;
    portENTER_CRITICAL(&imuMutex);
    temp1 = imu1; temp2 = imu2;
    portEXIT_CRITICAL(&imuMutex);

    Serial.printf("%s(qw:%.3f qx:%.3f qy:%.3f qz:%.3f) | %s(qw:%.3f qx:%.3f qy:%.3f qz:%.3f)\n",
                  SENSOR_ID_1, temp1.qw, temp1.qx, temp1.qy, temp1.qz,
                  SENSOR_ID_2, temp2.qw, temp2.qx, temp2.qy, temp2.qz);
  }

  if (webSocket.isConnected() && (now - lastSendTime > 100)) {
    lastSendTime = now;

    IMUState temp1, temp2;
    portENTER_CRITICAL(&imuMutex);
    temp1 = imu1; temp2 = imu2;
    portEXIT_CRITICAL(&imuMutex);

    sendQuat(SENSOR_ID_1, temp1.qw, temp1.qx, temp1.qy, temp1.qz);
    sendQuat(SENSOR_ID_2, temp2.qw, temp2.qx, temp2.qy, temp2.qz);
  }

  delay(1);
}
