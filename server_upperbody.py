"""
웨어러블 모션캡처 중앙 서버 v6.0 — 양팔 통합
============================================
왼팔 서버 v5.3 과 오른팔 서버를 하나로 합쳤다. 센서 6개(양팔 각각 어깨/상완/전완)를
한 프로세스에서 받고, 교정과 앵커는 팔마다 완전히 따로 관리한다.

주의: 합칠 때 발견한 것
  업로드된 server_R.py 는 v5.0 이었다. 즉 오른팔 쪽에는 v5.1~v5.3 에서 고친 내용이
  하나도 들어가 있지 않았다. 특히 자세 판정이 옛날 비율 방식(az_side 에 1/3, 2/3,
  뒤 판정 1.45 를 곱하는 것)이라, 왼팔 8/17 데이터에서 옆으로90 187샘플 중 94개를
  '뒤로'로 잘못 판정했던 그 로직이다. 그래서 이 파일은 양쪽 팔 모두 v5.3 로직을
  쓴다. 오른팔도 이제 scaled 기준각 / 히스테리시스 / az_margin 을 전부 쓴다.

팔 사이에서 공유하는 것과 나누는 것
  공유: 웹소켓 포트, 센서 수신 통계, 프리즈 검출
  분리: 교정값(calib_L.json / calib_R.json), 앵커, 자세 판정 이력, 판정 튜닝값

방위각 계산은 좌우와 무관하다. e_lat 을 '옆으로' 자세에서 관측한 방향으로부터
만들기 때문에 az_side 는 팔에 상관없이 양수로 나온다. 화면상 좌우 반전은
수집기/뷰어의 DIAL_SIDE / SIDE 상수가 담당한다.

프로토콜
  센서   {"id":"L_UPPERARM","qw":..,"qx":..,"qy":..,"qz":..}
  보드   {"cmd":"zero","arm":"L"}            src 없음 -> 앵커만 재설정
  UI     {"cmd":"zero","arm":"L","src":"ui"} src 있음 -> 차렷 영점 다시 잡기
  방송   {"v":6,"sensors":[..6..],"arms":{"L":{..},"R":{..}}}
"""

import asyncio
import json
import math
import os
import sys
import time
from collections import deque

try:
    import websockets
except ImportError:
    websockets = None

PORT = 8765

# ── 센서 배치 ──────────────────────────────────────────────────
# 몸통 센서가 하나뿐이라면 두 팔의 "shoulder" 를 같은 id 로 두면 된다.
# 그러면 한 센서가 양팔의 몸통 기준으로 함께 쓰인다. 교정은 팔마다 따로 잡는다.
ARM_SENSORS = {
    "L": {"shoulder": "L_SHOULDER", "upperarm": "L_UPPERARM", "forearm": "L_FOREARM"},
    "R": {"shoulder": "R_SHOULDER", "upperarm": "R_UPPERARM", "forearm": "R_FOREARM"},
}
ARM_NAME = {"L": "왼팔", "R": "오른팔"}
ARM_ORDER = ["L", "R"]
CALIB_FILE = {"L": "calib_L.json", "R": "calib_R.json"}

# ── 동작 설정 (양팔 공통) ──────────────────────────────────────
AUTO_LOAD_CALIB   = False   # 시작 시 이전 교정 적용 안 함. 콘솔 load 로만 적용.
FRESH_SEC         = 0.5
REST_ELEV_DEG     = 25.0    # 이 아래면 REST
ANCHOR_ELEV_DEG   = 18.0    # 앵커는 더 확실히 내려왔을 때만 (히스테리시스)
ANCHOR_HOLD_SEC   = 0.5
ANCHOR_MAX_RATE   = 0.10    # 앵커 보정이 초당 움직일 수 있는 최대 각도
# ── 앵커(요 표류 보정) 설정 ────────────────────────────────────
#
#  앵커는 팔을 내렸을 때 w_ua(=위팔 장축에 수직인 임의 축)의 수평 방위를 재서
#  그 변화를 표류로 간주한다. 그런데 팔을 내리면 위팔 장축이 거의 수직이라
#  w_ua 는 수평면에 눕고, 그 방위는 곧 '위팔의 장축 회전(손바닥 방향)'이 된다.
#  즉 팔을 내릴 때마다 손바닥 방향이 조금씩 달라지면 앵커가 그걸 표류로 착각한다.
#
#  2026-08-18 19시 세션(749샘플)에서 실제로 그 일이 일어났다.
#    앵커θ  -6.6 -> +6.6 -> +10.0  (3.5분간 한 방향으로 16.6도)
#    원본 az 는 추세가 없었다 (앞 -3.0/+5.3/+4.7, 대각 20.7/25.8/22.7,
#                              옆 44.8/54.3/44.5)
#    라벨별 시도 간 편차 합계: 앵커 적용 40.4도 / 원본 23.2도
#    판정 정확도: 앵커 적용 655/749 (최소여유 0.6도) / 앵커 끔 749/749 (4.3도)
#  앵커가 도움이 아니라 방해였다. 그래서 기본을 damped 로 낮추고, 자세 변화로
#  인한 큰 점프는 아예 표류로 인정하지 않으며, 총 보정량에 상한을 둔다.
#
#  off    : 보정하지 않는다. 세션이 짧고(5분 이내) 판정이 흔들리면 이게 가장 안전.
#  damped : 아래 게이트를 모두 통과한 관측만, 느리게, 상한 안에서 반영 (기본)
#  full   : 예전 동작 (게이트 없음, ANCHOR_MAX_RATE 만 적용)
ANCHOR_MODE          = "damped"
ANCHOR_DEADBAND_DEG  = 1.5   # 이보다 작은 차이는 잡음으로 보고 건드리지 않는다
ANCHOR_JUMP_REJECT   = 8.0   # 현재 보정값에서 이만큼 떨어진 관측은 '자세가 바뀐 것'
                             # 으로 보고 버린다 (표류는 이렇게 갑자기 안 생긴다)
ANCHOR_MAX_THETA     = 12.0  # 총 보정량 상한. 여기 닿으면 표류가 아니라 교정 문제다
ANCHOR_MIN_OBS       = 5     # 이만큼 관측이 쌓여야 보정을 시작한다
GRAVITY_EMA_ALPHA = 0.3

FREEZE_N          = 8       # 같은 쿼터니언이 8회(약 0.8초) 반복되면 죽은 센서
STEADY_WINDOW_SEC = 0.5     # 교정 캡처 전 정지 확인 구간
STEADY_MAX_DEG    = 3.0     # 그 구간 내 허용 흔들림
TILT_WARN_DEG     = 25.0    # 몸통 기울기 경고 임계값
REST_MAG_WARN     = 0.30    # 차렷인데 mag가 이보다 크면 교정 불일치
MIN_MAG           = 0.30    # 이보다 작으면 수평 방향 자체가 정의되지 않음
MARGIN_WARN_DEG   = 5.0     # 경계까지 이만큼도 안 남으면 아슬아슬하다고 표시
POSE_HYST_DEG     = 3.0     # 경계 근처 떨림 방지 이력

# 팔꿈치 포락선 여유(도). 교정 오차·잡음으로 클램프가 raw 를 물고 떠는 것을 막는다.
ELBOW_ENVELOPE_TOL = 5.0

# 차렷인데 팔꿈치가 이만큼 굽어 있다고 나오면 팔꿈치 영점이 잘못된 것이다.
FLEX_REST_MIN = 150.0

# ── 자세 판정 튜닝 (팔마다 따로) ───────────────────────────────
#
#  왼팔 실측 2회 (mocap_L_20260817_2258.csv 559샘플 / mocap_L_20260818_1711.csv 558샘플)
#    앞으로90  az  -2.8~+9.0 (평균 3.13)  /  -6.6~-3.8 (평균 -5.30)
#    대각선90  미수집                     /  +23.2~+24.5 (평균 23.60)
#    옆으로90  az +39.1~+47.7 (평균 43.25) / +49.5~+51.6 (평균 50.39)
#
#  두 세션에서 찾은 불변량
#    실제 앞->옆 벌어짐(span)은 교정 캡처 az_side 보다 항상 크다.
#      8/17  span 40.12 / az_side 30.53 = 1.314
#      8/18  span 55.68 / az_side 42.41 = 1.313
#    az_side 자체는 30.5 -> 42.4 로 크게 달랐는데 비율은 같다. 절대 각도를 박아두면
#    다음 세션에 어긋나지만 이 비율은 살아남는다. 대각선은 구간의 정확히 가운데다
#    (8/18 실측 0.519).
#
#  !! 오른팔은 아직 이 검증을 안 했다. 아래 R 값은 왼팔에서 얻은 것을 그대로
#     복사해 둔 것이다. 오른팔로 앞/옆/대각을 한 번 수집해서 span/az_side 를
#     확인하고, 다르면 R 쪽 SPAN_K 만 고치면 된다. 수집기가 CSV 에 az_side 와
#     az_ref 를 남기므로 계산은 바로 된다.
TUNING_DEFAULT = {
    "SPAN_K":     1.250,  # 실제 앞-옆 벌어짐 / 교정 캡처 az_side
                          # !! 이 값은 안정적이지 않다. 3세션 실측이
                          #    8/17 1.314 / 8/18 17시 1.313 / 8/18 19시 1.068
                          #    로 갈렸다. 앞의 두 번이 우연히 일치했을 뿐이다.
                          #    교정 때 얼마나 덜 벌리고 잡았는지에 달린 값이라
                          #    사람 습관에 따라 매번 달라진다.
                          #    1.250 은 3세션 최악 여유가 가장 큰 지점이지만
                          #    (1.3도 -> 2.1도), 근본 해결은 learn 명령이다.
                          #    자세를 실제로 잡고 learn fwd/diag/side 를 입력하면
                          #    이 계수를 아예 거치지 않는다.
    "DIAG_FRAC":  0.52,   # 앞->옆 구간에서 대각선의 위치
    "DIAG_HALF":  0.42,   # 대각 구간이 이웃 기준각 쪽으로 뻗는 비율 (0.5=중간점)
    "DIAG_MIN_WIDTH":  14.0,   # 대각선 구간 최소 폭 (도)
    "BACK_MARGIN_DEG": 25.0,   # 앞 안쪽 / 옆 바깥쪽으로 이만큼 더 나가야 '뒤'
}
TUNING_OVERRIDE = {
    "L": {},
    "R": {},   # 오른팔 실측 후 여기에 {"SPAN_K": ...} 식으로 덮어쓴다
}

# 절대 각도로 고정하고 싶을 때 쓰는 값 (profile abs). 왼팔 8/18 세션 실측 평균.
PROFILE_ABS = {
    "L": {"fwd": -5.3, "diag": 23.6, "side": 50.4},
    "R": {"fwd": -5.3, "diag": 23.6, "side": 50.4},   # 미검증
}
PROFILE_MODE = "scaled"     # scaled | abs | cal

POSE_FWD  = "앞으로 90도"
POSE_DIAG = "대각선 90도"
POSE_SIDE = "옆으로 90도"

# ── 전역 수신 상태 (팔 구분 없음) ──────────────────────────────
latest = {}
connected = set()
gravity_ema = {}
gravity_cache = {}   # sensor_id -> (quat, 평활 결과). 프레임당 1회 갱신 보장
sensor_stat = {}
history = {}          # sensor_id -> deque[(t, quat)]
notes = {"msg": "", "t": 0.0}


def all_sensor_ids():
    ids = []
    for side in ARM_ORDER:
        for key in ("shoulder", "upperarm", "forearm"):
            sid = ARM_SENSORS[side][key]
            if sid not in ids:
                ids.append(sid)
    return ids


SENSOR_ORDER = all_sensor_ids()


def short_id(sid):
    for p in ("L_", "R_"):
        if sid.startswith(p):
            return p[0] + ":" + sid[2:]
    return sid


def note(msg):
    """콘솔 한 줄 알림. 상태 표시줄을 덮어쓰지 않도록 줄바꿈을 앞에 둔다."""
    notes["msg"] = msg
    notes["t"] = time.time()
    sys.stdout.write("\r" + " " * 100 + "\r" + msg + "\n")
    sys.stdout.flush()


# ── 벡터 / 쿼터니언 ────────────────────────────────────────────
def qconj(q): return (q[0], -q[1], -q[2], -q[3])


def qmul(a, b):
    return (
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    )


def qrot(q, v): return qmul(qmul(q, (0.0, v[0], v[1], v[2])), qconj(q))[1:]
def qrel(qa, qb): return qmul(qconj(qa), qb)
def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def scale(v, s): return (v[0]*s, v[1]*s, v[2]*s)
def norm(v): return math.sqrt(dot(v, v))


def unit(v):
    n = norm(v)
    return (0.0, 0.0, 0.0) if n < 1e-12 else (v[0]/n, v[1]/n, v[2]/n)


def perp_to(u):
    a = (1.0, 0.0, 0.0) if abs(u[0]) < 0.9 else (0.0, 1.0, 0.0)
    return unit(sub(a, scale(u, dot(a, u))))


def angle_between(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def wrap180(a): return (a + 180.0) % 360.0 - 180.0
def gravity_in_sensor(q): return qrot(qconj(q), (0.0, 0.0, 1.0))


def gravity_smoothed(sensor_id, q):
    """중력 EMA. 같은 프레임에서 여러 번 불려도 한 번만 진행한다.

    예전에는 한 프레임 안에서 elevation_deg 와 elbow_flex 가 상완 EMA 를 각각
    갱신해서 상완만 두 번(실효 alpha 0.51), 전완은 한 번(0.30) 진행됐다.
    두 마디의 지연이 달라지니 팔을 곧게 편 채로 팔 전체만 들었다 내려도
    |t_fa - t_ua| 가 12도까지 벌어졌고, 그게 아래 포락선에 그대로 들어가
    팔꿈치가 12도 굽은 것처럼 보고됐다.
    """
    c = gravity_cache.get(sensor_id)
    if c is not None and c[0] == q:
        return c[1]
    g = gravity_in_sensor(q)
    prev = gravity_ema.get(sensor_id)
    if prev is not None:
        a = GRAVITY_EMA_ALPHA
        g = (a*g[0] + (1-a)*prev[0], a*g[1] + (1-a)*prev[1], a*g[2] + (1-a)*prev[2])
    gravity_ema[sensor_id] = g
    gravity_cache[sensor_id] = (q, g)
    return g


def clear_gravity(sensor_ids):
    for k in list(sensor_ids):
        gravity_ema.pop(k, None)
        gravity_cache.pop(k, None)


# ── 센서 건강 상태 ─────────────────────────────────────────────
def note_sample(sensor_id, t, quat):
    st = sensor_stat.setdefault(sensor_id, {
        "t": 0.0, "n": 0, "hz": 0.0, "_wt": t, "_wn": 0,
        "first": t, "last_q": None, "same": 0,
    })
    st["t"] = t
    st["n"] += 1
    st["_wn"] += 1

    key = tuple(round(x, 6) for x in quat)
    if st["last_q"] == key:
        st["same"] += 1
    else:
        st["same"] = 0
    st["last_q"] = key

    h = history.setdefault(sensor_id, deque())
    h.append((t, quat))
    while h and t - h[0][0] > STEADY_WINDOW_SEC:
        h.popleft()

    dt = t - st["_wt"]
    if dt >= 1.0:
        st["hz"] = st["_wn"] / dt
        st["_wt"] = t
        st["_wn"] = 0


def is_frozen(sensor_id):
    st = sensor_stat.get(sensor_id)
    return bool(st and st["same"] >= FREEZE_N)


def is_steady(sensor_id, max_deg=STEADY_MAX_DEG):
    """최근 STEADY_WINDOW_SEC 동안 자세가 거의 안 변했는지."""
    h = history.get(sensor_id)
    if not h or len(h) < 3:
        return False, None
    ref = gravity_in_sensor(h[-1][1])
    worst = 0.0
    for _, q in h:
        worst = max(worst, angle_between(ref, gravity_in_sensor(q)))
    return worst <= max_deg, worst


def fresh(sensor_id, now=None):
    """멈춘 센서는 데이터가 있어도 없는 것으로 취급한다."""
    now = now or time.time()
    if is_frozen(sensor_id):
        return None
    d = latest.get(sensor_id)
    if d and (now - d["t"] < FRESH_SEC):
        return (d["qw"], d["qx"], d["qy"], d["qz"])
    return None


def sensor_report(now=None):
    now = now or time.time()
    out = []
    ids = SENSOR_ORDER + [i for i in sorted(sensor_stat) if i not in SENSOR_ORDER]
    for sid in ids:
        st = sensor_stat.get(sid)
        if st is None:
            out.append({"id": sid, "known": True, "ok": False, "frozen": False,
                        "hz": 0.0, "age": None, "n": 0})
            continue
        age = now - st["t"]
        if age > 2.0:
            st["hz"] = 0.0
        frozen = st["same"] >= FREEZE_N
        out.append({"id": sid, "known": sid in SENSOR_ORDER,
                    "ok": (age < FRESH_SEC) and not frozen, "frozen": frozen,
                    "hz": round(st["hz"], 1), "age": round(age, 1), "n": st["n"]})
    return out


def sensor_line(now=None):
    parts = []
    for s in sensor_report(now):
        sid = short_id(s["id"])
        if s["frozen"]:
            parts.append(f"F {sid} 멈춤")
        elif s["ok"]:
            parts.append(f"O {sid} {s['hz']:4.1f}")
        elif s["age"] is None:
            parts.append(f"X {sid} 없음")
        else:
            parts.append(f"! {sid} {s['age']:3.1f}s")
    tail = []
    for side in ARM_ORDER:
        a = arms[side]
        tail.append(f"{side} {a.anchor['theta']:+5.1f}도" if a.ready() else f"{side} 미교정")
    return "  ".join(parts) + " | " + "  ".join(tail)


# ═══════════════════════════════════════════════════════════════
#  팔 하나의 상태
# ═══════════════════════════════════════════════════════════════
class Arm:
    def __init__(self, side):
        self.side = side
        self.name = ARM_NAME[side]
        self.sensors = ARM_SENSORS[side]
        self.tune = dict(TUNING_DEFAULT)
        self.tune.update(TUNING_OVERRIDE.get(side, {}))
        self.calib = {
            "u_ua": None, "u_fa": None, "a_ua": None, "a_fa": None, "w_ua": None,
            "down_sh": None, "e_fwd": None, "e_lat": None,
            "az_side": None, "az_diag": None,
        }
        self.anchor = {"enabled": True, "ref": None, "theta": 0.0, "rate": 0.0,
                       "since": None, "last": None, "prev": None, "bout": [],
                       "n": 0, "rejects": 0, "capped": False}
        self.cal_time = None
        self.saved = None
        self.pose = {"mode": PROFILE_MODE, "abs": dict(PROFILE_ABS[side]),
                     "last": None}
        self.az_hist = deque(maxlen=20)   # learn 명령용 최근 az (약 2초)

    # ── 편의 ──
    def sid(self, key): return self.sensors[key]
    def q(self, key, now=None): return fresh(self.sensors[key], now)
    def arm_axis(self): return self.calib["a_ua"] or self.calib["u_ua"]
    def fore_axis(self): return self.calib["a_fa"] or self.calib["u_fa"]

    def ready(self):
        return self.calib["u_ua"] is not None and self.calib["e_lat"] is not None

    def calib_state(self):
        c = self.calib
        return {"rest": c["u_ua"] is not None, "fwd": c["e_fwd"] is not None,
                "side": c["e_lat"] is not None, "diag": c["az_diag"] is not None,
                "twist": c["a_ua"] is not None, "anchor": self.anchor["ref"] is not None}

    # ── 기하 ──
    def horizontal_azimuth(self, q_sh, q_ua, axis):
        c = self.calib
        if c["e_fwd"] is None or c["e_lat"] is None:
            return None, 0.0
        v = qrot(qrel(q_sh, q_ua), axis)
        down = c["down_sh"]
        h = sub(v, scale(down, dot(v, down)))
        n = norm(h)
        if n < 1e-6:
            return None, n
        h = unit(h)
        return math.degrees(math.atan2(dot(h, c["e_lat"]), dot(h, c["e_fwd"]))), n

    def azimuth_deg(self, q_sh, q_ua):
        az, mag = self.horizontal_azimuth(q_sh, q_ua, self.arm_axis())
        if az is None:
            return None, mag
        return wrap180(az - self.anchor["theta"]), mag

    def elevation_deg(self, q_ua):
        if self.calib["u_ua"] is None:
            return None
        return angle_between(self.calib["u_ua"],
                             gravity_smoothed(self.sid("upperarm"), q_ua))

    def torso_tilt_deg(self, q_sh):
        """어깨(몸통) 센서가 차렷 기준에서 얼마나 기울었는지."""
        if self.calib["down_sh"] is None:
            return None
        return angle_between(gravity_smoothed(self.sid("shoulder"), q_sh),
                             self.calib["down_sh"])

    def elbow_flex(self, q_ua, q_fa):
        """팔꿈치 각도. 180 = 완전히 편 상태, 굽힐수록 줄어든다.

        raw(상대 쿼터니언에서 직접 구한 관절각)가 본래의 측정값이고, 중력으로 만든
        [lo, hi] 포락선은 요 표류로 raw 가 틀어졌을 때를 잡기 위한 안전장치다.

        그런데 포락선을 '평활된' 중력으로 만들면 raw 는 순간값, 포락선은 지연값이
        되어 시간축이 어긋난다. 빠르게 굽혔다 펴면 raw 는 곧바로 0 으로 돌아오는데
        포락선은 아직 굽은 자세를 가리켜, 클램프가 raw 를 덮어쓰고 120~130 도대로
        튀었다가 EMA 가 따라잡으면 170 도로 돌아왔다. 포락선을 순간 중력으로
        바꾸면 raw 와 같은 시점이 되어 이 반동이 사라진다.

        추가로 ELBOW_ENVELOPE_TOL 만큼 포락선을 넓혀, 교정 오차나 잡음으로
        포락선이 raw 를 아슬아슬하게 물고 떠는 것을 막는다.
        """
        c = self.calib
        if c["u_ua"] is None or c["u_fa"] is None:
            return None, 0.0

        # 포락선은 순간 중력으로 (raw 와 시점을 맞춘다)
        t_ua = angle_between(gravity_in_sensor(q_ua), c["u_ua"])
        t_fa = angle_between(gravity_in_sensor(q_fa), c["u_fa"])

        bone_ua = self.arm_axis()
        bone_fa = self.fore_axis()
        fa_in_ua = qrot(qrel(q_ua, q_fa), bone_fa)
        raw = angle_between(bone_ua, fa_in_ua)

        lo = abs(t_fa - t_ua)
        s = t_fa + t_ua
        hi = min(s, 360.0 - s)
        if hi < lo:
            lo, hi = hi, lo

        tol = ELBOW_ENVELOPE_TOL
        drift_min = max(0.0, lo - raw, raw - hi)          # 여유 적용 전 실제 위반량
        lo = max(0.0, lo - tol)
        hi = min(180.0, hi + tol)

        flex = max(lo, min(hi, raw))
        flex_inverted = 180.0 - max(0.0, min(180.0, flex))
        return flex_inverted, drift_min

    # ── 판정 기준각 / 경계 ──
    def pose_refs(self):
        """자세별 기준 방위각(도). (앞, 대각, 옆)

        scaled : 교정 캡처 az_side 에 SPAN_K 를 곱해 실제 벌어짐을 복원 (기본)
        abs    : 절대 각도를 그대로
        cal    : 교정 캡처값을 보정 없이 그대로 (예전 v5.0 방식)
        """
        mode = self.pose["mode"]
        if mode == "abs":
            p = self.pose["abs"]
            return p["fwd"], p["diag"], p["side"]

        s = self.calib["az_side"]
        if s is None or abs(s) < 1e-6:
            return None
        d = self.calib["az_diag"]
        d = d if (d is not None and 0.0 < d / s < 1.0) else None

        if mode == "scaled":
            side = self.tune["SPAN_K"] * s
            diag = (self.tune["SPAN_K"] * d if d is not None
                    else self.tune["DIAG_FRAC"] * side)
            return 0.0, diag, side

        return 0.0, (d if d is not None else 0.5 * s), s

    def bounds(self):
        """도 단위 판정 경계 (앞|대각 , 대각|옆)."""
        r = self.pose_refs()
        if r is None:
            return None
        f, d, s = r
        k = self.tune["DIAG_HALF"]
        lo = d - k * (d - f)
        hi = d + k * (s - d)
        if hi < lo:
            lo, hi = hi, lo
        w = hi - lo
        wmin = self.tune["DIAG_MIN_WIDTH"]
        if w < wmin:
            c = 0.5 * (lo + hi)
            lo, hi = c - wmin / 2.0, c + wmin / 2.0
        return lo, hi

    def outer_bounds(self):
        """'뒤'로 판정하기 시작하는 바깥 경계 (안쪽, 바깥쪽)."""
        r = self.pose_refs()
        if r is None:
            return None
        f, _, s = r
        m = self.tune["BACK_MARGIN_DEG"]
        if s >= f:
            return f - m, s + m
        return s - m, f + m

    def az_margin(self, az):
        """지금 방위각이 가장 가까운 판정 경계에서 몇 도 떨어져 있는지."""
        b, ob = self.bounds(), self.outer_bounds()
        if az is None or b is None or ob is None:
            return None
        return round(min(abs(az - e) for e in (b[0], b[1], ob[0], ob[1])), 1)

    def classify(self, elev, az, mag=1.0):
        """자세 판정. 경계에서 떨리지 않도록 POSE_HYST_DEG 만큼 이력을 준다."""
        if elev is None:
            return "UNKNOWN"
        if elev < REST_ELEV_DEG:
            self.pose["last"] = "REST"
            return "REST"

        b = self.bounds()
        ob = self.outer_bounds()
        if az is None or b is None:
            return "UNKNOWN"
        if mag < MIN_MAG:
            # 팔이 거의 수직이면 수평 성분이 사라져 방위각이 의미를 잃는다.
            return "UNKNOWN"

        lo, hi = b
        back_in, back_out = ob
        prev = self.pose["last"]
        h = POSE_HYST_DEG

        lo_eff = lo + h if prev == POSE_FWD else (lo - h if prev == POSE_DIAG else lo)
        hi_eff = hi - h if prev == POSE_SIDE else (hi + h if prev == POSE_DIAG else hi)
        if lo_eff > hi_eff:
            lo_eff = hi_eff = 0.5 * (lo + hi)

        if az < back_in:
            pose = "뒤/안쪽"
        elif az > back_out:
            pose = "뒤로"
        elif az < lo_eff:
            pose = POSE_FWD
        elif az > hi_eff:
            pose = POSE_SIDE
        else:
            pose = POSE_DIAG

        self.pose["last"] = pose
        return pose

    # ── 앵커 ──
    def clear_anchor(self):
        self.anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                            "last": None, "prev": None, "bout": [],
                            "n": 0, "rejects": 0, "capped": False})

    def update_anchor(self, elev, q_sh, q_ua, now):
        a = self.anchor
        if not a["enabled"] or not self.ready():
            return
        if elev is None or elev > ANCHOR_ELEV_DEG:
            a["since"] = None
            a["bout"] = []
            return
        if a["since"] is None:
            a["since"] = now
            a["bout"] = []
            return
        if now - a["since"] < ANCHOR_HOLD_SEC:
            return

        az_w, mag = self.horizontal_azimuth(q_sh, q_ua, self.calib["w_ua"])
        if az_w is None or mag < 0.5:
            return

        if a["ref"] is None:
            a.update({"ref": az_w, "theta": 0.0, "rate": 0.0,
                      "last": now, "prev": now, "n": 1})
            return

        raw_drift = wrap180(az_w - a["ref"])
        drift_unwrapped = a["theta"] + wrap180(raw_drift - a["theta"])

        a["bout"].append(drift_unwrapped)
        if len(a["bout"]) > 10:
            a["bout"].pop(0)
        vals = sorted(a["bout"])
        target = vals[len(vals) // 2]
        a["n"] += 1
        a["last"] = now

        if ANCHOR_MODE == "off":
            a["rate"] = 0.0
            return

        if ANCHOR_MODE == "damped":
            # 1) 관측이 충분히 쌓이기 전에는 움직이지 않는다
            if len(a["bout"]) < ANCHOR_MIN_OBS:
                a["rate"] = 0.0
                return
            gap = target - a["theta"]
            # 2) 갑자기 크게 벌어진 관측은 표류가 아니라 '팔을 내린 자세가 달라진 것'
            #    이다. 표류는 이렇게 튀지 않는다. 버린다.
            if abs(gap) > ANCHOR_JUMP_REJECT:
                a["rate"] = 0.0
                a["rejects"] = a.get("rejects", 0) + 1
                return
            # 3) 작은 차이는 잡음이다. 쫓아가면 오히려 흔들린다.
            if abs(gap) < ANCHOR_DEADBAND_DEG:
                a["rate"] = 0.0
                return

        # 급변 방지: 팔을 내린 채 위팔을 살짝 돌리기만 해도 w_ua 방위가 크게 튄다.
        dt = now - (a["prev"] or now)
        step = ANCHOR_MAX_RATE * max(dt, 1e-3)
        delta = max(-step, min(step, target - a["theta"]))

        new_theta = a["theta"] + delta
        # 4) 총 보정량 상한. 여기 닿을 만큼 밀렸으면 표류가 아니라 교정이 어긋난 것
        #    이므로, 계속 따라가지 말고 멈추고 사람에게 알린다.
        if ANCHOR_MODE != "full" and abs(new_theta) > ANCHOR_MAX_THETA:
            new_theta = math.copysign(ANCHOR_MAX_THETA, new_theta)
            a["capped"] = True

        a["theta"] = new_theta
        a["rate"] = delta / max(dt, 1e-3)
        a["prev"] = now

    def reset_anchor(self):
        if not self.ready():
            note(f"[거부] {self.name} 앵커: 방향 교정(2 앞 -> 3 옆)이 먼저 필요합니다.")
            return False
        q_sh, q_ua = self.q("shoulder"), self.q("upperarm")
        if not q_sh or not q_ua:
            note(f"[거부] {self.name} 앵커: 어깨 또는 상완 신호가 없습니다.")
            return False
        az_w, mag = self.horizontal_azimuth(q_sh, q_ua, self.calib["w_ua"])
        if az_w is None or mag < 0.5:
            note(f"[거부] {self.name} 앵커: 팔을 내린 상태에서 다시 시도하세요.")
            return False
        self.anchor.update({"ref": az_w, "theta": 0.0, "rate": 0.0, "since": None,
                            "last": time.time(), "prev": time.time(), "bout": [],
                            "n": 1, "rejects": 0, "capped": False})
        note(f"[ok] {self.name} 앵커 재설정. 표류 보정을 0도로 되돌렸습니다.")
        return True

    # ── 교정 ──
    def require(self, keys, need_steady=True):
        qs = {}
        for key in keys:
            sid = self.sid(key)
            if is_frozen(sid):
                note(f"[거부] {sid} 값이 멈췄습니다. 보드 전원과 WiFi를 확인하세요.")
                return False, None
            q = fresh(sid)
            if q is None:
                note(f"[거부] {sid} 신호가 없습니다.")
                return False, None
            if need_steady:
                ok, worst = is_steady(sid)
                if not ok:
                    w = "?" if worst is None else f"{worst:.1f}도"
                    note(f"[거부] {sid} 가 움직이는 중입니다 (흔들림 {w}). "
                         f"자세를 멈추고 다시 누르세요.")
                    return False, None
            qs[key] = q
        return True, qs

    def capture_rest(self):
        ok, qs = self.require(["shoulder", "upperarm"])
        if not ok:
            return False
        q_sh, q_ua = qs["shoulder"], qs["upperarm"]
        q_fa = self.q("forearm")
        c = self.calib

        c["u_ua"] = unit(gravity_in_sensor(q_ua))
        c["a_ua"] = c["a_fa"] = None
        c["w_ua"] = perp_to(c["u_ua"])
        c["down_sh"] = unit(gravity_in_sensor(q_sh))
        c["u_fa"] = unit(gravity_in_sensor(q_fa)) if q_fa else None
        c["e_fwd"] = c["e_lat"] = c["az_side"] = c["az_diag"] = None
        clear_gravity(self.sensors.values())
        self.cal_time = None
        self.clear_anchor()
        self.pose["last"] = None
        note(f"[ok] {self.name} 1 차렷 영점 완료. 방향 교정이 초기화되었습니다. "
             f"이어서 2 앞 -> 3 옆 -> 4 대각.")
        return True

    def capture_elbow_zero(self):
        ok, qs = self.require(["upperarm", "forearm"])
        if not ok:
            return False
        if self.calib["u_ua"] is None:
            note(f"[거부] {self.name} 팔꿈치: 1 차렷 영점이 먼저 필요합니다.")
            return False
        q_rel = qrel(qs["upperarm"], qs["forearm"])
        self.calib["u_fa"] = qrot(qconj(q_rel), self.calib["u_ua"])
        self.calib["a_fa"] = qrot(qconj(q_rel), self.arm_axis())
        self.save_calib()
        note(f"[ok] {self.name} 팔꿈치 영점 완료. 지금 자세가 완전 펴짐(0도 굽힘)입니다.")
        return True

    def capture_dir(self, kind):
        c = self.calib
        if c["u_ua"] is None:
            note(f"[거부] {self.name} 방향 교정: 1 차렷 영점이 먼저 필요합니다.")
            return False
        ok, qs = self.require(["shoulder", "upperarm"])
        if not ok:
            return False
        q_sh, q_ua = qs["shoulder"], qs["upperarm"]

        tilt = self.torso_tilt_deg(q_sh)
        if tilt is not None and tilt > TILT_WARN_DEG:
            note(f"[거부] {self.name} 몸통 기준이 {tilt:.0f}도 기울었습니다. 어깨 센서가 "
                 f"팔을 따라 움직이고 있는지 확인하세요. 몸통에 고정해야 합니다.")
            return False

        d = unit(qrot(qrel(q_sh, q_ua), self.arm_axis()))
        down = c["down_sh"]
        h = sub(d, scale(down, dot(d, down)))
        mag = norm(h)
        if mag < 0.35:
            note(f"[거부] {self.name} 팔이 충분히 올라가지 않았습니다 (mag {mag:.2f}). "
                 f"90도로 들고 다시.")
            return False
        h = unit(h)

        if kind == "forward":
            if c["e_lat"] is not None and c["az_side"] is not None:
                new_lat = unit(sub(c["e_lat"], scale(h, dot(c["e_lat"], h))))
                if norm(new_lat) < 1e-6:
                    note(f"[거부] {self.name} 앞 방향 재설정 실패. 옆 방향과 너무 가깝습니다.")
                    return False
                c["e_fwd"] = h
                c["e_lat"] = new_lat
                self.clear_anchor()
                self.cal_time = time.time()
                self.save_calib()
                note(f"[ok] {self.name} 2 앞 방향 재설정 완료.")
                return True
            c["e_fwd"] = h
            c["e_lat"] = c["az_side"] = c["az_diag"] = None
            note(f"[ok] {self.name} 2 앞 방향 저장. 이어서 3 옆 방향을 잡으세요.")
            return True

        if c["e_fwd"] is None:
            note(f"[거부] {self.name} 2 앞 방향을 먼저 잡으세요.")
            return False

        if kind == "side":
            lat = unit(sub(h, scale(c["e_fwd"], dot(h, c["e_fwd"]))))
            if norm(lat) < 1e-6:
                note(f"[거부] {self.name} 옆 방향이 앞 방향과 구분되지 않습니다.")
                return False
            c["e_lat"] = lat
            s = math.degrees(math.atan2(dot(h, lat), dot(h, c["e_fwd"])))
            c["az_side"] = s
            c["az_diag"] = None
            self.clear_anchor()
            self.cal_time = time.time()
            self.save_calib()

            # 캡처값은 실제 녹화 자세보다 늘 덜 벌어져 있다(왼팔 2세션 모두 계수 1.31).
            b = self.bounds()
            if abs(s) < 20.0:
                note(f"[경고] {self.name} 앞과 옆의 벌어짐이 {abs(s):.0f}도뿐입니다. "
                     f"두 자세를 더 확실히 구분해서 다시 잡는 편이 좋습니다.")
            else:
                note(f"[ok] {self.name} 3 옆 방향 완료. 캡처 벌어짐 {s:.0f}도 "
                     f"-> 판정 기준 옆 {self.tune['SPAN_K'] * s:.0f}도.\n"
                     f"       판정구간 앞 < {b[0]:.0f} < 대각 < {b[1]:.0f} < 옆. "
                     f"팔을 내리고 1초 정지하세요.")
            return True

        if kind == "diag":
            if c["e_lat"] is None:
                note(f"[거부] {self.name} 3 옆 방향을 먼저 잡으세요.")
                return False
            dg = math.degrees(math.atan2(dot(h, c["e_lat"]), dot(h, c["e_fwd"])))
            f = dg / c["az_side"] if c["az_side"] else 0.0
            if not (0.0 < f < 1.0):
                note(f"[거부] {self.name} 대각선이 앞과 옆 사이에 있지 않습니다 "
                     f"(비율 {f:.2f}). 앞과 옆의 중간 자세로 다시 잡으세요.")
                return False
            c["az_diag"] = dg
            self.save_calib()
            b = self.bounds()
            note(f"[ok] {self.name} 4 대각선 완료. 판정 경계 {b[0]:.0f}도 / {b[1]:.0f}도.")
            return True
        return False

    # ── 교정 파일 ──
    def save_calib(self):
        try:
            payload = {k: (list(v) if isinstance(v, tuple) else v)
                       for k, v in self.calib.items()}
            payload["_arm"] = self.side
            payload["_saved_at"] = time.time()
            payload["_saved_str"] = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(CALIB_FILE[self.side], "w") as f:
                json.dump(payload, f, indent=2)
        except OSError as e:
            note(f"[경고] {self.name} 교정 저장 실패: {e}")

    def read_calib_file(self):
        path = CALIB_FILE[self.side]
        if not os.path.exists(path):
            return None
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def apply_calib(self, d):
        for k in self.calib:
            v = d.get(k)
            self.calib[k] = tuple(v) if isinstance(v, list) else v
        if self.calib["u_ua"] is not None:
            self.calib["w_ua"] = perp_to(self.arm_axis())
        self.cal_time = time.time()
        self.clear_anchor()
        self.pose["last"] = None
        clear_gravity(self.sensors.values())

    def load_calib(self):
        d = self.saved or self.read_calib_file()
        if not d:
            note(f"[거부] {self.name} 불러올 교정 파일이 없습니다.")
            return False
        self.apply_calib(d)
        when = d.get("_saved_str", "시각 불명")
        note(f"[ok] {self.name} {when} 에 저장된 교정을 적용했습니다. "
             f"센서를 다시 붙였다면 이 값은 쓰면 안 됩니다.")
        return True

    def reset_calib(self):
        for k in self.calib:
            self.calib[k] = None
        self.cal_time = None
        self.clear_anchor()
        self.pose["last"] = None
        clear_gravity(self.sensors.values())
        note(f"[ok] {self.name} 교정을 전부 지웠습니다. 1 차렷 영점부터 다시 시작하세요.")

    # ── 분석 ──
    def analyze(self, now, frozen_ids):
        q_sh = self.q("shoulder", now)
        q_ua = self.q("upperarm", now)
        q_fa = self.q("forearm", now)

        b, ob, r = self.bounds(), self.outer_bounds(), self.pose_refs()
        out = {
            "arm": self.side, "name": self.name,
            "status": "OK", "pose": "UNKNOWN", "elev": 0.0, "az": 0.0, "mag": 0.0,
            "flex": None, "flex_drift": 0.0,
            "az_side": self.calib["az_side"], "az_diag": self.calib["az_diag"],
            "bound_lo": (None if b is None else round(b[0], 1)),
            "bound_hi": (None if b is None else round(b[1], 1)),
            "bound_back": (None if ob is None else round(ob[0], 1)),
            "bound_far": (None if ob is None else round(ob[1], 1)),
            "az_ref": (None if r is None else
                       {"fwd": round(r[0], 1), "diag": round(r[1], 1),
                        "side": round(r[2], 1)}),
            "profile": self.pose["mode"],
            "az_margin": None, "sh_tilt": None,
            "drift": round(self.anchor["theta"], 1) if self.ready() else None,
            "drift_rate": round(self.anchor["rate"], 2) if self.ready() else None,
            "anchor_age": (None if self.anchor["last"] is None
                           else round(now - self.anchor["last"])),
            "anchor_n": self.anchor["n"],
            "anchor_mode": ANCHOR_MODE,
            "anchor_rejects": self.anchor.get("rejects", 0),
            "anchor_capped": bool(self.anchor.get("capped")),
            "auto_anchor": self.anchor["enabled"],
            "cal_age": (None if self.cal_time is None else round(now - self.cal_time)),
            "calib": self.calib_state(), "quat": {}, "warn": [],
        }

        mine = [s for s in frozen_ids if s in self.sensors.values()]
        if mine:
            out["warn"].append("frozen")
            out["status"] = "값이 멈춘 센서: " + ", ".join(mine)
            return out

        missing = [k for k, v in (("어깨", q_sh), ("상완", q_ua)) if v is None]
        if missing:
            out["status"] = "센서 대기: " + ", ".join(missing)
            return out

        if self.calib["u_ua"] is None:
            out["status"] = "1 차렷 영점이 필요합니다"
            return out

        out["quat"] = {"sh": list(q_sh), "ua": list(q_ua),
                       "fa": list(q_fa) if q_fa else None}

        elev = self.elevation_deg(q_ua)
        tilt = self.torso_tilt_deg(q_sh)
        if tilt is not None:
            out["sh_tilt"] = round(tilt, 1)

        self.update_anchor(elev, q_sh, q_ua, now)
        az, mag = self.azimuth_deg(q_sh, q_ua)
        out["elev"] = round(elev, 2)
        out["mag"] = round(mag, 3)
        if az is not None:
            out["az"] = round(az, 2)
        out["drift"] = round(self.anchor["theta"], 1) if self.ready() else None

        if q_fa and self.calib["u_fa"]:
            flex, fdrift = self.elbow_flex(q_ua, q_fa)
            out["flex"] = round(flex, 2)
            out["flex_drift"] = round(fdrift, 2)

        if not self.ready():
            out["pose"] = "REST" if elev < REST_ELEV_DEG else "UNKNOWN"
            out["status"] = "방향 교정이 필요합니다 (2 앞 -> 3 옆)"
            return out

        out["pose"] = self.classify(elev, az, mag)
        if elev >= REST_ELEV_DEG and mag >= MIN_MAG:
            out["az_margin"] = self.az_margin(az)
            self.az_hist.append(az)
            if out["az_margin"] is not None and out["az_margin"] < MARGIN_WARN_DEG:
                out["warn"].append("near_bound")

        # 팔꿈치 영점이 제대로 안 잡혔는지. 차렷이면 팔은 대개 펴져 있다.
        if (elev < REST_ELEV_DEG and out["flex"] is not None
                and out["flex"] < FLEX_REST_MIN):
            out["warn"].append("elbow_zero")
            out["status"] = (f"차렷인데 팔꿈치가 {180 - out['flex']:.0f}도 굽은 것으로 "
                             f"나옵니다. '팔꿈치 영점'을 팔을 편 상태에서 다시 잡으세요.")
            return out

        # 교정이 이 세션 것이 맞는지 실시간 확인
        if elev < REST_ELEV_DEG and mag > REST_MAG_WARN:
            out["warn"].append("stale_calib")
            out["status"] = (f"차렷 자세인데 mag가 {mag:.2f}입니다. 교정이 지금 센서 "
                             f"부착 상태와 맞지 않습니다. reset 후 다시 잡으세요.")
            return out

        if tilt is not None and tilt > TILT_WARN_DEG:
            out["warn"].append("torso_tilt")
            out["status"] = (f"몸통 기준이 {tilt:.0f}도 기울었습니다. 어깨 센서가 팔을 "
                             f"따라 움직이면 앞/옆 판정이 어긋납니다.")
            return out

        if self.anchor["ref"] is None:
            out["status"] = "팔을 내리고 1초 정지하면 표류 앵커가 잡힙니다"
        elif self.anchor.get("capped"):
            out["warn"].append("anchor_capped")
            out["status"] = (f"앵커 보정이 상한 {ANCHOR_MAX_THETA:.0f}도에 닿았습니다. "
                             f"이건 표류가 아니라 교정이 어긋난 것입니다. "
                             f"'2 앞으로'를 다시 잡으세요.")
        return out


arms = {side: Arm(side) for side in ARM_ORDER}
active = {"arm": "L"}


def analyze():
    now = time.time()
    sensors = sensor_report(now)
    frozen_ids = [s["id"] for s in sensors if s["frozen"]]
    return {
        "v": 6,
        "t": round(now, 3),
        "sensors": sensors,
        "active": active["arm"],
        "arms": {side: arms[side].analyze(now, frozen_ids) for side in ARM_ORDER},
    }


# ═══════════════════════════════════════════════════════════════
#  명령 라우팅
# ═══════════════════════════════════════════════════════════════
def pick_arms(token):
    """'l' / 'r' / 'both' / '' -> 대상 팔 목록. 없으면 활성 팔."""
    t = (token or "").strip().lower()
    if t in ("l", "left", "왼", "왼팔"):
        return ["L"], True
    if t in ("r", "right", "오른", "오른팔"):
        return ["R"], True
    if t in ("both", "all", "양팔", "둘"):
        return list(ARM_ORDER), True
    return [active["arm"]], False


def run_cmd(cmd, arg="", src="console"):
    """cmd 는 소문자. arg 첫 토큰이 팔 지정이면 소비한다."""
    parts = arg.split()
    head = parts[0] if parts else ""
    targets, consumed = pick_arms(head)
    rest = " ".join(parts[1:] if consumed else parts)

    if cmd == "arm":
        if consumed and len(targets) == 1:
            active["arm"] = targets[0]
        note(f"[ok] 활성 팔: {ARM_NAME[active['arm']]} "
             f"(명령 뒤에 L / R / both 를 붙여 그때그때 지정할 수도 있습니다)")
        return True

    if cmd == "profile":
        for side in targets:
            set_profile(arms[side], rest)
        return True

    if cmd == "learn":
        for side in targets:
            learn_ref(arms[side], rest)
        return True

    if cmd == "anchormode":
        global ANCHOR_MODE
        m = rest.strip().lower()
        if m in ("off", "damped", "full"):
            ANCHOR_MODE = m
            note(f"[ok] 앵커 모드 -> {m}")
        elif m:
            note("사용법: anchormode [off|damped|full]")
        for side in ARM_ORDER:
            a = arms[side].anchor
            note(f"  {ARM_NAME[side]} 모드 {ANCHOR_MODE}  보정 {a['theta']:+.1f}도  "
                 f"관측 {a['n']}회  거부 {a.get('rejects', 0)}회"
                 f"{'  [상한 도달]' if a.get('capped') else ''}")
        return True

    if cmd == "tune":
        for side in targets:
            set_tune(arms[side], rest)
        return True

    if cmd == "status":
        print_status(targets)
        return True

    table = {
        "zero":   lambda a: a.capture_rest(),
        "fwd":    lambda a: a.capture_dir("forward"),
        "side":   lambda a: a.capture_dir("side"),
        "diag":   lambda a: a.capture_dir("diag"),
        "elbow":  lambda a: a.capture_elbow_zero(),
        "anchor": lambda a: a.reset_anchor(),
        "reset":  lambda a: a.reset_calib(),
        "save":   lambda a: a.save_calib(),
        "load":   lambda a: a.load_calib(),
    }
    fn = table.get(cmd)
    if fn is None:
        return False
    for side in targets:
        fn(arms[side])
    return True


HELP = ("명령 (뒤에 L / R / both 를 붙이면 그 팔에만 적용, 없으면 활성 팔)\n"
        "  arm L|R        활성 팔 바꾸기\n"
        "  zero fwd side diag    교정 4단계\n"
        "  elbow anchor reset save load\n"
        "  status         현재 상태\n"
        "  anchormode [off|damped|full]   표류 보정 방식\n"
        "  learn fwd|diag|side   지금 자세를 그 자세의 기준각으로 저장\n"
        "  profile [scaled|abs|cal|<앞> <대각> <옆>]\n"
        "  tune [SPAN_K=1.313 DIAG_HALF=0.42 BACK_MARGIN_DEG=25 ...]")


def learn_ref(a, arg=""):
    """지금 취하고 있는 실제 자세의 az 를 그 자세의 기준각으로 저장한다.

    교정 캡처(az_side)는 늘 실제 녹화 자세보다 덜 벌어져 있고, 그 비율(SPAN_K)이
    세션마다 다르다. 3세션 실측이 1.314 / 1.313 / 1.068 로 갈렸다. 즉 고정 계수로는
    맞출 수 없다. 이 명령은 계수를 거치지 않고 실제 자세에서 직접 읽는다.

      사용법: 그 자세를 잡고 멈춘 상태에서
              learn fwd     /  learn diag  /  learn side
              learn L side  처럼 팔을 지정할 수도 있다.
    세 자세를 다 잡으면 자동으로 abs 모드로 바뀐다.
    """
    key = {"fwd": "fwd", "forward": "fwd", "앞": "fwd",
           "diag": "diag", "대각": "diag",
           "side": "side", "옆": "side"}.get(arg.strip().lower())
    if key is None:
        note("사용법: learn [L|R|both] fwd|diag|side   (그 자세를 잡고 멈춘 뒤 입력)")
        return False
    if not a.ready():
        note(f"[거부] {a.name} learn: 방향 교정(1~3)이 먼저 필요합니다.")
        return False

    vals = list(a.az_hist)
    if len(vals) < 10:
        note(f"[거부] {a.name} learn: 유효한 az 표본이 부족합니다 ({len(vals)}개). "
             f"팔을 들어 자세를 잡은 채로 1초쯤 기다렸다가 다시 입력하세요.")
        return False
    vals.sort()
    n = len(vals)
    med = vals[n // 2]
    # 흔들림은 최소/최대 대신 10~90 백분위로 본다. 자세를 잡는 순간의
    # 표본 한두 개가 섞여도 무너지지 않는다.
    spread = vals[int(0.9 * (n - 1))] - vals[int(0.1 * (n - 1))]
    if spread > 6.0:
        note(f"[거부] {a.name} learn: 최근 az 가 {spread:.0f}도나 흔들립니다. "
             f"자세를 멈추고 다시 입력하세요.")
        return False

    a.pose["abs"][key] = round(med, 1)
    a.pose["mode"] = "abs"
    p = a.pose["abs"]
    b = a.bounds()
    note(f"[ok] {a.name} '{key}' 기준각 = {med:.1f}도 (표본 {len(vals)}개, 흔들림 {spread:.1f}도)\n"
         f"       기준각 앞 {p['fwd']:.1f} / 대각 {p['diag']:.1f} / 옆 {p['side']:.1f}\n"
         f"       판정구간 앞 < {b[0]:.1f} < 대각 < {b[1]:.1f} < 옆")
    return True

def set_profile(a, arg=""):
    """기준각 모드 보기/바꾸기."""
    p = arg.split()
    if p and p[0] in ("scaled", "abs", "cal"):
        a.pose["mode"] = p[0]
        if p[0] == "abs":
            a.pose["abs"] = dict(PROFILE_ABS[a.side])
        note(f"[ok] {a.name} 기준각 모드 -> {p[0]}")
    elif len(p) == 3:
        try:
            f, dg, s = (float(x) for x in p)
        except ValueError:
            note("[거부] 숫자 세 개를 주세요. 예: profile L -5.3 23.6 50.4")
            return
        a.pose["abs"] = {"fwd": f, "diag": dg, "side": s}
        a.pose["mode"] = "abs"
        note(f"[ok] {a.name} 기준각 고정: 앞 {f} / 대각 {dg} / 옆 {s} 도")
    elif p:
        note("사용법: profile [L|R|both] [scaled|abs|cal|<앞> <대각> <옆>]")
        return

    r, b, ob = a.pose_refs(), a.bounds(), a.outer_bounds()
    if r is None or b is None:
        note(f"  {a.name}: 기준각 없음 (교정 필요)")
        return
    note(f"  {a.name} 모드 {a.pose['mode']} (캡처 az_side "
         f"{'-' if a.calib['az_side'] is None else round(a.calib['az_side'], 1)})\n"
         f"    기준각  앞 {r[0]:.1f} / 대각 {r[1]:.1f} / 옆 {r[2]:.1f} 도\n"
         f"    판정구간 앞 < {b[0]:.1f} < 대각 < {b[1]:.1f} < 옆  "
         f"(뒤 {ob[0]:.1f} 미만 / {ob[1]:.1f} 초과, 표시폭 {ob[1] - ob[0]:.0f}도)")


def set_tune(a, arg=""):
    """판정 튜닝값 보기/바꾸기. 예: tune R SPAN_K=1.28"""
    changed = []
    for tok in arg.split():
        if "=" not in tok:
            continue
        k, _, v = tok.partition("=")
        k = k.upper()
        if k not in a.tune:
            note(f"[거부] 모르는 항목: {k}. 가능: {', '.join(sorted(a.tune))}")
            return
        try:
            a.tune[k] = float(v)
            changed.append(f"{k}={a.tune[k]}")
        except ValueError:
            note(f"[거부] {k} 값이 숫자가 아닙니다: {v}")
            return
    if changed:
        note(f"[ok] {a.name} 튜닝 변경: {', '.join(changed)}")
    vals = "  ".join(f"{k}={v}" for k, v in a.tune.items())
    b = a.bounds()
    tail = "" if b is None else f"   -> 경계 {b[0]:.1f} / {b[1]:.1f}"
    note(f"  {a.name}  {vals}{tail}")


def print_status(targets=None):
    d = analyze()
    lines = [""]
    lines.append("  센서   " + sensor_line())
    for side in (targets or ARM_ORDER):
        x = d["arms"][side]
        r = x["az_ref"]
        src = {"scaled": "az_side x SPAN_K", "abs": "절대 프로파일",
               "cal": "교정 캡처값"}.get(x["profile"], x["profile"])
        mark = " *" if side == active["arm"] else "  "
        lines += [
            f"{mark}[{x['name']}] {x['status']}",
            f"    판정  {x['pose']}   여유 {x['az_margin']}도",
            f"    elev {x['elev']}도  az {x['az']}도  mag {x['mag']}  "
            f"몸통기울기 {x['sh_tilt']}도  flex {x['flex']}도",
            f"    기준각 {src}  앞 {r and r['fwd']} / 대각 {r and r['diag']} / "
            f"옆 {r and r['side']} 도  (캡처 az_side {x['az_side']})",
            f"    판정구간 앞 < {x['bound_lo']} < 대각 < {x['bound_hi']} < 옆  "
            f"(뒤 {x['bound_back']} 미만 / {x['bound_far']} 초과)",
            f"    앵커 {x['anchor_mode']}  보정 {x['drift']}도  관측 {x['anchor_n']}회  "
            f"거부 {x['anchor_rejects']}회{'  [상한]' if x['anchor_capped'] else ''}",
            f"    교정 {x['calib']}",
        ]
    note("\n".join(lines))


# ═══════════════════════════════════════════════════════════════
#  통신
# ═══════════════════════════════════════════════════════════════
async def handler(ws):
    connected.add(ws)
    try:
        async for message in ws:
            try:
                data = json.loads(message)
            except ValueError:
                continue

            if "id" in data and "qw" in data:
                t = time.time()
                data["t"] = t
                latest[data["id"]] = data
                note_sample(data["id"], t,
                            (data["qw"], data["qx"], data["qy"], data["qz"]))
                continue

            cmd = data.get("cmd")
            if not cmd:
                continue
            arm_tok = data.get("arm", "")

            if cmd == "zero" and not data.get("src"):
                # 보드 BOOT 버튼. src 가 없으면 보드에서 온 것으로 본다.
                # 방향 교정을 지우면 안 되므로 앵커 재설정으로만 처리한다.
                targets, _ = pick_arms(arm_tok)
                for side in targets:
                    note(f"[보드 버튼] {ARM_NAME[side]} 앵커만 재설정합니다. "
                         f"차렷 영점을 다시 잡으려면 UI에서 1번을 누르세요.")
                    arms[side].reset_anchor()
                continue

            if cmd == "cal":
                pose = data.get("pose", "forward")
                run_cmd({"forward": "fwd", "side": "side", "diag": "diag"}[pose],
                        arm_tok)
                continue

            run_cmd(cmd, arm_tok, src="ui")
    finally:
        connected.discard(ws)


async def broadcast_loop():
    while True:
        msg = json.dumps(analyze())
        if connected:
            await asyncio.gather(*(w.send(msg) for w in connected),
                                 return_exceptions=True)
        await asyncio.sleep(0.1)


async def sensor_status_loop():
    while True:
        sys.stdout.write("\r" + sensor_line().ljust(100)[:100])
        sys.stdout.flush()
        await asyncio.sleep(0.5)


async def console_loop():
    loop = asyncio.get_event_loop()
    while True:
        line = (await loop.run_in_executor(None, sys.stdin.readline)).strip()
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        if not run_cmd(cmd.lower(), arg.strip()):
            note(HELP)


async def main():
    print("웨어러블 모션캡처 서버 v6.0 (양팔 / 6축)")
    print(f"포트 {PORT}   센서 {', '.join(SENSOR_ORDER)}")
    if ARM_SENSORS["L"]["shoulder"] == ARM_SENSORS["R"]["shoulder"]:
        print("몸통 센서 1개를 양팔이 공유하는 설정입니다.")

    for side in ARM_ORDER:
        a = arms[side]
        d = a.read_calib_file()
        a.saved = d
        if d and AUTO_LOAD_CALIB:
            a.apply_calib(d)
            print(f"  {a.name}: 이전 교정 적용됨 ({d.get('_saved_str', '시각 불명')})")
        elif d:
            print(f"  {a.name}: 저장된 교정 있음 "
                  f"({d.get('_saved_str', '시각 불명')}) — 적용하지 않았습니다. "
                  f"'load {side}' 로만 적용됩니다.")
        else:
            print(f"  {a.name}: 저장된 교정 없음")
    print("\n순서(팔마다): zero -> fwd -> side -> diag, 그 뒤 팔 내리고 1초 정지")
    print(HELP + "\n")

    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.gather(broadcast_loop(), sensor_status_loop(), console_loop())


if __name__ == "__main__":
    if websockets is None:
        print("pip install websockets 가 필요합니다.")
    else:
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            print("\n종료합니다.")
