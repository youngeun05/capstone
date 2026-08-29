"""
웨어러블 모션캡처 중앙 서버 v6.1 — 양팔 통합 + 보드 원격 진단
============================================================
센서 6개(양팔 각각 어깨/상완/전완)를 한 프로세스에서 받고,
교정과 앵커는 팔마다 완전히 따로 관리한다.

프로토콜
  센서   {"id":"L_UPPERARM","qw":..,"qx":..,"qy":..,"qz":..}
  보드   {"cmd":"zero","arm":"L"}            src 없음 -> 앵커만 재설정
  UI     {"cmd":"zero","arm":"L","src":"ui"} src 있음 -> 차렷 영점 다시 잡기
  방송   {"v":6,"sensors":[..6..],"arms":{"L":{..},"R":{..}}}   ← 뷰어에게만

  [v6.1] 서버 -> 보드
         {"cmd":"boardcal","target":"ALL"}     자이로 0점 재측정
         {"cmd":"boardcheck","target":"ALL"}   흔들림/잔여 바이어스 진단
         {"cmd":"boarddrift","target":"ALL"}   30초 실측 표류
         target 은 "ALL" 또는 보드 이름(SHOULDER_BOARD / ARM_BOARD_L / ARM_BOARD_R)

         보드 -> 서버
         {"type":"hello","board":..,"fw":..,"cal":[{"id":..,"sd":..,"out":..,
                                                    "n":..,"ok":..}]}
         {"type":"diag","board":..,"kind":"check"|"drift",
          "d":[{"id":..,"sd":..,"bias":..,"out":..,"drift":..}]}

────────────────────────────────────────────────────────────────
v6.1 변경점
  [FIX-A] 보드에게 analyze() 프레임을 보내지 않는다.
          v6.0 은 broadcast_loop 가 connected 전체에 보냈다. 보드 펌웨어는
          WStype_TEXT 를 처리하지 않으므로 순수한 낭비였고, 센서 6개 프레임은
          3KB 에 가까워 보드 3대면 초당 90KB 를 헛되이 실어보냈다.
          하체 서버(v1.3)가 이미 쓰던 board_sockets 방식을 그대로 가져왔다.

  [FIX-B] 보드 자이로 0점을 서버 콘솔에서 실행하고 결과를 판정한다.
          보드는 원래 cal 직후 sendHello() 로 결과를 보내고 있었는데
          서버가 그걸 버리고 있었다. 이제 저장해두고 표로 보여준다.
            bcal   [L|R|SH|all]   자이로 0점 재측정 -> 흔들림/이상치 판정
            bcheck [L|R|SH|all]   오프셋 유지한 채 잔여 바이어스 진단
            bdrift [L|R|SH|all]   30초 적분해 실제 표류량 측정
            boards                마지막 결과를 다시 표로 출력

          ⚠ 보드 펌웨어에 웹소켓 명령 수신부가 있어야 동작한다.
            (esp32_*_v1.4 이상. v1.3 은 WStype_TEXT 를 무시한다)

  [FIX-C] 콘솔이 stdin 없이 실행돼도 죽지 않는다 (하체 서버 FIX-2 와 동일).
────────────────────────────────────────────────────────────────
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
ARM_SENSORS = {
    "L": {"shoulder": "L_SHOULDER", "upperarm": "L_UPPERARM", "forearm": "L_FOREARM"},
    "R": {"shoulder": "R_SHOULDER", "upperarm": "R_UPPERARM", "forearm": "R_FOREARM"},
}
ARM_NAME = {"L": "왼팔", "R": "오른팔"}
ARM_ORDER = ["L", "R"]
CALIB_FILE = {"L": "calib_L.json", "R": "calib_R.json"}

# ── [v6.1] 보드 진단 ───────────────────────────────────────────
# 보드 이름 -> 사람이 읽는 이름. 펌웨어의 BOARD_NAME 과 일치해야 한다.
BOARD_NAME_KR = {
    "SHOULDER_BOARD": "어깨 보드",
    "ARM_BOARD_L": "왼팔 보드",
    "ARM_BOARD_R": "오른팔 보드",
}
# 콘솔에서 쓰는 짧은 별명 -> BOARD_NAME
BOARD_ALIAS = {
    "sh": "SHOULDER_BOARD", "shoulder": "SHOULDER_BOARD", "어깨": "SHOULDER_BOARD",
    "l": "ARM_BOARD_L", "left": "ARM_BOARD_L", "왼": "ARM_BOARD_L",
    "왼팔": "ARM_BOARD_L",
    "r": "ARM_BOARD_R", "right": "ARM_BOARD_R", "오른": "ARM_BOARD_R",
    "오른팔": "ARM_BOARD_R",
}

# 통과 기준. 여기 숫자만 바꾸면 판정이 바뀐다.
BOARD_SD_MAX_DPS    = 0.50   # cal 중 흔들림 (도/초). 이 아래여야 0점이 유효
BOARD_OUT_MAX_PCT   = 0.0    # 읽기 오류 비율 (%). v1.3 펌웨어면 0.0 이 정상값
BOARD_BIAS_MAX_DPS  = 0.50   # 잔여 바이어스 (도/초, bcheck)
BOARD_DRIFT_MAX_DPM = 10.0   # 실측 표류 (도/분, bdrift)

BOARD_CAL_WAIT_SEC   = 14.0  # bcal 후 결과를 기다리는 시간
BOARD_CHECK_WAIT_SEC = 12.0
BOARD_DRIFT_WAIT_SEC = 75.0  # 센서당 30초 x 2 + 여유

# ── 동작 설정 (양팔 공통) ──────────────────────────────────────
AUTO_LOAD_CALIB   = False   # 시작 시 이전 교정 적용 안 함. 콘솔 load 로만 적용.
FRESH_SEC         = 0.5
REST_ELEV_DEG     = 25.0    # 이 아래면 REST
ANCHOR_ELEV_DEG   = 18.0    # 앵커는 더 확실히 내려왔을 때만 (히스테리시스)
ANCHOR_HOLD_SEC   = 0.5
ANCHOR_MAX_RATE   = 0.10    # 앵커 보정이 초당 움직일 수 있는 최대 각도

# ── 앵커(요 표류 보정) 설정 ────────────────────────────────────
#  앵커는 팔을 내렸을 때 w_ua(=위팔 장축에 수직인 임의 축)의 수평 방위를 재서
#  그 변화를 표류로 간주한다. 그런데 팔을 내리면 위팔 장축이 거의 수직이라
#  w_ua 는 수평면에 눕고, 그 방위는 곧 '위팔의 장축 회전(손바닥 방향)'이 된다.
#  즉 팔을 내릴 때마다 손바닥 방향이 달라지면 앵커가 그걸 표류로 착각한다.
#
#  2026-08-18 19시 세션(749샘플) 실측
#    앵커θ  -6.6 -> +6.6 -> +10.0  (3.5분간 한 방향으로 16.6도)
#    원본 az 는 추세가 없었다.
#    판정 정확도: 앵커 적용 655/749 / 앵커 끔 749/749
#  앵커가 도움이 아니라 방해였다. 그래서 기본을 damped 로 낮췄다.
#
#  off    : 보정하지 않는다. 세션이 짧고(5분 이내) 판정이 흔들리면 가장 안전.
#  damped : 게이트를 모두 통과한 관측만, 느리게, 상한 안에서 반영 (기본)
#  full   : 예전 동작 (게이트 없음, ANCHOR_MAX_RATE 만 적용)
ANCHOR_MODE          = "damped"
ANCHOR_DEADBAND_DEG  = 1.5
ANCHOR_JUMP_REJECT   = 8.0
ANCHOR_MAX_THETA     = 12.0
ANCHOR_MIN_OBS       = 5
GRAVITY_EMA_ALPHA = 0.3

FREEZE_N          = 8       # 같은 쿼터니언이 8회(약 0.8초) 반복되면 죽은 센서
STEADY_WINDOW_SEC = 0.5     # 교정 캡처 전 정지 확인 구간
STEADY_MAX_DEG    = 3.0     # 그 구간 내 허용 흔들림
TILT_WARN_DEG     = 25.0    # 몸통 기울기 경고 임계값
REST_MAG_WARN     = 0.30    # 차렷인데 mag가 이보다 크면 교정 불일치
MIN_MAG           = 0.30    # 이보다 작으면 수평 방향 자체가 정의되지 않음
MARGIN_WARN_DEG   = 5.0     # 경계까지 이만큼도 안 남으면 아슬아슬하다고 표시
POSE_HYST_DEG     = 3.0     # 경계 근처 떨림 방지 이력

# 팔꿈치 포락선 여유(도).
ELBOW_ENVELOPE_TOL = 5.0

# 차렷인데 팔꿈치가 이만큼 굽어 있다고 나오면 팔꿈치 영점이 잘못된 것이다.
FLEX_REST_MIN = 150.0

# ── 자세 판정 튜닝 (팔마다 따로) ───────────────────────────────
TUNING_DEFAULT = {
    "SPAN_K":     1.250,  # 실제 앞-옆 벌어짐 / 교정 캡처 az_side
                          # 3세션 실측이 1.314 / 1.313 / 1.068 로 갈렸다.
                          # 근본 해결은 learn 명령이다.
    "DIAG_FRAC":  0.52,   # 앞->옆 구간에서 대각선의 위치
    "DIAG_HALF":  0.42,   # 대각 구간이 이웃 기준각 쪽으로 뻗는 비율
    "DIAG_MIN_WIDTH":  14.0,
    "BACK_MARGIN_DEG": 25.0,
}
TUNING_OVERRIDE = {
    "L": {},
    "R": {},
}

PROFILE_ABS = {
    "L": {"fwd": -5.3, "diag": 23.6, "side": 50.4},
    "R": {"fwd": -5.3, "diag": 23.6, "side": 50.4},   # 미검증
}
PROFILE_MODE = "scaled"     # scaled | abs | cal

POSE_FWD  = "앞으로 90도"
POSE_DIAG = "대각선 90도"
POSE_SIDE = "옆으로 90도"

# ── 전역 수신 상태 ─────────────────────────────────────────────
latest = {}
connected = set()
gravity_ema = {}
gravity_cache = {}   # sensor_id -> (quat, 평활 결과). 프레임당 1회 갱신 보장
sensor_stat = {}
history = {}         # sensor_id -> deque[(t, quat)]
notes = {"msg": "", "t": 0.0}

# [v6.1] 보드/뷰어 구분. 쿼터니언을 보낸 소켓 = 보드.
board_sockets = set()
subscribed = set()      # {"cmd":"subscribe"} 를 보내면 보드 분류에서 빠진다
socket_since = {}
CLASSIFY_GRACE = 1.0

# [v6.1] 보드가 보고한 마지막 진단 결과
#   {BOARD_NAME: {"fw":.., "t":.., "cal":[..], "check":[..], "drift":[..]}}
board_state = {}


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
    두 마디의 지연이 달라지니 팔을 곧게 편 채로 들었다 내려도
    |t_fa - t_ua| 가 12도까지 벌어져 팔꿈치가 굽은 것처럼 보고됐다.
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
        tail.append(f"{side} {a.anchor['theta']:+5.1f}도" if a.ready()
                    else f"{side} 미교정")
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
                "twist": c["a_ua"] is not None,
                "anchor": self.anchor["ref"] is not None}

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

        raw(상대 쿼터니언에서 직접 구한 관절각)가 본래의 측정값이고, 중력으로
        만든 [lo, hi] 포락선은 요 표류로 raw 가 틀어졌을 때를 잡는 안전장치다.

        포락선은 '순간' 중력으로 만든다. 평활된 중력을 쓰면 raw 는 순간값,
        포락선은 지연값이라 시간축이 어긋나서, 빠르게 굽혔다 펴면 클램프가
        raw 를 덮어쓰고 120~130 도대로 튀었다가 EMA 가 따라잡으면 돌아왔다.
        """
        c = self.calib
        if c["u_ua"] is None or c["u_fa"] is None:
            return None, 0.0

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
        drift_min = max(0.0, lo - raw, raw - hi)   # 여유 적용 전 실제 위반량
        lo = max(0.0, lo - tol)
        hi = min(180.0, hi + tol)

        flex = max(lo, min(hi, raw))
        flex_inverted = 180.0 - max(0.0, min(180.0, flex))
        return flex_inverted, drift_min

    # ── 판정 기준각 / 경계 ──
    def pose_refs(self):
        """자세별 기준 방위각(도). (앞, 대각, 옆)"""
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
            # 2) 갑자기 크게 벌어진 관측은 표류가 아니라 자세가 달라진 것이다
            if abs(gap) > ANCHOR_JUMP_REJECT:
                a["rate"] = 0.0
                a["rejects"] = a.get("rejects", 0) + 1
                return
            # 3) 작은 차이는 잡음이다. 쫓아가면 오히려 흔들린다.
            if abs(gap) < ANCHOR_DEADBAND_DEG:
                a["rate"] = 0.0
                return

        dt = now - (a["prev"] or now)
        step = ANCHOR_MAX_RATE * max(dt, 1e-3)
        delta = max(-step, min(step, target - a["theta"]))

        new_theta = a["theta"] + delta
        # 4) 총 보정량 상한. 여기 닿았으면 표류가 아니라 교정이 어긋난 것이다.
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
        note(f"[ok] {self.name} 팔꿈치 영점 완료. 지금 자세가 완전 펴짐입니다.")
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
            note(f"[거부] {self.name} 몸통 기준이 {tilt:.0f}도 기울었습니다. 어깨 "
                 f"센서가 팔을 따라 움직이는지 확인하세요. 몸통에 고정해야 합니다.")
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
                    note(f"[거부] {self.name} 앞 방향 재설정 실패. 옆과 너무 가깝습니다.")
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
        note(f"[ok] {self.name} 교정을 전부 지웠습니다. 1 차렷 영점부터 다시.")

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
            "cal_age": (None if self.cal_time is None
                        else round(now - self.cal_time)),
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
            out["status"] = (f"차렷인데 팔꿈치가 {180 - out['flex']:.0f}도 굽은 "
                             f"것으로 나옵니다. 팔을 편 상태에서 영점을 다시 잡으세요.")
            return out

        # 교정이 이 세션 것이 맞는지 실시간 확인
        if elev < REST_ELEV_DEG and mag > REST_MAG_WARN:
            out["warn"].append("stale_calib")
            out["status"] = (f"차렷 자세인데 mag가 {mag:.2f}입니다. 교정이 지금 센서 "
                             f"부착 상태와 맞지 않습니다. reset 후 다시 잡으세요.")
            return out

        if tilt is not None and tilt > TILT_WARN_DEG:
            out["warn"].append("torso_tilt")
            out["status"] = (f"몸통 기준이 {tilt:.0f}도 기울었습니다. 어깨 센서가 "
                             f"팔을 따라 움직이면 앞/옆 판정이 어긋납니다.")
            return out

        if self.anchor["ref"] is None:
            out["status"] = "팔을 내리고 1초 정지하면 표류 앵커가 잡힙니다"
        elif self.anchor.get("capped"):
            out["warn"].append("anchor_capped")
            out["status"] = (f"앵커 보정이 상한 {ANCHOR_MAX_THETA:.0f}도에 닿았습니다. "
                             f"표류가 아니라 교정이 어긋난 것입니다. "
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
        "boards": board_summary(),
        "arms": {side: arms[side].analyze(now, frozen_ids) for side in ARM_ORDER},
    }


# ═══════════════════════════════════════════════════════════════
#  [v6.1] 보드 진단
# ═══════════════════════════════════════════════════════════════
def board_pick(token):
    """'l' / 'r' / 'sh' / 'all' / '' -> 대상 BOARD_NAME 목록 ([] = 전체)."""
    t = (token or "").strip().lower()
    if not t or t in ("all", "전체", "다", "both"):
        return None            # None = ALL
    name = BOARD_ALIAS.get(t)
    return [name] if name else None


def cal_verdict(e):
    """cal 결과 한 줄에 대한 통과/불합격 판정."""
    sd = e.get("sd")
    out = e.get("out")
    reasons = []
    if sd is None:
        reasons.append("흔들림 값 없음")
    elif sd > BOARD_SD_MAX_DPS:
        reasons.append(f"흔들림 {sd:.3f} > {BOARD_SD_MAX_DPS}")
    if out is None:
        reasons.append("이상치 값 없음")
    elif out > BOARD_OUT_MAX_PCT:
        reasons.append(f"이상치 {out:.1f}% > {BOARD_OUT_MAX_PCT}%")
    return (not reasons), reasons


def diag_verdict(e, kind):
    """check / drift 결과 한 줄 판정."""
    reasons = []
    if kind == "check":
        b = e.get("bias")
        o = e.get("out")
        if b is None:
            reasons.append("바이어스 값 없음")
        elif b > BOARD_BIAS_MAX_DPS:
            reasons.append(f"잔여 바이어스 {b:.3f} > {BOARD_BIAS_MAX_DPS}")
        if o is not None and o > BOARD_OUT_MAX_PCT:
            reasons.append(f"이상치 {o:.1f}% > {BOARD_OUT_MAX_PCT}%")
    else:
        d = e.get("drift")
        if d is None:
            reasons.append("표류 값 없음")
        elif d > BOARD_DRIFT_MAX_DPM:
            reasons.append(f"표류 {d:.2f} 도/분 > {BOARD_DRIFT_MAX_DPM}")
    return (not reasons), reasons


def board_summary():
    """뷰어로 내보낼 간단한 요약."""
    out = {}
    now = time.time()
    for name, st in board_state.items():
        rows = st.get("cal") or []
        oks = [cal_verdict(e)[0] for e in rows]
        out[name] = {
            "fw": st.get("fw"),
            "age": round(now - st.get("t", now)),
            "n": len(rows),
            "pass": bool(rows) and all(oks),
        }
    return out


def board_table(kind="cal"):
    """콘솔용 결과 표. kind: cal | check | drift"""
    title = {"cal": "자이로 0점 (bcal)",
             "check": "잔여 바이어스 진단 (bcheck)",
             "drift": "실측 표류 (bdrift)"}[kind]
    lines = ["", f"  ── 보드 진단 결과: {title} ──"]
    if kind == "cal":
        lines.append(f"  {'센서':<13}{'흔들림':>9}{'이상치':>8}{'샘플':>7}  판정")
    elif kind == "check":
        lines.append(f"  {'센서':<13}{'흔들림':>9}{'바이어스':>10}"
                     f"{'예상표류':>10}{'이상치':>8}  판정")
    else:
        lines.append(f"  {'센서':<13}{'표류':>12}  판정")

    any_row = False
    all_pass = True
    now = time.time()
    for name in ("SHOULDER_BOARD", "ARM_BOARD_L", "ARM_BOARD_R"):
        st = board_state.get(name)
        kr = BOARD_NAME_KR.get(name, name)
        if st is None:
            lines.append(f"  [{kr}] 보고 없음 (보드가 접속했는지, 펌웨어가 "
                         f"v1.4 이상인지 확인)")
            all_pass = False
            continue
        rows = st.get(kind) or []
        if not rows:
            lines.append(f"  [{kr}] {kind} 결과 없음")
            all_pass = False
            continue
        age = now - st.get(kind + "_t", st.get("t", now))
        lines.append(f"  [{kr}] fw {st.get('fw', '?')}  {age:.0f}초 전")
        for e in rows:
            any_row = True
            ok, why = (cal_verdict(e) if kind == "cal"
                       else diag_verdict(e, kind))
            all_pass = all_pass and ok
            mark = "통과" if ok else "불합격"
            sid = e.get("id", "?")
            if kind == "cal":
                lines.append(
                    f"    {sid:<11}{fmt(e.get('sd'), 3, '도/초'):>11}"
                    f"{fmt(e.get('out'), 1, '%'):>10}{str(e.get('n', '-')):>7}"
                    f"  {mark}" + (f"  ({', '.join(why)})" if why else ""))
            elif kind == "check":
                lines.append(
                    f"    {sid:<11}{fmt(e.get('sd'), 3):>9}"
                    f"{fmt(e.get('bias'), 3):>10}"
                    f"{fmt(e.get('per_min'), 1, '도/분'):>12}"
                    f"{fmt(e.get('out'), 1, '%'):>10}"
                    f"  {mark}" + (f"  ({', '.join(why)})" if why else ""))
            else:
                lines.append(
                    f"    {sid:<11}{fmt(e.get('drift'), 2, '도/분'):>14}"
                    f"  {mark}" + (f"  ({', '.join(why)})" if why else ""))

    if any_row:
        lines.append("")
        if all_pass:
            lines.append("  => 전체 통과. 이제 서버 교정(zero -> fwd -> side -> "
                         "diag)으로 넘어가세요.")
        else:
            lines.append("  => 불합격 항목이 있습니다. 보드를 완전히 정지시킨 뒤 "
                         "다시 실행하세요.")
            lines.append("     흔들림 초과 -> 움직이는 중. 이상치 초과 -> I2C 배선/"
                         "풀업/접지 점검.")
    return "\n".join(lines)


def fmt(v, nd=2, unit=""):
    if v is None:
        return "-"
    return f"{v:.{nd}f}{unit}"


async def send_to_boards(payload):
    """보드로 분류된 소켓에만 보낸다."""
    msg = json.dumps(payload)
    targets = [w for w in connected if w in board_sockets]
    if not targets:
        note("[거부] 접속한 보드가 없습니다. 보드가 켜져 있는지 확인하세요.")
        return 0
    await asyncio.gather(*(w.send(msg) for w in targets), return_exceptions=True)
    return len(targets)


async def board_command(kind, target_names, wait_sec):
    """보드에 진단 명령을 보내고, 결과가 올 때까지 기다렸다가 표를 출력한다."""
    cmd = {"cal": "boardcal", "check": "boardcheck", "drift": "boarddrift"}[kind]
    who = "전체" if target_names is None else ", ".join(
        BOARD_NAME_KR.get(n, n) for n in target_names)

    # 이번 회차 결과만 보이도록 이전 것을 지운다
    for name, st in board_state.items():
        if target_names is None or name in target_names:
            st.pop(kind, None)
            st.pop(kind + "_t", None)

    payload = {"cmd": cmd}
    if target_names is not None and len(target_names) == 1:
        payload["target"] = target_names[0]
    else:
        payload["target"] = "ALL"

    n = await send_to_boards(payload)
    if n == 0:
        return

    note(f"[{kind}] {who} 보드 {n}대에 명령을 보냈습니다. "
         f"{wait_sec:.0f}초 동안 움직이지 마세요.\n"
         f"       (측정 중에는 그 보드의 센서가 '끊김'으로 표시됩니다. 정상입니다)")

    # 결과가 다 모이면 일찍 끝낸다
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        await asyncio.sleep(0.5)
        want = (target_names if target_names is not None
                else [nm for nm in board_state])
        if want and all(board_state.get(nm, {}).get(kind) for nm in want):
            break

    note(board_table(kind))


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

    # ── [v6.1] 보드 진단 (비동기로 돌린다) ──
    if cmd in ("bcal", "bcheck", "bdrift", "calboards"):
        kind = {"bcal": "cal", "calboards": "cal",
                "bcheck": "check", "bdrift": "drift"}[cmd]
        wait = {"cal": BOARD_CAL_WAIT_SEC, "check": BOARD_CHECK_WAIT_SEC,
                "drift": BOARD_DRIFT_WAIT_SEC}[kind]
        names = board_pick(head)
        asyncio.ensure_future(board_command(kind, names, wait))
        return True

    if cmd == "boards":
        note(board_table(head.strip().lower()
                         if head.strip().lower() in ("cal", "check", "drift")
                         else "cal"))
        return True

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
        "  tune [SPAN_K=1.313 DIAG_HALF=0.42 ...]\n"
        "\n"
        "보드 진단 (뒤에 sh | L | R 을 붙이면 그 보드만, 없으면 전체)\n"
        "  bcal    자이로 0점 재측정 + 통과 판정  (약 10초, 정지 필요)\n"
        "  bcheck  오프셋 유지한 채 잔여 바이어스 진단 (약 8초)\n"
        "  bdrift  30초씩 적분해 실제 표류량 측정 (약 70초)\n"
        "  boards [cal|check|drift]   마지막 결과 다시 보기")


def learn_ref(a, arg=""):
    """지금 취하고 있는 실제 자세의 az 를 그 자세의 기준각으로 저장한다.

    교정 캡처(az_side)는 늘 실제 녹화 자세보다 덜 벌어져 있고, 그 비율(SPAN_K)이
    세션마다 다르다(실측 1.314 / 1.313 / 1.068). 고정 계수로는 맞출 수 없다.
    이 명령은 계수를 거치지 않고 실제 자세에서 직접 읽는다.
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
             f"자세를 잡은 채로 1초쯤 기다렸다가 다시 입력하세요.")
        return False
    vals.sort()
    n = len(vals)
    med = vals[n // 2]
    # 흔들림은 최소/최대 대신 10~90 백분위로 본다.
    spread = vals[int(0.9 * (n - 1))] - vals[int(0.1 * (n - 1))]
    if spread > 6.0:
        note(f"[거부] {a.name} learn: 최근 az 가 {spread:.0f}도나 흔들립니다. "
             f"자세를 멈추고 다시 입력하세요.")
        return False

    a.pose["abs"][key] = round(med, 1)
    a.pose["mode"] = "abs"
    p = a.pose["abs"]
    b = a.bounds()
    note(f"[ok] {a.name} '{key}' 기준각 = {med:.1f}도 "
         f"(표본 {len(vals)}개, 흔들림 {spread:.1f}도)\n"
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
    bs = d.get("boards") or {}
    if bs:
        parts = []
        for nm, b in bs.items():
            parts.append(f"{BOARD_NAME_KR.get(nm, nm)} "
                         f"{'통과' if b['pass'] else '미통과'}({b['age']}초전)")
        lines.append("  보드   " + "  ".join(parts))
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
            f"    앵커 {x['anchor_mode']}  보정 {x['drift']}도  "
            f"관측 {x['anchor_n']}회  거부 {x['anchor_rejects']}회"
            f"{'  [상한]' if x['anchor_capped'] else ''}",
            f"    교정 {x['calib']}",
        ]
    note("\n".join(lines))


# ═══════════════════════════════════════════════════════════════
#  통신
# ═══════════════════════════════════════════════════════════════
def handle_board_report(data):
    """보드가 보낸 hello / diag 를 저장한다."""
    name = data.get("board") or "?"
    st = board_state.setdefault(name, {})
    st["t"] = time.time()
    if data.get("fw"):
        st["fw"] = data["fw"]

    typ = data.get("type")
    if typ == "hello":
        rows = data.get("cal") or []
        st["cal"] = rows
        st["cal_t"] = st["t"]
        oks = [cal_verdict(e)[0] for e in rows]
        mark = "통과" if rows and all(oks) else "확인 필요"
        note(f"[보드] {BOARD_NAME_KR.get(name, name)} 자이로 0점 보고 "
             f"({len(rows)}개 센서) -> {mark}")
    elif typ == "diag":
        kind = data.get("kind")
        if kind in ("check", "drift"):
            st[kind] = data.get("d") or []
            st[kind + "_t"] = st["t"]
            note(f"[보드] {BOARD_NAME_KR.get(name, name)} {kind} 결과 도착 "
                 f"({len(st[kind])}개 센서)")


async def handler(ws, path=None):
    """[FIX-3 상당] path 기본값으로 websockets 10.x / 11+ 모두 대응."""
    connected.add(ws)
    socket_since[ws] = time.time()
    try:
        async for message in ws:
            try:
                data = json.loads(message)
            except ValueError:
                continue
            if not isinstance(data, dict):
                continue

            if "id" in data and "qw" in data:
                t = time.time()
                data["t"] = t
                latest[data["id"]] = data
                note_sample(data["id"], t,
                            (data["qw"], data["qx"], data["qy"], data["qz"]))
                # [v6.1] 쿼터니언을 보낸 소켓 = 보드
                if ws not in subscribed:
                    board_sockets.add(ws)
                continue

            if data.get("type") in ("hello", "diag"):
                if ws not in subscribed:
                    board_sockets.add(ws)
                handle_board_report(data)
                continue

            cmd = data.get("cmd")
            if not cmd:
                continue

            if cmd == "subscribe":
                # 쿼터니언도 보내면서 방송도 받고 싶은 클라이언트용 탈출구
                board_sockets.discard(ws)
                subscribed.add(ws)
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
                key = {"forward": "fwd", "side": "side", "diag": "diag"}.get(pose)
                if key:
                    run_cmd(key, arm_tok)
                continue

            run_cmd(cmd, arm_tok, src="ui")
    except Exception:
        pass
    finally:
        connected.discard(ws)
        board_sockets.discard(ws)
        subscribed.discard(ws)
        socket_since.pop(ws, None)


async def broadcast_loop():
    """[FIX-A] 보드에는 보내지 않는다. 보드 펌웨어는 TEXT 를 처리하지 않고,
    센서 6개 프레임은 3KB 에 가까워 순수한 낭비다."""
    while True:
        try:
            msg = json.dumps(analyze())
            now = time.time()
            targets = [w for w in connected
                       if w not in board_sockets
                       and (w in subscribed
                            or now - socket_since.get(w, 0) >= CLASSIFY_GRACE)]
            if targets:
                await asyncio.gather(*(w.send(msg) for w in targets),
                                     return_exceptions=True)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass
        await asyncio.sleep(0.1)


async def sensor_status_loop():
    while True:
        sys.stdout.write("\r" + sensor_line().ljust(100)[:100])
        sys.stdout.flush()
        await asyncio.sleep(0.5)


async def console_loop():
    loop = asyncio.get_event_loop()
    while True:
        try:
            line = (await loop.run_in_executor(None, sys.stdin.readline))
        except (EOFError, OSError, ValueError):
            # [FIX-C] 백그라운드 실행처럼 stdin 이 없으면 조용히 끈다.
            note("[정보] stdin 이 없어 키보드 명령을 끕니다. (뷰어는 그대로 동작)")
            return
        if not line:
            note("[정보] stdin 이 닫혔습니다. 키보드 명령을 끕니다.")
            return
        line = line.strip()
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        try:
            if not run_cmd(cmd.lower(), arg.strip()):
                note(HELP)
        except Exception as e:
            note(f"[오류] 명령 처리 실패 (서버는 계속 동작): {e}")


async def main():
    print("웨어러블 모션캡처 서버 v6.1 (양팔 / 6축 / 보드 원격 진단)")
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

    print("\n권장 순서")
    print("  1) bcal            보드 자이로 0점 (전부 통과할 때까지)")
    print("  2) bdrift          실측 표류 확인 (분당 10도 아래면 통과)")
    print("  3) zero -> fwd -> side -> diag   서버 교정 (팔마다)")
    print("  4) 팔 내리고 1초 정지 (앵커)")
    print()
    print(HELP + "\n")

    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.gather(broadcast_loop(), sensor_status_loop(),
                             console_loop())


if __name__ == "__main__":
    if websockets is None:
        print("pip install websockets 가 필요합니다.")
    else:
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            print("\n종료합니다.")
