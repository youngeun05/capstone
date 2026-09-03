"""
웨어러블 모션캡처 - 하체+허리(무릎/고관절/허리) 웹소켓 서버 v1.9
=============================================
팀 서버 server_v3_9.py 의 '팔꿈치 v3.4 방식'을 무릎에 그대로 이식한 것.
프로토콜(포트/JSON 형식/zero 명령)이 팀 서버와 동일해서,
보드 펌웨어와 뷰어를 그대로 팀 서버에 붙여도 동작한다.

무릎 = 팔꿈치와 완전히 같은 구조의 관절이다.
    상완(UPPERARM) -> 허벅지(THIGH)     : 몸통에 가까운 분절
    전완(FOREARM)  -> 종아리(CALF)      : 몸통에서 먼 분절
    팔꿈치 굴곡    -> 무릎 굴곡          : 0도 = 편 상태, 굽힐수록 커짐

계산 방식:
  1) 장축 방식 - 차렷(직립) 때 각 센서가 느낀 중력벡터를 그 분절의
     '장축'으로 삼고, 상대 쿼터니언으로 종아리 장축을 허벅지 프레임에
     옮겨 사이각을 잰다. 기하학적으로 정확하고 부착 각도가 상쇄된다.
  2) 중력 범위 클램프 - 중력만으로 결정되는 허용 범위 [하한, 상한]으로
     자른다. 이 범위는 yaw 드리프트에 면역이다.
  3) 범위를 벗어난 초과분은 '드리프트가 최소 이만큼 있다'는 증거로
     knee_*_drift_min 에 실어 내보내고 콘솔에 경고한다.

────────────────────────────────────────────────────────────────
v1.1~v1.2.1  서버가 죽는 버그 수정 (기형 패킷 / stdin 없음 / websockets
             버전 / 루프 격리 / zero 결과 방송 / 유효시간 필터 /
             필터 시점 일치 / Ctrl+C 종료 / 포트 점유 안내)
v1.3         고관절 추가 (WAIST_LOW - THIGH). 무릎과 같은 구조라 코드 재사용.
v1.4         앞뒤 구분(forward_axis) + 허리 관절 lumbar 추가.
v1.5         시상면 뺄셈 방식 추가. 자세가 바뀌어도 무릎각이 따라온다.
v1.6         전방축 자동 학습 + 주값을 '중력 경계 선택' 으로 교체.
v1.7         부호 규약을 분절별로 정리(SEGMENT_AXIS_SIGN 등) + 전방축 전파.
v1.8         허리 부호를 상대 회전으로 판정 (스쿼트에서 값이 점프하던 문제).
────────────────────────────────────────────────────────────────

────────────────────────────────────────────────────────────────
v1.9 변경점 — "허리가 확확 튄다" 완전 해결 + debug 명령 복구

  [FIX-15] 허리 각도를 '투영 방식' 으로 교체.

    ■ v1.8 이 여전히 틀린 이유
      v1.8 은 크기를 중력 경계(lo/hi)에서 고르고, 부호만 상대회전의
      '회전축 방향' 으로 정했다. 그런데 굴곡이 작을 때 그 회전축은
      옆굽힘·비틀림 성분에 지배된다. 골반 45도 기운 스쿼트 자세에서
      옆굽힘 10도 / 비틀림 8도만 섞여도 부호가 뒤집혔다.

        참 굴곡  옆굽힘  비틀림 |  v1.8   |  v1.9
          +2.0     10      8   |  -2.8   |  +2.7
          +0.5     14     12   |  -2.1   |  +2.0
          -3.0     10      8   |  +2.0   |  -2.3

      크기(lo)도 옆굽힘 성분을 포함해서 부풀어 있었다.

    ■ v1.9 가 하는 것
      상대회전을 회전벡터(축 x 각도)로 풀어 굴곡축에 투영한다.
      크기와 부호가 한 번에 나오고, 시상면 밖 성분은 투영에서 제거된다.
      0 근처를 부드럽게 통과하므로 부호 히스테리시스도 필요 없다.

    ■ 곱하는 순서 (중요)
        rel1 * conj(rel0)  -> 회전이 '근위 센서 프레임' 에 놓인다  ← 이걸 쓴다
        conj(rel0) * rel1  -> 원위 센서 프레임. v1.8 이 쓰던 순서다.
      forward_axis 는 근위 센서 프레임의 벡터라 반드시 전자와 맞춰야 한다.
      검증 결과 (두 허리 센서를 각각 20도 / 33도 비뚤게 붙인 상태):

        골반  참 허리 |  conj(rel0)*rel1  |  rel1*conj(rel0)
          0     +20   |       19.1        |      20.0
         45     -10   |       -9.5        |     -10.0
         70     +40   |       38.1        |      40.0

      상부 센서에 요 표류 20도를 넣고 골반을 0->60도 기울이면
        기존 순서 14.5 -> 12.4 -> 10.5 -> 9.1  (흘러내림)
        새 순서  14.8 -> 14.8 -> 14.8 -> 14.8  (고정)

    · lumbar_method 가 "proj" 로 바뀐다.
    · 무릎/고관절은 손대지 않았다. 그쪽은 lo 와 hi 가 자세에 따라 진짜로
      갈리는 관절이고 실측 검증도 끝났으므로 경계 선택이 맞다.

  [FIX-16] 교차 검증값 lumbar_grav / lumbar_gap 추가.
    lumbar_grav = waist_up_tilt - pelvis_tilt (중력만 쓰는 완전히 다른 경로)
    두 값이 10도 넘게 벌어지면 표류나 센서 이탈의 증거다.

  [FIX-17] debug 명령이 죽던 버그. print_debug() 가 정의되지 않은
    PASSTHROUGH_SENSORS 를 참조해 NameError 를 냈다. 지금은 WAIST_UP 도
    계산에 쓰므로 '통과 전용 센서' 자체가 없다. 해당 블록을 삭제하고,
    허리 요약을 투영값/중력차 대조로 바꿨다.
────────────────────────────────────────────────────────────────

실행법
------
    pip install "websockets>=11"
    python server_leg_v1_9.py

  'zero'    : 차렷(직립) 자세에서 영점
  'forward' : (선택) 앞으로 숙인 자세에서 앞뒤 기준을 고정
              안 써도 됩니다 - 15도 이상 움직이면 자동으로 잡힙니다
  'sensors' : 센서별 수신 상태
  'debug'   : 중력벡터/기울기/범위 진단 출력
  'stat'    : 접속/센서/기형패킷 상태
"""

import asyncio
import json
import math
import os
import sys
import threading
import time
import traceback

import websockets

PORT = 8766   # [통합] 상체 서버(v6, 8765)와 같은 PC에서 함께 돌리려고 8766 으로 옮겼다.
              #   두 서버는 방송 형식이 달라서(상체 {"v":6,...} / 하체 {"type":"angles",...})
              #   포트를 나누고 통합 뷰어가 양쪽에 각각 접속한다.
              #   ⚠ 다리 보드 펌웨어의 WS_PORT 도 8766 으로 바꿔 업로드할 것.

# ══════════════════════════════════════════════════════════════
#  센서 배치 정의
# ══════════════════════════════════════════════════════════════
# 튜플 순서: (몸통에 가까운 쪽 = 근위, 먼 쪽 = 원위)
KNEE_JOINTS = {
    "knee_L": ("L_THIGH", "L_CALF"),
    "knee_R": ("R_THIGH", "R_CALF"),
}

# ── [v1.3] 고관절 ──
#     골반(WAIST_LOW) : 근위 분절   <- 무릎의 허벅지 자리
#     허벅지(THIGH)   : 원위 분절   <- 무릎의 종아리 자리
HIP_JOINTS = {
    "hip_L": ("WAIST_LOW", "L_THIGH"),
    "hip_R": ("WAIST_LOW", "R_THIGH"),
}

# ── [v1.4] 허리 관절 ──
#     WAIST_LOW(아래=골반) : 근위 분절
#     WAIST_UP (위=흉요추) : 원위 분절
LUMBAR_JOINTS = {
    "lumbar": ("WAIST_LOW", "WAIST_UP"),
}

# 모든 관절 = (근위 분절 센서, 원위 분절 센서)
ALL_JOINTS = {**KNEE_JOINTS, **HIP_JOINTS, **LUMBAR_JOINTS}

# 출력 키에 쓸 분절 이름. knee_L_tilt_thigh / hip_L_tilt_pelvis 처럼 붙는다.
SEGMENT_LABELS = {
    "knee_L": ("thigh", "calf"),
    "knee_R": ("thigh", "calf"),
    "hip_L":  ("pelvis", "thigh"),
    "hip_R":  ("pelvis", "thigh"),
    "lumbar": ("low", "up"),
}

# ══════════════════════════════════════════════════════════════
#  [v1.7] 부호 규약 — 앞뒤 판정의 핵심
# ══════════════════════════════════════════════════════════════
# 분절의 기울기는 '영점 때 중력벡터' 와 '지금 중력벡터' 의 사이각으로 재고,
# 그 회전축은 cross(영점, 현재) 다. 여기에 함정이 하나 있다:
#
#     허벅지를 앞으로 들면 (고관절 굴곡)  -> 회전축 -x
#     상체를 앞으로 숙이면                -> 회전축 +x        (실측값)
#
# 둘 다 몸 기준으로는 '앞' 인데 회전축이 정반대다. 몸통은 '위쪽' 이 앞으로
# 가고 다리는 '아래쪽' 이 앞으로 가기 때문이다.
SEGMENT_AXIS_SIGN = {
    "WAIST_LOW": -1, "WAIST_UP": -1,      # 몸통: 위쪽이 앞으로 간다
    "R_THIGH": +1, "L_THIGH": +1,         # 다리: 아래쪽이 앞으로 간다
    "R_CALF": +1, "L_CALF": +1,
}

# SEGMENT_LEARN_FLIP: 그 센서가 '가장 크게' 하는 동작이 몸 기준 앞인가 뒤인가.
SEGMENT_LEARN_FLIP = {
    "WAIST_LOW": +1, "WAIST_UP": +1,
    "R_THIGH": +1, "L_THIGH": +1,
    "R_CALF": -1, "L_CALF": -1,
}

# JOINT_COEF: 관절각 = a*근위기울기 + b*원위기울기  (둘 다 '몸 기준 앞이 +')
#   · 무릎   = 허벅지 - 종아리
#   · 고관절 = 골반 + 허벅지
#   · 허리   = 상부 - 하부
JOINT_COEF = {
    "knee_L": (+1, -1), "knee_R": (+1, -1),
    "hip_L":  (+1, +1), "hip_R":  (+1, +1),
    "lumbar": (-1, +1),
}

# [v1.8] 부호를 '상대 회전' 으로 판정할 관절. (v1.9 에서 허리는 투영 방식이
#   이걸 대체하지만, 투영이 불가능한 상황의 대비책으로 남겨둔다.)
RELATIVE_SIGN_JOINTS = {"lumbar"}

# [v1.9] 허리는 '투영 방식' 으로 크기와 부호를 한 번에 구한다.
#   기존: 중력 경계(lo/hi)로 크기를 정하고 회전축 방향으로 ±1 만 정했다.
#         굴곡이 작을 때 그 회전축이 옆굽힘·비틀림에 지배돼 부호가 뒤집혔다.
#         (실측: 참 +2도인데 -2.8도, 참 -3도인데 +2.0도)
#   지금: 상대회전을 회전벡터로 풀어 굴곡축에 투영한다. 시상면 밖 성분은
#         투영에서 제거되고, 0 근처를 부드럽게 통과해 히스테리시스도 불필요.
LUMBAR_PROJ_JOINTS = {"lumbar"}
LUMBAR_MAX_DEG = 90.0

# 부호가 0 근처에서 떨리는 걸 막는 히스테리시스(도).
SIGN_HYST_DEG = 4.0
sign_hold = {}      # {joint_name: +1.0 / -1.0} 직전에 확정된 부호

# 골반 기울기를 재는 센서 = 아래쪽 허리 센서.
PELVIS_SENSOR = "WAIST_LOW"
WAIST_UP_SENSOR = "WAIST_UP"

# 전방축 학습에 필요한 최소 기울기.
FORWARD_MIN_TILT = 15.0

QUAT_KEYS = ("qw", "qx", "qy", "qz")

# ══════════════════════════════════════════════════════════════
#  전역 상태
# ══════════════════════════════════════════════════════════════
latest = {}          # 센서별 최신 쿼터니언
connected = set()    # 접속 중인 클라이언트

# [v1.3] 쿼터니언 패킷을 보낸 적 있는 소켓 = 보드. 나머지 = 뷰어/로거.
# 보드에는 angles 프레임(1.2KB 이상)을 되쏘지 않는다. ESP32 수신 버퍼에
# 걸려 보드가 끊길 수 있고, 보드 1대당 12KB/s 씩 아낀다.
board_sockets = set()
subscribed = set()   # {"cmd":"subscribe"} 를 보낸 소켓은 보드로 분류하지 않는다

# [v1.4] 소켓별 접속 시각. 보드인지 뷰어인지 판별될 때까지 angles 를 안 보낸다.
socket_since = {}
CLASSIFY_GRACE = 1.0

# 영점: 직립 자세에서 각 센서가 느낀 중력벡터
#   {joint_name: {"g_prox": (x,y,z), "g_dist": (x,y,z)}}
joint_gravity_zero = {}

# 상대 쿼터니언 영점
joint_quat_zero = {name: None for name in ALL_JOINTS}

# [v1.4] 센서별 직립 기준 중력벡터 (관절이 아니라 센서 단위)
sensor_gravity_zero = {}

# [v1.7] 영점 시점의 센서 자세 전체. 전방축 전파에 쓴다.
sensor_quat_zero = {}

# [v1.4] 센서별 '전방축'. 앞으로 숙였을 때의 회전축.
#   ※ 전부 센서 프레임의 중력벡터만 쓰므로 yaw 드리프트에 완전히 면역이다.
forward_axis = {}

# [v1.6] 전방축 자동 학습.
auto_best = {}
forward_locked = set()
forward_src = {}         # sid -> "manual" | "auto" | "prop"(전파받음)
AUTO_MIN_TILT = 15.0
AUTO_MARGIN = 1.25

# [v1.6] 관절별로 직전에 lo/hi 중 무엇을 골랐는지 (히스테리시스용)
bound_pick = {}
BOUND_HYST = 8.0

# ── EMA 필터 ──
EMA_ALPHA = 0.3
quat_smoothed = {}       # {sensor_id: (qw,qx,qy,qz)}

# 센서 데이터 유효시간
SENSOR_TIMEOUT = 0.5

bad_packets = 0
bad_packet_last_reason = ""
reported_stale = set()

# ── [v1.4] 센서별 수신 진단 ──
sensor_stats = {}


# ══════════════════════════════════════════════════════════════
#  쿼터니언 유틸리티
# ══════════════════════════════════════════════════════════════
def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def relative_quat(qA, qB):
    """A 기준으로 본 B 의 회전. A=허벅지, B=종아리면 곧 무릎."""
    return quat_mul(quat_conjugate(qA), qB)


def quat_rotate(q, v):
    vq = (0.0, v[0], v[1], v[2])
    r = quat_mul(quat_mul(q, vq), quat_conjugate(q))
    return (r[1], r[2], r[3])


def vec_dot(a, b):
    return sum(ai*bi for ai, bi in zip(a, b))


def vec_cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def vec_unit(v):
    n = math.sqrt(sum(c*c for c in v))
    return None if n < 1e-9 else tuple(c / n for c in v)


# ══════════════════════════════════════════════════════════════
#  [FIX-1] 수신 패킷 검증 / 정규화
# ══════════════════════════════════════════════════════════════
def sanitize_quat_packet(data):
    """
    보드가 보낸 dict 를 검사해서 안전한 쿼터니언 패킷으로 바꾼다.
    통과 못 하면 (None, 사유) 를 돌려주고 호출자가 그냥 버린다.
    """
    if not isinstance(data, dict):
        return None, "dict 가 아님"

    sid = data.get("id")
    if not isinstance(sid, str) or not sid:
        return None, "id 없음/문자열 아님"

    vals = []
    for k in QUAT_KEYS:
        if k not in data:
            return None, f"{k} 누락"
        try:
            v = float(data[k])
        except (TypeError, ValueError):
            return None, f"{k} 가 숫자가 아님({data[k]!r})"
        if not math.isfinite(v):
            return None, f"{k} 가 유한수가 아님({v})"
        vals.append(v)

    n = math.sqrt(sum(v*v for v in vals))
    if n < 1e-6:
        return None, "쿼터니언 크기가 0에 가까움"

    clean = dict(data)
    for k, v in zip(QUAT_KEYS, vals):
        clean[k] = v / n

    z = data.get("z")
    clean["z"] = z if isinstance(z, int) else 0
    return clean, ""


def note_bad_packet(reason):
    global bad_packets, bad_packet_last_reason
    bad_packets += 1
    bad_packet_last_reason = reason
    if bad_packets == 1 or bad_packets % 100 == 0:
        print(f"[경고] 기형 패킷 무시 (누적 {bad_packets}개) 최근 사유: {reason}")


# ══════════════════════════════════════════════════════════════
#  중력벡터 (yaw 드리프트 무관)
# ══════════════════════════════════════════════════════════════
def gravity_in_sensor(q):
    """센서 프레임에서 본 중력방향. yaw 드리프트에 영향을 받지 않는다."""
    return quat_rotate(quat_conjugate(q), (0.0, 0.0, 1.0))


def quat_ema_update(sensor_id, q_raw):
    """
    [v1.2] 쿼터니언 EMA. 패킷을 받을 때마다 갱신한다.
    이중 덮개(q 와 -q) 보정과 재정규화를 반드시 한다.
    """
    prev = quat_smoothed.get(sensor_id)
    if prev is None:
        quat_smoothed[sensor_id] = q_raw
        return q_raw

    q = q_raw
    if sum(a*b for a, b in zip(prev, q)) < 0.0:
        q = tuple(-c for c in q)

    a = EMA_ALPHA
    m = tuple(a*qi + (1 - a)*pi for qi, pi in zip(q, prev))
    n = math.sqrt(sum(c*c for c in m))
    if n < 1e-9:
        quat_smoothed[sensor_id] = q_raw
        return q_raw
    m = tuple(c / n for c in m)
    quat_smoothed[sensor_id] = m
    return m


def tilt_between_deg(g_zero, g_now):
    dot = max(-1.0, min(1.0, vec_dot(g_zero, g_now)))
    return math.degrees(math.acos(dot))


def propagate_forward_axes():
    """
    [v1.7] 축이 잡힌 센서에서 아직 안 잡힌 센서로 '앞' 방향을 전달한다.

      천골(WAIST_LOW)은 허리를 젖힐 때 10도도 잘 안 기울어 자기 힘으로는
      기준을 못 잡는다. 반면 허벅지는 앉기·걷기만 해도 90도씩 기운다.
      영점 자세에서는 모든 센서가 같은 몸 방향을 보고 있었으므로,
      영점 상대회전으로 옮기면 된다:
          v_in_B = R(q_zero_B)^-1 * R(q_zero_A) * v_in_A
    """
    if not forward_axis:
        return []
    src = max(forward_axis, key=lambda s: auto_best.get(s, 0.0))
    q_src = sensor_quat_zero.get(src)
    if q_src is None:
        return []

    filled = []
    for sid in {s for pair in ALL_JOINTS.values() for s in pair}:
        if sid in forward_axis or sid not in sensor_quat_zero:
            continue
        v_world = quat_rotate(q_src, forward_axis[src])
        v_sid = quat_rotate(quat_conjugate(sensor_quat_zero[sid]), v_world)
        ax = vec_unit(v_sid)
        if ax is None:
            continue
        # 몸통 <-> 다리는 '앞' 의 회전축이 정반대다
        if SEGMENT_AXIS_SIGN.get(src, 1) * SEGMENT_AXIS_SIGN.get(sid, 1) < 0:
            ax = tuple(-c for c in ax)
        forward_axis[sid] = ax
        forward_src[sid] = "prop"
        auto_best[sid] = 0.0
        filled.append(sid)
    if filled:
        print(f"    [전파] {src} 의 앞 방향을 {', '.join(sorted(filled))} 에 전달 "
              f"(영점 자세 기준) -> 이제 그 센서들도 앞뒤가 구분됩니다")
    return filled


def auto_learn_forward(sensor_id, g_now):
    """
    [v1.6] 패킷이 들어올 때마다 '가장 크게 기운 방향' 을 전방축으로 잡아 둔다.
    사람이 'forward' 로 직접 잡아준 센서는 건드리지 않는다.
    """
    if sensor_id in forward_locked:
        return
    gz = sensor_gravity_zero.get(sensor_id)
    if gz is None:
        return
    ang = tilt_between_deg(gz, g_now)
    if ang < AUTO_MIN_TILT:
        return
    if ang <= auto_best.get(sensor_id, 0.0) * AUTO_MARGIN:
        return
    ax = vec_unit(vec_cross(gz, g_now))
    if ax is None:
        return
    if SEGMENT_LEARN_FLIP.get(sensor_id, 1) < 0:
        ax = tuple(-c for c in ax)
    first = sensor_id not in forward_axis or forward_src.get(sensor_id) == "prop"
    forward_axis[sensor_id] = ax
    auto_best[sensor_id] = ang
    forward_src[sensor_id] = "auto"
    if first:
        print(f"    [자동] {sensor_id} 전방축 학습 ({ang:.0f}도 기울임)")
    propagate_forward_axes()


def signed_tilt_deg(sensor_id, g_now):
    """
    [v1.4] 부호가 붙은 기울기. + = 앞으로 숙임, - = 뒤로 젖힘.
    전방축을 아직 학습하지 않았으면 크기만 돌려준다(항상 양수).
    """
    gz = sensor_gravity_zero.get(sensor_id)
    if gz is None:
        return None
    ang = tilt_between_deg(gz, g_now)
    ax = forward_axis.get(sensor_id)
    if ax is None or ang < 1.0:
        return ang
    return ang if vec_dot(vec_cross(gz, g_now), ax) >= 0 else -ang


# ══════════════════════════════════════════════════════════════
#  관절 각도 계산
# ══════════════════════════════════════════════════════════════
def segment_tilts(joint_name, idA, idB, q_th, q_ca):
    """각 분절이 영점(직립) 대비 기운 각. 드리프트 면역."""
    zero = joint_gravity_zero.get(joint_name)
    if zero is None:
        return None
    g_th = gravity_in_sensor(q_th)
    g_ca = gravity_in_sensor(q_ca)
    return (tilt_between_deg(zero["g_prox"], g_th),
            tilt_between_deg(zero["g_dist"], g_ca))


def compute_longaxis(joint_name, q_th, q_ca):
    """
    두 분절 장축 사이각. 부착 각도 상쇄, 기하학적으로 정확.
    ⚠ 상대 쿼터니언을 쓰므로 두 센서의 yaw 드리프트 차이가 섞인다.
    """
    zero = joint_gravity_zero.get(joint_name)
    if zero is None:
        return None
    ca_in_th = quat_rotate(relative_quat(q_th, q_ca), zero["g_dist"])
    dot = max(-1.0, min(1.0, vec_dot(ca_in_th, zero["g_prox"])))
    return math.degrees(math.acos(dot))


def gravity_bounds(t_th, t_ca):
    """중력만으로 결정되는 관절 각도 허용 범위 (구면 코사인 법칙)."""
    lo = abs(t_ca - t_th)
    s = t_ca + t_th
    hi = min(s, 360.0 - s)
    return (lo, hi) if lo <= hi else (hi, lo)


def compute_sagittal(joint_name, idA, idB, q_prox, q_dist):
    """
    [v1.5] 부호 있는 기울기의 뺄셈으로 구한 시상면 굴곡각.
    각 분절의 기울기는 중력만으로 구하므로 방위 오차에 완전히 면역이다.
    전방축이 두 센서 모두 학습돼 있어야 쓸 수 있다.
    """
    if idA not in forward_axis or idB not in forward_axis:
        return None
    sp = signed_tilt_deg(idA, gravity_in_sensor(q_prox))
    sd = signed_tilt_deg(idB, gravity_in_sensor(q_dist))
    if sp is None or sd is None:
        return None
    a, b = JOINT_COEF.get(joint_name, (-1, +1))
    return a * sp + b * sd


def compute_sagittal_relative(joint_name, idA, idB, q_prox, q_dist):
    """
    [v1.8] 상대 회전으로 부호만 구한다. (+1 굽힘 / -1 젖힘 / None 판정불가)

    아래쪽(prox) 분절 자신을 기준으로 보므로 몸통이 얼마나 기울어도
    판정이 흔들리지 않는다. v1.9 의 투영 방식이 이걸 대체하지만,
    전방축이 없어 투영이 불가능할 때의 대비책으로 남겨둔다.
    """
    zero = joint_quat_zero.get(joint_name)
    if zero is None:
        return None
    ax = forward_axis.get(idA)
    if ax is None:
        return None

    q_delta = quat_mul(relative_quat(q_prox, q_dist), quat_conjugate(zero))

    w, x, y, z = q_delta
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    vlen = math.sqrt(x*x + y*y + z*z)
    if vlen < 1e-6:
        return None

    axis = (x/vlen, y/vlen, z/vlen)
    # 전방축은 '직립중력 x 현재중력' 외적이라 실제 굴곡 회전축과 반대를
    # 가리킨다. 그래서 내적이 '음수' 일 때가 앞으로 굽힌 것이다.
    d = vec_dot(axis, ax)
    if abs(d) < 1e-6:
        return None
    return 1.0 if d < 0 else -1.0


def signed_flex_proj(joint_name, idA, q_prox, q_dist):
    """
    [v1.9] 상대회전을 굴곡축에 투영해 부호 있는 굴곡각을 직접 구한다.

    ⚠ 곱하는 순서가 결과를 바꾼다.
        rel1 * conj(rel0)  -> 회전이 '근위 센서 프레임' 에 놓인다  ← 이걸 쓴다
        conj(rel0) * rel1  -> 원위 센서 프레임이라, 두 센서의 부착 각도
                              차이만큼 축이 어긋난다 (검증: 참 20도 -> 19.1도,
                              골반이 기울수록 오차가 커진다)
    forward_axis 는 근위 센서 프레임의 벡터이므로 반드시 전자와 맞춰야 한다.

    부호: forward_axis 는 '영점중력 x 현재중력' 외적이라 실제 굴곡 회전축과
    반대를 가리킨다. 그래서 내적에 마이너스를 붙인다 (v1.8 주석과 동일).
    """
    zero = joint_quat_zero.get(joint_name)
    ax = forward_axis.get(idA)
    if zero is None or ax is None:
        return None

    qd = quat_mul(relative_quat(q_prox, q_dist), quat_conjugate(zero))
    w, x, y, z = qd
    if w < 0:                       # q 와 -q 는 같은 회전
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-9:
        return 0.0                  # 영점과 같은 자세
    a = math.degrees(2.0 * math.atan2(n, w))
    r = (x / n * a, y / n * a, z / n * a)      # 회전벡터 (도)
    v = -vec_dot(r, ax)
    return max(-LUMBAR_MAX_DEG, min(LUMBAR_MAX_DEG, v))


def compute_joint(joint_name, idA, idB, q_th, q_ca):
    """
    반환 dict:
        flex      : 최종값 (0도 = 편 상태, 굽힐수록 커짐)
        long      : 장축 원본 (클램프 전)
        sub       : 뺄셈값 (t_dist - t_prox)
        sag       : 시상면 가정 추정치 = t_prox + t_dist. 드리프트 면역이라
                    flex 와 대조하면 드리프트가 꼈는지 알 수 있다.
        signed    : 부호 있는 값 (+ 굽힘 / - 젖힘)
        method    : "bound_lo" | "bound_hi" | "proj"
        lo, hi    : 중력이 허용하는 범위
        drift_min : 장축값이 범위를 벗어난 정도 (드리프트 하한 증거)
        t_prox    : 근위 분절 기울기
        t_dist    : 원위 분절 기울기
    """
    t = segment_tilts(joint_name, idA, idB, q_th, q_ca)
    lng = compute_longaxis(joint_name, q_th, q_ca)
    if t is None or lng is None:
        return None
    t_prox, t_dist = t
    lo, hi = gravity_bounds(t_prox, t_dist)
    drift_min = max(0.0, lo - lng, lng - hi)

    # ══ [v1.6] 주값 = 중력 경계 중 가까운 쪽 고르기 ══
    #
    # lo = |t_dist - t_prox|,  hi = t_prox + t_dist  는 중력만으로 구하므로
    # 방위(yaw) 오차에 완전히 면역이다. 시상면 동작에서는 참값이 반드시
    # 이 둘 중 하나다:
    #     두 분절이 같은 방향으로 기울면 -> 참값 = lo
    #     서로 반대 방향으로 기울면      -> 참값 = hi
    # 그래서 둘 중 어느 쪽인지만 고르면 되고, 그 판정에만 장축값을 쓴다.
    # 장축값이 45도 틀어져도 '가까운 쪽 고르기' 는 |hi-lo|/2 의 여유가 있다.
    d_lo, d_hi = abs(lng - lo), abs(lng - hi)
    prev = bound_pick.get(joint_name)
    if prev == "lo":
        pick = "lo" if d_lo <= d_hi + BOUND_HYST else "hi"
    elif prev == "hi":
        pick = "hi" if d_hi <= d_lo + BOUND_HYST else "lo"
    else:
        pick = "lo" if d_lo <= d_hi else "hi"
    bound_pick[joint_name] = pick

    flex = lo if pick == "lo" else hi
    flex = max(0.0, min(180.0, flex))
    method = "bound_" + pick

    # [v1.9] 허리는 경계 선택을 쓰지 않는다. 투영값이 크기와 부호를 모두 준다.
    if joint_name in LUMBAR_PROJ_JOINTS:
        pr = signed_flex_proj(joint_name, idA, q_th, q_ca)
        if pr is not None:
            sign_hold[joint_name] = 1.0 if pr >= 0 else -1.0
            return {"flex": abs(pr), "long": lng,
                    "sub": max(0.0, t_dist - t_prox),
                    "sag": min(180.0, t_prox + t_dist),
                    "signed": pr, "method": "proj",
                    "lo": lo, "hi": hi, "drift_min": drift_min,
                    "t_prox": t_prox, "t_dist": t_dist}

    # 부호(앞/뒤)는 전방축이 있어야 알 수 있다. 크기는 위에서 이미 정했으므로
    # 부호만 붙여 준다.
    signed = None
    if joint_name in RELATIVE_SIGN_JOINTS:
        sgn = compute_sagittal_relative(joint_name, idA, idB, q_th, q_ca)
        if sgn is not None:
            signed = sgn * flex
    if signed is None:
        sag_signed = compute_sagittal(joint_name, idA, idB, q_th, q_ca)
        if sag_signed is not None:
            signed = (math.copysign(flex, sag_signed)
                      if abs(sag_signed) > 1e-9 else 0.0)

    # [v1.8] 부호 히스테리시스. 굴곡각이 작을 때는 회전축 방향이 원래
    #   불안정하므로, 그 구간에서는 직전 부호를 유지해 깜빡임을 막는다.
    if signed is not None:
        if flex >= SIGN_HYST_DEG:
            sign_hold[joint_name] = 1.0 if signed >= 0 else -1.0
        else:
            held = sign_hold.get(joint_name)
            if held is not None:
                signed = held * flex

    return {"flex": flex, "long": lng, "sub": max(0.0, t_dist - t_prox),
            "sag": min(180.0, t_prox + t_dist),
            "signed": signed, "method": method,
            "lo": lo, "hi": hi, "drift_min": drift_min,
            "t_prox": t_prox, "t_dist": t_dist}


# ── 호환용: 팀 서버 FLEX_JOINTS 방식 (총 회전량) ──
def rotation_angle_deg(qd):
    w = min(1.0, abs(qd[0]))
    return math.degrees(2 * math.acos(w))


# ══════════════════════════════════════════════════════════════
#  센서 데이터 접근 / 영점
# ══════════════════════════════════════════════════════════════
def is_fresh(sensor_id, now):
    """그 센서가 지금 살아 있는가."""
    d = latest.get(sensor_id)
    return (d is not None
            and now - d["t"] <= SENSOR_TIMEOUT
            and d.get("z") == 1)


def _fresh_quat(sensor_id, now):
    """계산에 쓰는 쿼터니언 = 필터를 거친 값."""
    if not is_fresh(sensor_id, now):
        return None
    return quat_smoothed.get(sensor_id)


def _fresh_quat_raw(sensor_id, now):
    """뷰어/로거로 내보내는 원본 쿼터니언."""
    if not is_fresh(sensor_id, now):
        return None
    d = latest[sensor_id]
    return (d["qw"], d["qx"], d["qy"], d["qz"])


def capture_zero():
    """직립(차렷) 자세에서 모든 기준값을 저장."""
    now = time.time()
    zeroed, failed, missing_all = [], [], []

    # [v1.4] 영점이 바뀌면 예전 전방축은 기준이 달라져 무의미해진다.
    forward_axis.clear()
    auto_best.clear()
    forward_locked.clear()
    sensor_gravity_zero.clear()
    # [v1.8] 직전 판정 상태도 같이 비운다.
    sign_hold.clear()
    bound_pick.clear()
    all_sensor_ids = sorted({sid for pair in ALL_JOINTS.values() for sid in pair})
    sensor_quat_zero.clear()
    forward_src.clear()
    for sid in all_sensor_ids:
        q = _fresh_quat_raw(sid, now)
        if q is not None:
            sensor_gravity_zero[sid] = gravity_in_sensor(q)
            sensor_quat_zero[sid] = q

    for name, (idA, idB) in ALL_JOINTS.items():
        # 영점은 '지금 이 순간' 기준이므로 원본을 쓰고, 필터 상태도 그 값으로
        # 리셋해 영점과 이후 값의 시점을 맞춘다.
        qA, qB = _fresh_quat_raw(idA, now), _fresh_quat_raw(idB, now)
        if qA is not None and qB is not None:
            quat_smoothed[idA] = qA
            quat_smoothed[idB] = qB
            joint_gravity_zero[name] = {
                "g_prox": gravity_in_sensor(qA),
                "g_dist": gravity_in_sensor(qB),
            }
            joint_quat_zero[name] = relative_quat(qA, qB)
            zeroed.append(name)
            g = joint_gravity_zero[name]["g_prox"]
            lp, _ = SEGMENT_LABELS[name]
            print(f"    [영점] {name} 저장 완료  "
                  f"g_{lp}=[{g[0]:.3f}, {g[1]:.3f}, {g[2]:.3f}]")
        else:
            missing = [i for i, q in ((idA, qA), (idB, qB)) if q is None]
            failed.append(name)
            missing_all.extend(missing)
            print(f"    [영점 실패] {name}: {', '.join(missing)} 데이터 없음")

    # [v1.4] 보드 자체가 없어서 실패한 관절은 조용히 넘어간다.
    absent = {sid for sid in missing_all if sid not in latest}
    soft_failed = [n for n in failed
                   if any(sid in absent for sid in ALL_JOINTS[n])]
    hard_failed = [n for n in failed if n not in soft_failed]

    if soft_failed:
        print(f"    [정보] 센서가 없어 건너뛴 관절: {', '.join(soft_failed)} "
              f"(없는 센서: {', '.join(sorted(absent))})")

    if hard_failed:
        print(f"    ⚠⚠ 영점 실패: {', '.join(hard_failed)} — "
              f"이 상태로 측정하면 flex 값이 전부 비어 나옵니다.")
        print(f"       보드 전원/와이파이를 확인하고 'zero' 를 다시 치세요.")

    return {"ok": bool(zeroed) and not hard_failed,
            "zeroed": zeroed, "failed": failed,
            "missing": sorted(set(missing_all)),
            "has_pelvis": PELVIS_SENSOR in sensor_gravity_zero,
            "has_lumbar": "lumbar" in zeroed,
            "forward": []}


def capture_forward():
    """
    [v1.4] '앞으로 숙인 자세' 를 보여주면 각 센서의 전방축을 학습한다.

      ★ 추천 (한 번에 6개 센서 전부): 앉아 윗몸 앞으로 굽히기(체전굴)
        바닥에 앉아 두 다리를 곧게 앞으로 뻗고, 고관절부터 접어 상체를
        발끝 쪽으로 숙인다.

      ⚠ 선 채로 등만 둥글게 마는 자세는 안 된다. 골반(WAIST_LOW)이 10도도
        안 기울어서 학습에서 빠지고, 뒤로 젖혀도 앞으로 나온다.
    """
    now = time.time()
    learned, too_small, no_data = [], [], []

    for sid in sorted({s for pair in ALL_JOINTS.values() for s in pair}):
        q = _fresh_quat_raw(sid, now)
        gz = sensor_gravity_zero.get(sid)
        if q is None or gz is None:
            if sid in latest or gz is not None:
                no_data.append(sid)
            continue
        g = gravity_in_sensor(q)
        ang = tilt_between_deg(gz, g)
        if ang < FORWARD_MIN_TILT:
            too_small.append(f"{sid}({ang:.0f}도)")
            continue
        ax = vec_unit(vec_cross(gz, g))
        if ax is None:
            too_small.append(f"{sid}(축 불안정)")
            continue
        forward_axis[sid] = ax
        forward_locked.add(sid)      # 수동 지정이 자동 학습보다 우선
        auto_best[sid] = 180.0
        forward_src[sid] = "manual"
        learned.append(sid)
        print(f"    [전방축] {sid} 수동 지정 완료 (숙인 각도 {ang:.0f}도)")

    if too_small:
        print(f"    [건너뜀] 덜 숙여진 센서: {', '.join(too_small)}  "
              f"(최소 {FORWARD_MIN_TILT:.0f}도 필요)")
    if no_data:
        print(f"    [건너뜀] 데이터 없음: {', '.join(no_data)}")
    propagate_forward_axes()
    still = sorted(sid for sid in latest if sid not in forward_axis
                   and sid in {s for pair in ALL_JOINTS.values() for s in pair})
    if not learned:
        print("    ⚠ 학습된 센서가 없습니다. 'zero' 를 먼저 하고, "
              "앞으로 더 숙인 자세에서 다시 시도하세요.")
    else:
        print(f"    => 학습된 센서는 앞으로 +, 뒤로 - 로 나옵니다.")
    if still:
        print(f"    ⚠ 아직 학습 안 된 센서: {', '.join(still)}")
        if any(s.startswith("WAIST") for s in still):
            print(f"       허리가 남았으면 '고관절부터 접어' 상체를 숙인 자세에서")
            print(f"       다시 실행하세요. 등만 둥글게 말면 골반이 안 기울어")
            print(f"       학습이 안 되고, 뒤로 젖혀도 앞으로 나옵니다.")
        print(f"       추천 자세: 앉아 윗몸 앞으로 굽히기(체전굴) - 한 번에 6개 전부")

    return {"ok": bool(learned), "learned": learned,
            "too_small": too_small, "no_data": no_data}


def forward_result_message(result):
    return dumps_safe({"cmd": "forward", **result})


def zero_result_message(result):
    """기존 뷰어/로거는 d.cmd === 'zero' 만 보므로 필드를 더해도 안전하다."""
    return dumps_safe({"cmd": "zero", **result})


# ══════════════════════════════════════════════════════════════
#  관절각도 계산 (0.1초마다 브라우저로 전송되는 딕셔너리)
# ══════════════════════════════════════════════════════════════
def compute_angles():
    result = {}
    now = time.time()

    for name, (idA, idB) in ALL_JOINTS.items():
        lab_prox, lab_dist = SEGMENT_LABELS[name]
        keys = ("_flex_g", "_flex_long", "_flex_sub", "_flex_sag",
                "_signed", "_method",
                "_lo", "_hi", "_drift_min",
                "_tilt_" + lab_prox, "_tilt_" + lab_dist)

        qA, qB = _fresh_quat(idA, now), _fresh_quat(idB, now)

        # 호환용 (팀 서버 knee_* 와 같은 키/방식)
        qz = joint_quat_zero.get(name)
        if qA is not None and qB is not None and qz is not None:
            qd = quat_mul(relative_quat(qA, qB), quat_conjugate(qz))
            result[name] = round(rotation_angle_deg(qd), 1)
        else:
            result[name] = None

        k = compute_joint(name, idA, idB, qA, qB) \
            if (qA is not None and qB is not None) else None
        if k is not None:
            result[name + "_flex_g"] = round(k["flex"], 1)
            result[name + "_flex_long"] = round(k["long"], 1)
            result[name + "_flex_sub"] = round(k["sub"], 1)
            result[name + "_flex_sag"] = round(k["sag"], 1)
            result[name + "_signed"] = (round(k["signed"], 1)
                                        if k["signed"] is not None else None)
            result[name + "_method"] = k["method"]
            result[name + "_lo"] = round(k["lo"], 1)
            result[name + "_hi"] = round(k["hi"], 1)
            result[name + "_drift_min"] = round(k["drift_min"], 1)
            result[name + "_tilt_" + lab_prox] = round(k["t_prox"], 1)
            result[name + "_tilt_" + lab_dist] = round(k["t_dist"], 1)
        else:
            for kk in keys:
                result[name + kk] = None

    # ── 골반 단독 기울기 (상체를 얼마나 숙였나) ──
    qp = _fresh_quat(PELVIS_SENSOR, now)
    s_low = signed_tilt_deg(PELVIS_SENSOR, gravity_in_sensor(qp)) \
        if qp is not None else None
    result["pelvis_tilt"] = round(s_low, 1) if s_low is not None else None

    # 위쪽 허리 센서의 기울기 (부호 포함)
    qu = _fresh_quat(WAIST_UP_SENSOR, now)
    s_up = signed_tilt_deg(WAIST_UP_SENSOR, gravity_in_sensor(qu)) \
        if qu is not None else None
    result["waist_up_tilt"] = round(s_up, 1) if s_up is not None else None

    # [v1.9] 계산 경로가 완전히 다른 두 번째 허리각. 투영값과 벌어지면
    #   표류나 센서 이탈의 증거다 (flex 와 sag 를 대조하는 것과 같은 발상).
    if s_low is not None and s_up is not None:
        grav = s_up - s_low
        result["lumbar_grav"] = round(grav, 1)
        lp = result.get("lumbar_signed")
        result["lumbar_gap"] = None if lp is None else round(abs(lp - grav), 1)
    else:
        result["lumbar_grav"] = None
        result["lumbar_gap"] = None

    # 부호가 의미 있는 상태인지 클라이언트가 알 수 있게
    result["signed_ok"] = (PELVIS_SENSOR in forward_axis
                           and WAIST_UP_SENSOR in forward_axis)

    # 전방축이 아직 없는 센서
    result["need_forward"] = sorted(
        sid for sid in {s for pair in ALL_JOINTS.values() for s in pair}
        if sid in latest and sid not in forward_axis)

    return result


# ══════════════════════════════════════════════════════════════
#  전송 헬퍼
# ══════════════════════════════════════════════════════════════
async def send_to_all(msg, viewers_only=False):
    """viewers_only=True 면 보드(쿼터니언을 보내는 쪽)는 건너뛴다."""
    if viewers_only:
        now = time.time()
        targets = [ws for ws in connected
                   if ws not in board_sockets
                   and (ws in subscribed
                        or now - socket_since.get(ws, 0) >= CLASSIFY_GRACE)]
    else:
        targets = list(connected)
    if not targets:
        return
    await asyncio.gather(
        *(ws.send(msg) for ws in targets),
        return_exceptions=True,
    )


def dumps_safe(obj):
    """
    allow_nan=False 로 직렬화한다. NaN 이 섞이면 파이썬 json 은 'NaN' 이라는
    비표준 리터럴을 뱉고 브라우저의 JSON.parse 가 예외를 던진다.
    """
    try:
        return json.dumps(obj, allow_nan=False)
    except ValueError as e:
        print(f"[경고] 직렬화 불가(NaN/Inf 포함) - 이 프레임 건너뜀: {e}")
        return None


# ══════════════════════════════════════════════════════════════
#  웹소켓 처리
# ══════════════════════════════════════════════════════════════
# [FIX-3] path 기본값을 둬서 websockets 10.x(2인자)와 11+(1인자) 모두 동작.
async def handler(websocket, path=None):
    connected.add(websocket)
    socket_since[websocket] = time.time()
    print(f"[+] 클라이언트 연결됨 (현재 {len(connected)}개)")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except (json.JSONDecodeError, TypeError, UnicodeDecodeError):
                note_bad_packet("JSON 파싱 실패")
                continue

            if not isinstance(data, dict):
                note_bad_packet("최상위가 객체가 아님")
                continue

            if "id" in data and "qw" in data:
                clean, reason = sanitize_quat_packet(data)
                if clean is None:
                    note_bad_packet(reason)
                    continue
                now_t = time.time()
                clean["t"] = now_t
                sid = clean["id"]
                q = (clean["qw"], clean["qx"], clean["qy"], clean["qz"])

                # [v1.4] 센서별 수신 통계 (진단용)
                st = sensor_stats.get(sid)
                if st is None:
                    st = sensor_stats[sid] = {
                        "count": 0, "first_t": now_t, "last_change": now_t,
                        "frozen_n": 0, "prev_q": None}
                st["count"] += 1
                if st["prev_q"] == q:
                    st["frozen_n"] += 1
                else:
                    st["frozen_n"] = 0
                    st["last_change"] = now_t
                st["prev_q"] = q

                latest[sid] = clean
                if websocket not in subscribed:
                    board_sockets.add(websocket)

                # [v1.2] 필터는 '패킷을 받을 때마다' 갱신한다.
                quat_ema_update(sid, q)
                # [v1.6] 부호 기준을 스스로 잡는다 (사용자 조작 불필요)
                auto_learn_forward(sid, gravity_in_sensor(quat_smoothed[sid]))

            elif data.get("cmd") == "subscribe":
                # 쿼터니언도 보내면서 angles 도 받고 싶은 클라이언트용 탈출구.
                board_sockets.discard(websocket)
                subscribed.add(websocket)
                print("[정보] 클라이언트가 subscribe 요청 - angles 를 보냅니다")

            elif data.get("cmd") == "zero":
                print(">>> 클라이언트 요청으로 영점 처리 <<<")
                zero_msg = zero_result_message(capture_zero())
                if zero_msg:
                    await send_to_all(zero_msg)

            elif data.get("cmd") == "forward":
                print(">>> 클라이언트 요청으로 전방축 학습 <<<")
                fw_msg = forward_result_message(capture_forward())
                if fw_msg:
                    await send_to_all(fw_msg)

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception:
        # [FIX-4] 이 연결 하나 때문에 서버가 멈추지 않게
        print("[오류] 클라이언트 핸들러 예외:")
        traceback.print_exc()
    finally:
        connected.discard(websocket)
        was_board = websocket in board_sockets
        board_sockets.discard(websocket)
        subscribed.discard(websocket)
        socket_since.pop(websocket, None)
        kind = "보드" if was_board else "뷰어/로거"
        print(f"[-] {kind} 연결 끊김 "
              f"(보드 {len(board_sockets)}대, "
              f"뷰어 {len(connected)-len(board_sockets)}개)")


async def broadcast_loop():
    tick = 0
    while True:
        # [FIX-4] 한 틱이 터져도 루프는 계속 돈다.
        try:
            angles = compute_angles()
            now = time.time()

            quats, status, stale = {}, {}, []
            for sid, v in latest.items():
                if now - v["t"] <= SENSOR_TIMEOUT:
                    status[sid] = v.get("z", 0)
                    quats[sid] = {"qw": round(v["qw"], 5),
                                  "qx": round(v["qx"], 5),
                                  "qy": round(v["qy"], 5),
                                  "qz": round(v["qz"], 5)}
                else:
                    stale.append(sid)

            for sid in stale:
                if sid not in reported_stale:
                    reported_stale.add(sid)
                    print(f"[경고] {sid} 데이터 끊김 "
                          f"({SENSOR_TIMEOUT}초 이상 무응답) - 보드 전원/와이파이 확인")
            for sid in list(reported_stale):
                if sid not in stale:
                    reported_stale.discard(sid)
                    print(f"[정보] {sid} 데이터 복구됨")

            # [v1.7] 부호가 왜 안 잡히는지 화면에서 바로 보이게
            diag = {}
            for sid in sorted({s for pair in ALL_JOINTS.values() for s in pair}):
                if sid not in latest:
                    continue
                qd = _fresh_quat(sid, now)
                st_ = (signed_tilt_deg(sid, gravity_in_sensor(qd))
                       if qd is not None else None)
                diag[sid] = {"t": round(st_, 1) if st_ is not None else None,
                             "src": forward_src.get(sid, "-"),
                             "max": round(auto_best.get(sid, 0.0), 0)}

            msg = dumps_safe({"type": "angles", "angles": angles,
                              "status": status, "quats": quats,
                              "stale": stale, "diag": diag,
                              "forward": sorted(forward_axis.keys())})
            if msg:
                await send_to_all(msg, viewers_only=True)

            tick += 1
            if tick % 10 == 0:
                if latest:
                    info = ""
                    for name in ALL_JOINTS:
                        fg = angles.get(name + "_flex_g")
                        if fg is None:
                            continue
                        dm = angles.get(name + "_drift_min") or 0.0
                        sag = angles.get(name + "_flex_sag")
                        warn = (f"  [드리프트 >={dm:.0f}도! 영점 재설정 권장]"
                                if dm > 5 else "")
                        info += f" | {name}: 굴곡{fg}°(시상면{sag}°){warn}"
                    pt = angles.get("pelvis_tilt")
                    if pt is not None:
                        info += f" | 골반숙임 {pt}°"
                    ls = angles.get("lumbar_signed")
                    lgap = angles.get("lumbar_gap")
                    if ls is not None:
                        info += f" | 허리 {ls:+.1f}°"
                        if lgap is not None and lgap > 10:
                            info += f" [중력차와 {lgap:.0f}도 불일치]"
                    print(f"[수신중] 센서: {list(latest.keys())}{info}")
                else:
                    print("[대기중] 아직 어떤 보드에서도 데이터가 안 옴...")

        except asyncio.CancelledError:
            raise
        except Exception:
            print("[오류] 브로드캐스트 한 틱 실패 (서버는 계속 동작):")
            traceback.print_exc()

        await asyncio.sleep(0.1)


def _stdin_reader(loop, queue):
    """
    [FIX-8] 키보드 입력을 '데몬 스레드' 에서 읽는다. 기본 실행기의 작업
    스레드는 데몬이 아니라서, 인터프리터 종료 시 input() 이 돌아오기를
    영원히 기다리며 포트를 계속 물고 있었다.
    """
    while True:
        try:
            line = input("")
        except (EOFError, OSError, ValueError):
            loop.call_soon_threadsafe(queue.put_nowait, None)
            return
        except BaseException:
            loop.call_soon_threadsafe(queue.put_nowait, None)
            return
        try:
            loop.call_soon_threadsafe(queue.put_nowait, line)
        except RuntimeError:      # 이벤트 루프가 이미 닫힘 = 종료 중
            return


async def zero_command_loop():
    loop = asyncio.get_running_loop()
    queue = asyncio.Queue()
    threading.Thread(target=_stdin_reader, args=(loop, queue),
                     daemon=True, name="stdin-reader").start()

    while True:
        line = await queue.get()
        if line is None:
            print("[정보] stdin 이 없어 키보드 명령을 끕니다.")
            print("       (뷰어의 zero 버튼은 그대로 동작합니다)")
            return

        try:
            cmd = line.strip().lower()

            if cmd == "zero":
                print(">>> 영점 처리 <<<")
                cmd_msg = zero_result_message(capture_zero())
                if cmd_msg:
                    await send_to_all(cmd_msg)

            elif cmd in ("forward", "fwd", "f"):
                print(">>> 전방축 학습 (앞으로 숙인 자세를 유지하세요) <<<")
                fw_msg = forward_result_message(capture_forward())
                if fw_msg:
                    await send_to_all(fw_msg)

            elif cmd == "debug":
                print_sensors()
                print_debug()

            elif cmd in ("sensors", "sensor", "s"):
                print_sensors()

            elif cmd in ("stat", "stats"):
                print(f"  접속 클라이언트 {len(connected)}개 / "
                      f"센서 {list(latest.keys())} / "
                      f"보드 {len(board_sockets)}대 / "
                      f"기형 패킷 누적 {bad_packets}개"
                      + (f" (최근: {bad_packet_last_reason})"
                         if bad_packets else ""))

        except asyncio.CancelledError:
            raise
        except Exception:
            print("[오류] 명령 처리 실패 (서버는 계속 동작):")
            traceback.print_exc()


def print_sensors():
    """
    [v1.4] 센서별 수신 상태를 한눈에.

      · '없음'       -> 그 ID 로 데이터가 아예 안 온다 (전원/와이파이/ID 중복)
      · '값 고정'    -> 패킷은 오는데 쿼터니언이 안 변한다 = MPU 를 못 읽는 중
      · 전송률 낮음  -> 와이파이가 약하거나 보드가 자주 재부팅된다
    """
    now = time.time()
    expected = sorted({sid for pair in ALL_JOINTS.values() for sid in pair})
    print("\n=== 센서 상태 ===")
    print(f"  {'센서':<11}{'상태':<12}{'전송률':>8}{'마지막수신':>10}"
          f"{'값변화':>10}  비고")
    problems = []
    for sid in expected:
        st = sensor_stats.get(sid)
        d = latest.get(sid)
        if st is None or d is None:
            print(f"  {sid:<11}{'없음':<12}{'-':>8}{'-':>10}{'-':>10}  "
                  f"데이터가 한 번도 안 옴")
            problems.append(f"{sid}: 데이터 없음")
            continue

        age = now - d["t"]
        dur = max(1e-6, d["t"] - st["first_t"])
        rate = (st["count"] - 1) / dur if st["count"] > 1 else 0.0
        since_change = now - st["last_change"]
        frozen = st["frozen_n"] >= 20        # 10Hz 기준 2초 이상 완전 동일

        if age > SENSOR_TIMEOUT:
            state, tail = "끊김", f"{age:.1f}초째 무응답"
            problems.append(f"{sid}: 끊김")
        elif frozen:
            state, tail = "값 고정", "MPU 를 못 읽는 중 (배선/AD0/납땜 확인)"
            problems.append(f"{sid}: 값 고정")
        elif rate < 5.0:
            state, tail = "느림", "보드는 10Hz 로 보내야 정상"
            problems.append(f"{sid}: 전송률 {rate:.1f}Hz")
        else:
            state, tail = "정상", ""

        q = (d["qw"], d["qx"], d["qy"], d["qz"])
        if abs(q[0] - 1.0) < 1e-6 and all(abs(c) < 1e-6 for c in q[1:]):
            tail = (tail + "  " if tail else "") + "쿼터니언이 초기값 그대로"

        print(f"  {sid:<11}{state:<12}{rate:>6.1f}Hz{age:>9.1f}초"
              f"{since_change:>9.1f}초  {tail}")

    unknown = sorted(set(sensor_stats) - set(expected))
    if unknown:
        print(f"  [참고] 정의에 없는 센서 ID: {', '.join(unknown)}")
    n_boards = len(board_sockets)
    n_ids = len([s for s in expected if s in latest])
    print(f"\n  접속한 보드 {n_boards}대 / 살아있는 센서 ID {n_ids}개")
    if n_boards and n_ids < n_boards:
        print(f"  ⚠ 보드 수보다 센서 ID 가 적습니다. 두 보드가 같은 ID 를 쓰고")
        print(f"     있을 가능성이 큽니다 -> 각 보드의 BOARD_SELECT 를 확인하세요.")

    # [v1.7] 앞뒤 부호 상태
    print(f"\n  ── 앞뒤 기준(전방축) 상태 ──")
    print(f"  {'센서':<11}{'현재 기울기':>12}{'최대 관측':>10}  기준 출처")
    SRC = {"manual": "수동 지정", "auto": "직접 학습",
           "prop": "다른 센서에서 전파받음", "-": "없음 (부호 안 붙음)"}
    for sid in expected:
        if sid not in latest:
            continue
        q = _fresh_quat(sid, now)
        t = signed_tilt_deg(sid, gravity_in_sensor(q)) if q is not None else None
        src = forward_src.get(sid, "-")
        print(f"  {sid:<11}{(f'{t:+.1f}도' if t is not None else '-'):>12}"
              f"{auto_best.get(sid, 0.0):>9.0f}도  {SRC.get(src, src)}")
    if not forward_axis:
        print("  ⚠ 아직 아무 센서도 기준을 못 잡았습니다. 어느 부위든 15도 이상")
        print("     한 번 움직이면 그 센서가 잡고, 나머지에 자동으로 전파됩니다.")
        print("     (앉았다 일어서기 한 번이면 허벅지가 90도 기울어 확실합니다)")

    if problems:
        print(f"\n  문제 있는 센서: {', '.join(problems)}")
    else:
        print("\n  모든 센서 정상")
    print()


def print_debug():
    print("\n=== 관절 진단 ===")
    now = time.time()

    qp = _fresh_quat(PELVIS_SENSOR, now)
    qu = _fresh_quat(WAIST_UP_SENSOR, now)
    pt = ut = None
    if qp is not None:
        pt = signed_tilt_deg(PELVIS_SENSOR, gravity_in_sensor(qp))
        mark = "앞으로" if (pt or 0) >= 0 else "뒤로"
        cal = "" if PELVIS_SENSOR in forward_axis else "  ⚠ 전방축 미학습(부호 없음)"
        print(f"  골반({PELVIS_SENSOR}) 숙임 = {pt:+7.1f}도 ({mark}){cal}")
    if qu is not None:
        ut = signed_tilt_deg(WAIST_UP_SENSOR, gravity_in_sensor(qu))
        print(f"  상부허리({WAIST_UP_SENSOR}) 기울기 = {ut:+7.1f}도")

    # [v1.9] 허리는 투영값이 주값이다. 중력차와 대조해 표류를 잡는다.
    if qp is not None and qu is not None and pt is not None and ut is not None:
        grav = ut - pt
        k = compute_joint("lumbar", PELVIS_SENSOR, WAIST_UP_SENSOR, qp, qu)
        proj = None if k is None else k["signed"]
        kind = "굽힘(flexion)" if (proj if proj is not None else grav) >= 0 \
            else "폄/젖힘(extension)"
        pstr = "-" if proj is None else f"{proj:+.1f}"
        print(f"  => 허리각  투영 {pstr}도   중력차 {grav:+.1f}도   {kind}"
              f"   [{'-' if k is None else k['method']}]")
        if proj is not None and abs(proj - grav) > 10:
            print(f"     ⚠ 두 방식이 {abs(proj - grav):.1f}도 차이 -> "
                  f"표류 또는 옆굽힘 성분")
        if proj is None:
            print(f"     ⚠ 투영 불가: {PELVIS_SENSOR} 전방축이 아직 없습니다. "
                  f"앉았다 일어서기를 한 번 하세요.")

    if not forward_axis:
        print("  ⚠ 전방축 미학습: 15도 이상 한 번 움직이면 자동으로 잡힙니다.")
    elif PELVIS_SENSOR not in latest:
        print(f"  골반({PELVIS_SENSOR}) 센서 없음 - 무릎만 쓰는 구성")

    for name, (idA, idB) in ALL_JOINTS.items():
        lab_prox, lab_dist = SEGMENT_LABELS[name]
        z = joint_gravity_zero.get(name)
        qA, qB = _fresh_quat(idA, now), _fresh_quat(idB, now)
        if z is None:
            print(f"  {name}: 영점 미설정 ('zero' 를 먼저 실행)")
            continue
        if qA is None or qB is None:
            miss = [i for i, q in ((idA, qA), (idB, qB)) if q is None]
            print(f"  {name}: {', '.join(miss)} 데이터 없음")
            continue
        g_p, g_d = gravity_in_sensor(qA), gravity_in_sensor(qB)
        t_p = tilt_between_deg(z["g_prox"], g_p)
        t_d = tilt_between_deg(z["g_dist"], g_d)
        print(f"  {name}")
        print(f"    {lab_prox}({idA}) 영점 g=[{z['g_prox'][0]:+.3f}, "
              f"{z['g_prox'][1]:+.3f}, {z['g_prox'][2]:+.3f}]  "
              f"현재 g=[{g_p[0]:+.3f}, {g_p[1]:+.3f}, {g_p[2]:+.3f}]")
        print(f"      tilt_{lab_prox} = {t_p:6.1f}도")
        print(f"    {lab_dist}({idB}) tilt_{lab_dist} = {t_d:6.1f}도")
        k = compute_joint(name, idA, idB, qA, qB)
        if k is not None:
            print(f"    뺄셈           = {k['sub']:6.1f}도")
            print(f"    장축 원본      = {k['long']:6.1f}도")
            print(f"    시상면 추정치  = {k['sag']:6.1f}도"
                  f"  <- 앞뒤 동작(스쿼트 등)이면 이게 참값. 드리프트 면역")
            print(f"    중력 허용 범위 = [{k['lo']:.1f}, {k['hi']:.1f}]도"
                  f"  (폭 {k['hi']-k['lo']:.0f}도)")
            if k["drift_min"] > 0.5:
                print(f"    ⚠ 장축값이 범위를 {k['drift_min']:.1f}도 벗어남"
                      f" -> 드리프트 최소 그만큼. 영점을 다시 잡으세요.")
            if k["method"] != "proj":
                gap = abs(k["flex"] - k["sag"])
                if gap > 10:
                    print(f"    ⚠ flex 와 시상면 추정치가 {gap:.1f}도 차이 -> "
                          f"옆으로 벌린 동작이거나 드리프트")
            sg = "-" if k["signed"] is None else f"{k['signed']:+.1f}"
            print(f"    => 최종 flex   = {k['flex']:6.1f}도  부호값 {sg}도"
                  f"  [{k['method']}]   (뷰어 표시각 {180 - k['flex']:.1f}도)")
    if bad_packets:
        print(f"  [기형 패킷 누적 {bad_packets}개, 최근 사유: "
              f"{bad_packet_last_reason}]")
    print()


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
async def supervised(coro_factory, name):
    """[FIX-4] 루프가 예기치 못하게 죽으면 로그를 남기고 되살린다."""
    while True:
        try:
            await coro_factory()
            print(f"[정보] {name} 종료됨.")
            return
        except asyncio.CancelledError:
            raise
        except Exception:
            print(f"[오류] {name} 가 예외로 중단됨 - 1초 후 재시작:")
            traceback.print_exc()
            await asyncio.sleep(1.0)


async def main():
    print(f"하체(무릎/고관절/허리) 웹소켓 서버 v1.9: ws://<이 컴퓨터의 IP>:{PORT}")
    print("터미널 명령:")
    print("  'zero'    직립 자세에서 영점")
    print("  'forward' (선택) 앞으로 굽힌 자세에서 앞뒤 기준을 '고정'")
    print("            안 써도 됩니다 - 15도 이상 움직이면 자동으로 잡힙니다")
    print("  'sensors' 센서별 수신 상태 (특정 센서가 이상할 때)")
    print("  'debug'   각도 진단 출력   /   'stat' 접속 상태")
    print()
    print("v1.9: 허리 각도를 '투영 방식' 으로 교체했습니다.")
    print("  - 상대회전을 회전벡터로 풀어 굴곡축에 투영 -> 크기와 부호를 동시에")
    print("  - 옆굽힘·비틀림 성분이 투영에서 제거돼 부호가 안 뒤집힙니다")
    print("  - lumbar_method 가 'proj' 로 나오면 적용된 것입니다")
    print("  - lumbar_grav(중력차)와 10도 넘게 벌어지면 표류를 의심하세요")
    print()
    print("센서 ID:  L_THIGH / L_CALF / R_THIGH / R_CALF   (다리 보드 4대)")
    print("          WAIST_LOW / WAIST_UP                  (허리 보드 1대, MPU 2개)")
    print("관절:     무릎 knee_L/knee_R,  고관절 hip_L/hip_R,  허리 lumbar")
    print()

    # [FIX-9] 포트가 이미 쓰이고 있을 때 트레이스백 대신 해결법을 안내
    try:
        server = await websockets.serve(handler, "0.0.0.0", PORT)
    except OSError as e:
        print()
        print(f"[오류] 포트 {PORT} 를 열 수 없습니다: {e}")
        print()
        print("  이미 이 서버가 떠 있을 가능성이 큽니다. 남은 프로세스를 정리하세요.")
        print(f"    Windows : Get-Process -Id (Get-NetTCPConnection "
              f"-LocalPort {PORT}).OwningProcess | Stop-Process -Force")
        print(f"    macOS/리눅스 : lsof -ti tcp:{PORT} | xargs kill")
        print()
        return

    async with server:
        await asyncio.gather(
            supervised(broadcast_loop, "브로드캐스트 루프"),
            supervised(zero_command_loop, "키보드 명령 루프"),
        )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    finally:
        # [FIX-8] stdin 데몬 스레드가 버퍼 락을 쥔 채 멈춰 있어서, 정상 종료
        # 절차를 밟으면 _enter_buffered_busy 오류가 난다. 즉시 빠져나간다.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
