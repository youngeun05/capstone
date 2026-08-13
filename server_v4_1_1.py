"""
웨어러블 모션캡처 중앙 서버 v4.1  (오른팔)
==========================================

v4.0 에서 바뀐 것 두 가지
  1. REST 자동 재앵커      - 시간이 갈수록 앞/옆이 틀어지던 문제
  2. 센서 연결 실시간 표시 - 콘솔 한 줄 + 뷰어 패널

■ 왜 시간이 지나면 앞/옆이 틀어지나 (측정으로 확인)

  거상각(elev) 은 중력벡터만 쓰므로 드리프트가 없다. v3.9 방식 그대로다.
      elev = acos( 차렷때_상완중력 · 현재_상완중력 )

  방위각(az) 은 어깨센서 대비 상완센서의 상대 회전을 쓴다.
  두 센서의 yaw 는 각자 따로 흘러가므로 그 차이가 az 에 그대로 들어온다.

      debug_data_2026-08-12.csv 의 REST 구간(팔 내리고 정지, 10.6초)
          어깨-상완 상대 yaw 표류   약 6~16 도/분
          측정 잔차 표준편차        0.41 도

  판정 구간 폭이 (앞->옆 51도) / 3 = 약 17도 이므로 1분에 한 칸씩 밀린다.
  정확히 "처음엔 맞다가 점점 틀어지는" 증상이다.

■ 중력만으로 앞/옆을 가를 수는 없나 - 없다 (측정으로 확인)

  v3 때 쓰던 전완 중력 z성분 방식과 상완 중력의 롤 성분을
  debug_data 로 다시 재봤다. 손바닥 방향(몸쪽/앞쪽/뒤쪽)에 완전히 묻힌다.

      상완 롤   앞 66.7 / 90.4 / 43.8      옆 44.2 / 92.7 / 62.7
      전완 g.z  앞 -0.5 / -0.6 / +0.5      옆 +0.2 / -0.9 / +0.5

  중력은 수평 방향 정보를 갖지 않는다. 어깨센서 기준은 피할 수 없다.

■ 그래서 자동 재앵커

  드리프트는 "관측이 안 되는" 양이 아니다.
  팔을 내린 상태(REST)에서는 참값을 안다 - 팔은 그때 아래를 향한다.
  그 자세에서 상완 센서의 롤축이 몸통 프레임에서 어느 쪽을 보는지 재면
  그동안 쌓인 표류량 θ 가 그대로 나온다. (위 잔차 0.41도 = 충분히 정밀)

      elev < 15도 가 1초 이상 유지되면  ->  θ 를 다시 재서 EMA 로 반영
      az_보정 = az_원시 - θ

  헬스 동작은 세트 사이에 팔이 내려오므로, 표류가 쌓이는 구간이
  한 번 드는 몇 초로 줄어든다. 분당 16도면 5초에 1.3도다.

  ⚠ 팔을 내릴 때 손바닥 방향(상완 회선)을 매번 비슷하게 두어야 한다.
     차렷에서 손바닥이 허벅지를 향하도록 습관을 들이면 된다.
     한 번에 45도 넘게 튀는 보정은 잘못 잡은 것으로 보고 버린다.

  ⚠ 근본 해결은 여전히 지자계다. 상완/전완 양쪽 지자계를 켜면
     yaw 가 절대 기준에 묶이므로 이 보정 자체가 필요 없어진다.

실행
----
    pip install websockets
    python server_v4.py

    콘솔 명령
        zero    차렷 자세에서 3초 녹화. 그동안 손바닥을 안팎으로 비틀면
                팔 장축까지 함께 잡힌다. 가만히 있으면 차렷 중력방향을 쓴다.
        fwd     앞으로 90도. (교정 완료 후엔 앞 방향만 빠르게 재교정)
        side    옆으로 90도
        diag    (선택) 대각선 90도
        twist   상완 장축만 다시 재측정 (차렷 유지, 3초)
        elbow   팔꿈치 경첩축 측정 (굽혔다 펴기 반복, 4초)
        anchor  지금 이 차렷 자세로 드리프트 앵커를 수동 재설정
        auto    자동 재앵커 켜기/끄기
        sensors 센서 접속 현황
        debug   현재 수치
        reset   교정 초기화
        replay debug_data_2026-08-12.csv     CSV 자가채점
"""

import asyncio
import json
import math
import os
import sys
import time
import traceback

try:
    import websockets
except ImportError:
    websockets = None

PORT = 8765
CALIB_FILE = "calib_R.json"

SENSORS = {
    "shoulder": "R_SHOULDER",
    "upperarm": "R_UPPERARM",
    "forearm":  "R_FOREARM",
}
SENSOR_ORDER = ["R_SHOULDER", "R_UPPERARM", "R_FOREARM"]

FRESH_SEC = 0.5
REST_ELEV_DEG = 20.0
ANCHOR_ELEV_DEG = 15.0     # 이보다 낮으면 차렷으로 보고 앵커 후보
ANCHOR_HOLD_SEC = 0.4      # 이만큼만 유지되면 관측 1건으로 인정 (짧은 휴식도 잡는다)
ANCHOR_MIN_FIT = 3         # 이만큼 관측이 쌓이면 기울기(표류율)를 추정한다
ANCHOR_WINDOW = 1800.0     # 최근 이 시간(초) 안의 관측만 쓴다
ANCHOR_MAX_OBS = 80        # 보관하는 관측 개수. 많을수록 손바닥 잡음이 상쇄된다
ANCHOR_MAX_RATE = 90.0     # 표류율 상한(도/분). 넘으면 추정을 믿지 않는다
TWIST_SEC = 3.0            # 비틀기 교정 녹화 시간
TWIST_MIN_SWEEP = 40.0     # 이만큼은 비틀어야 축이 제대로 나온다
TWIST_MAX_TILT = 45.0      # 차렷 중력축과 이보다 많이 벌어지면 잘못 잡은 것
GRAVITY_EMA_ALPHA = 0.3

latest = {}
connected = set()
gravity_ema = {}

# 센서별 수신 통계 (연결 상태 실시간 표시용)
sensor_stat = {}

calib = {
    "u_ua":    None,   # 차렷 중력방향 (상완센서 프레임). 거상각 기준
    "u_fa":    None,   # 차렷 중력방향 (전완센서 프레임)
    "a_ua":    None,   # 상완 진짜 장축. 비틀기 교정으로 구한다 (없으면 u_ua)
    "a_fa":    None,   # 전완 진짜 장축
    "w_ua":    None,   # 상완 장축에 수직인 고정축. 드리프트 관측용
    "down_sh": None,   # 몸통 아래 (어깨센서 프레임)
    "e_fwd":   None,   # 앞 (어깨센서 프레임, 수평)
    "e_lat":   None,   # 옆 (어깨센서 프레임, 수평)
    "h_ua":    None,   # 팔꿈치 경첩축 (상완센서 프레임). 굽혔다 펴서 측정
    "f0_ua":   None,   # 차렷에서 전완 장축을 상완 프레임에서 본 것
    "p_fa":    None,   # 전완 장축에 수직인 고정축 (전완 프레임). 회내각 측정용
    "p0_ua":   None,   # 차렷에서 그 축을 상완 프레임에서 본 것
    "f0_body": None,   # 차렷에서 전완이 향한 방향 (몸통 프레임)
    "p0_body": None,   # 차렷에서 손바닥 기준축이 향한 방향 (몸통 프레임)
    "d0":      None,   # 차렷에서 팔이 향한 방향 (어깨 프레임). 회선각 기준
    "w0":      None,   # 차렷에서 상완 롤축이 향한 방향 (어깨 프레임)
    "az_side": None,   # 앞->옆 폭(도)
    "az_diag": None,
}

anchor = {
    "enabled": True,
    "ref": None,       # 기준이 되는 롤축 방위(도)
    "theta": 0.0,      # 현재 보정량(도)
    "rate": 0.0,       # 추정된 표류율(도/분)
    "since": None,     # 차렷 유지 시작 시각
    "last": None,      # 마지막 관측 시각
    "obs": [],         # [(시각, 관측된 표류량)] - 휴식 1회당 1건
    "bout": [],        # 현재 휴식 구간에서 모으는 중인 값
    "n": 0,            # 누적 관측 수
}
cal_time = {"t": None}
# 녹화 버퍼. 차렷 영점과 비틀기 재측정이 같이 쓴다.
rec = {"on": False, "mode": None, "t0": 0.0, "until": 0.0,
       "sh": [], "ua": [], "fa": []}


# ── 벡터 / 쿼터니언 ────────────────────────────────────────────

def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qmul(a, b):
    return (
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    )


def qrot(q, v):
    r = qmul(qmul(q, (0.0, v[0], v[1], v[2])), qconj(q))
    return (r[1], r[2], r[3])


def qrel(qa, qb):
    return qmul(qconj(qa), qb)


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])


def scale(v, s):
    return (v[0]*s, v[1]*s, v[2]*s)


def norm(v):
    return math.sqrt(dot(v, v))


def unit(v):
    n = norm(v)
    return (0.0, 0.0, 0.0) if n < 1e-12 else (v[0]/n, v[1]/n, v[2]/n)


def arm_axis():
    """방위각에 쓸 상완 장축. 비틀기 교정을 했으면 그 축, 아니면 차렷 중력축."""
    return calib["a_ua"] or calib["u_ua"]


def fore_axis():
    return calib["a_fa"] or calib["u_fa"]


def perp_to(u):
    """u 에 수직인 결정적(deterministic) 단위벡터. 드리프트 관측축으로 쓴다."""
    a = (1.0, 0.0, 0.0) if abs(u[0]) < 0.9 else (0.0, 1.0, 0.0)
    return unit(sub(a, scale(u, dot(a, u))))


def angle_between(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def quat_about(axis, deg):
    """axis 둘레로 deg 만큼 도는 쿼터니언."""
    r = math.radians(deg) / 2.0
    sn = math.sin(r)
    return (math.cos(r), axis[0]*sn, axis[1]*sn, axis[2]*sn)


def rotate_between(a, b, v):
    """a 를 b 로 보내는 최소 회전을 v 에 적용 (Rodrigues)."""
    a, b = unit(a), unit(b)
    c = max(-1.0, min(1.0, dot(a, b)))
    n = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    sn = norm(n)
    if sn < 1e-9:
        return v if c > 0 else scale(v, -1.0)
    k = unit(n)
    kv = (k[1]*v[2]-k[2]*v[1], k[2]*v[0]-k[0]*v[2], k[0]*v[1]-k[1]*v[0])
    kd = dot(k, v)
    return tuple(v[i]*c + kv[i]*sn + k[i]*kd*(1.0-c) for i in range(3))


def signed_angle(a, b, axis):
    """axis 둘레에서 a 로부터 b 까지의 부호 있는 각(도)."""
    a = unit(sub(a, scale(axis, dot(a, axis))))
    b = unit(sub(b, scale(axis, dot(b, axis))))
    cr = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    return math.degrees(math.atan2(dot(cr, axis), dot(a, b)))


def gravity_in_sensor(q):
    """센서 프레임에서 본 중력. yaw 드리프트와 무관한 유일한 양."""
    return qrot(qconj(q), (0.0, 0.0, 1.0))


def gravity_smoothed(sensor_id, q):
    g = gravity_in_sensor(q)
    prev = gravity_ema.get(sensor_id)
    if prev is None:
        gravity_ema[sensor_id] = g
        return g
    a = GRAVITY_EMA_ALPHA
    g = (a*g[0] + (1-a)*prev[0], a*g[1] + (1-a)*prev[1], a*g[2] + (1-a)*prev[2])
    gravity_ema[sensor_id] = g
    return g


# ── 기하 ───────────────────────────────────────────────────────

def rel_corrected(q_sh, q_ua):
    """
    어깨->상완 상대 회전에서 추정된 yaw 표류를 미리 걷어낸다.
    표류는 연직축 둘레 회전이므로 몸통 아래축(down_sh) 둘레로 되돌려 곱한다.
    (부호는 실측으로 맞춘 것: 이 방향이라야 보정 방위각 = 원시 - θ 가 된다)
    이렇게 하면 방위각과 회선각이 같은 보정을 자동으로 공유한다.
    """
    r = qrel(q_sh, q_ua)
    if calib["down_sh"] is None or abs(anchor["theta"]) < 1e-9:
        return r
    return qmul(quat_about(calib["down_sh"], anchor["theta"]), r)


def arm_roll_deg(q_sh, q_ua):
    """
    상완이 자기 장축 둘레로 얼마나 회선했는가 (차렷 자세 대비, 도).
    스윙-트위스트 분해: 팔이 향하는 방향 변화(스윙)를 걷어낸 나머지가 회선이다.
    손바닥이 어느 쪽을 보는지에 따라 팔꿈치가 접히는 평면이 돌아가는데,
    그 평면의 각도가 바로 이 값이다. (3단계에서 쓴다)
    """
    if calib["d0"] is None or calib["w0"] is None:
        return None
    r = rel_corrected(q_sh, q_ua)
    d = unit(qrot(r, arm_axis()))
    w = unit(qrot(r, calib["w_ua"]))
    w0_swung = rotate_between(calib["d0"], d, calib["w0"])
    return signed_angle(w0_swung, w, d)


def horizontal_azimuth(q_sh, q_ua, axis, corrected=False):
    """
    상완 프레임의 axis 를 몸통 수평면에 투영한 방위각.
    axis = a_ua 면 팔이 가리키는 방향, axis = w_ua 면 드리프트 관측축이다.
    corrected=True 면 표류 보정을 적용한 상대회전을 쓴다.
    (앵커 관측은 보정 전 원시값이어야 하므로 기본은 False)
    """
    if calib["e_fwd"] is None or calib["e_lat"] is None:
        return None, 0.0
    v = qrot(rel_corrected(q_sh, q_ua) if corrected else qrel(q_sh, q_ua), axis)
    down = calib["down_sh"]
    h = sub(v, scale(down, dot(v, down)))
    n = norm(h)
    if n < 1e-6:
        return None, n
    h = unit(h)
    return math.degrees(math.atan2(dot(h, calib["e_lat"]),
                                   dot(h, calib["e_fwd"]))), n


def azimuth_deg(q_sh, q_ua):
    """앞=0도, 옆=+도. 드리프트 보정을 적용한 상대회전에서 잰다."""
    az, mag = horizontal_azimuth(q_sh, q_ua, arm_axis(), corrected=True)
    return (None, mag) if az is None else (wrap180(az), mag)


def elevation_deg(q_ua):
    """
    v3.9 그대로. 차렷 때 상완센서가 느낀 중력방향(= 팔 장축)과
    지금 중력방향 사이의 각. 임계값도 어깨센서도 필요 없고 드리프트도 없다.
    """
    if calib["u_ua"] is None:
        return None
    return angle_between(calib["u_ua"], gravity_smoothed(SENSORS["upperarm"], q_ua))


# 대각선을 안 잰 경우의 기본 경계 (폭에 대한 비율).
# 대각선은 폭의 절반이 아니라 60~70% 지점에 온다 - 견갑골이 옆으로 들 때
# 더 많이 따라 돌기 때문이다. debug_data 실측 구간:
#     앞 -3.0~8.4   대각 21.7~32.2   옆 41.2~51.8   (폭 42.6)
# 빈 공간의 가운데를 잡으면 0.35 / 0.85 다.
BOUND_RATIO_TWIST = (0.35, 0.85)   # 비틀기 교정을 한 경우
BOUND_RATIO_PLAIN = (1/3.0, 2/3.0)  # 차렷 중력축을 쓰는 경우


def bounds():
    s = calib["az_side"]
    if s is None:
        return None
    d = calib["az_diag"]
    if d is not None and 0.0 < d < s:
        return (d/2.0, (d+s)/2.0)      # 대각선을 쟀으면 빈 공간의 중점
    r = BOUND_RATIO_TWIST if calib["a_ua"] else BOUND_RATIO_PLAIN
    return (s*r[0], s*r[1])


def classify(elev, az):
    if elev is None:
        return "UNKNOWN"
    if elev < REST_ELEV_DEG:
        return "REST"
    b = bounds()
    if az is None or b is None:
        return "UNKNOWN"
    s = calib["az_side"]
    if az < -0.35 * s:
        return "뒤/안쪽"
    if az > s * 1.45:
        return "뒤로"
    if az < b[0]:
        return "앞으로 90도"
    if az > b[1]:
        return "옆으로 90도"
    return "대각선 90도"


def elbow_hinge_flex(q_ua, q_fa):
    """
    측정한 경첩축 둘레의 부호 있는 굴곡각.

    v3.4 의 '장축 사이각'은 3차원 전체 각이라 상완-전완 사이 yaw 표류가
    그대로 들어온다. 경첩축 둘레 성분만 재면 축에 수직인 표류는 무시된다.
    상완이 옆으로 누워 경첩축이 수평일 때(대부분의 운동 자세) 연직축 표류는
    경첩축과 거의 직교하므로 굴곡각이 거의 흔들리지 않는다.
    """
    if calib["h_ua"] is None or calib["f0_ua"] is None or calib["a_fa"] is None:
        return None
    f = qrot(qrel(q_ua, q_fa), calib["a_fa"])
    return signed_angle(calib["f0_ua"], f, calib["h_ua"])


def forearm_pron_deg(q_ua, q_fa):
    """
    전완이 자기 장축 둘레로 얼마나 회내/회외했는가 (차렷 대비, 도).
    = 손바닥이 어느 쪽을 보는가.

    팔꿈치가 굽으면 전완 방향 자체가 바뀌므로, 그 방향 변화(스윙)를
    먼저 걷어낸 뒤 남는 성분만 잰다. 상완 프레임에서 계산하므로
    어깨-상완 표류와는 무관하고, 팔꿈치 굴곡과도 분리된다.

    ⚠ 상완 회선(roll)과는 다른 값이다.
       roll  = 상완뼈가 도는 것   -> 팔꿈치 경첩축이 같이 돈다
       pron  = 전완만 도는 것     -> 경첩축은 그대로, 손바닥만 돈다
    """
    if calib["p_fa"] is None or calib["p0_ua"] is None or calib["f0_ua"] is None:
        return None
    r = qrel(q_ua, q_fa)
    f = unit(qrot(r, calib["a_fa"] or calib["u_fa"]))
    p = unit(qrot(r, calib["p_fa"]))
    p0 = rotate_between(calib["f0_ua"], f, calib["p0_ua"])
    return signed_angle(p0, p, f)


def forearm_frame_body(q_sh, q_ua, q_fa):
    """
    전완의 자세를 몸통 프레임에서. 상완을 거쳐 가므로
    어깨-상완 표류는 보정되고 상완-전완 표류만 남는다.
    """
    return qmul(rel_corrected(q_sh, q_ua), qrel(q_ua, q_fa))


def forearm_roll_deg(q_sh, q_ua, q_fa):
    """
    전완이 자기 장축 둘레로 얼마나 돌아 있는가 (차렷 대비, 몸통 기준).
    = 손바닥이 어느 쪽을 보는가. 뷰어는 이 값만큼 손을 비틀면 된다.

    전완이 향하는 방향 변화(스윙)를 먼저 걷어내고 남는 성분만 잰다.
    회내(손목)로 돌든 상완 회선으로 돌든, 몸통 기준으로 실제 손바닥이
    돌아간 각이 그대로 나온다.
    """
    if calib["f0_body"] is None or calib["p_fa"] is None:
        return None
    Q = forearm_frame_body(q_sh, q_ua, q_fa)
    f = unit(qrot(Q, calib["a_fa"] or calib["u_fa"]))
    p = unit(qrot(Q, calib["p_fa"]))
    p0 = rotate_between(calib["f0_body"], f, calib["p0_body"])
    return signed_angle(p0, p, f)


def forearm_dir_body(q_sh, q_ua, flex):
    """
    전완이 몸통 기준 어디를 향하는가. 경첩 모델로 재구성하므로
    상완-전완 표류가 방향에 새어 들어오지 않는다.

    전완방향 = (표류보정된 상완->몸통 회전) x (경첩축 둘레로 flex 만큼 돌린 차렷 전완축)
    뷰어는 이 값을 그대로 그리면 되므로 '접히는 평면'을 추측할 필요가 없다.
    2단계(평면 고정)와 3단계(회선 반영)가 자동으로 한꺼번에 해결된다.
    """
    if calib["h_ua"] is None or calib["f0_ua"] is None or flex is None:
        return None
    f_ua = qrot(quat_about(calib["h_ua"], flex), calib["f0_ua"])
    return unit(qrot(rel_corrected(q_sh, q_ua), f_ua))


def dir_to_body_angles(d):
    """몸통 프레임 단위벡터 -> (연직에서의 각, 앞=0 방위각)."""
    # e_fwd 만 확인하면 안 된다. 'fwd' 를 누르고 'side' 를 누르기 전 구간에서는
    # e_lat 이 아직 None 이라 아래 내적이 터진다. (연결 끊김의 원인이었다)
    if d is None or calib["e_fwd"] is None or calib["e_lat"] is None:
        return None, None
    down = calib["down_sh"]
    elev = angle_between(d, down)
    h = sub(d, scale(down, dot(d, down)))
    if norm(h) < 1e-6:
        return elev, 0.0
    h = unit(h)
    return elev, math.degrees(math.atan2(dot(h, calib["e_lat"]), dot(h, calib["e_fwd"])))


def elbow_flex(q_ua, q_fa):
    """v3.4 방식. 장축 사이각을 중력이 허용하는 범위로 자른다."""
    if calib["u_ua"] is None or calib["u_fa"] is None:
        return None, 0.0
    t_ua = angle_between(gravity_smoothed(SENSORS["upperarm"], q_ua), calib["u_ua"])
    t_fa = angle_between(gravity_smoothed(SENSORS["forearm"],  q_fa), calib["u_fa"])
    # v3.4 그대로. 장축을 차렷 중력축으로 두어야 차렷에서 정확히 0도가 된다.
    # (비틀기 축을 쓰면 차렷에서 0이 안 나와 '편 상태' 규약이 깨진다)
    raw = angle_between(qrot(q_ua, calib["u_ua"]), qrot(q_fa, calib["u_fa"]))
    lo = abs(t_fa - t_ua)
    s = t_fa + t_ua
    hi = min(s, 360.0 - s)
    if hi < lo:
        lo, hi = hi, lo
    flex = max(lo, min(hi, raw))
    return flex, abs(raw - flex)


# ── 비틀기 교정: 상완 진짜 장축 구하기 ────────────────────────
#
# 차렷 중력방향을 팔 장축으로 쓰면, 팔이 완전히 수직으로 안 떨어질 때
# 그 오차만큼 장축이 어긋난다. 그러면 손목/상완을 비트는 회전이
# 장축 둘레 회전으로 상쇄되지 않고 방위각에 새어 들어온다.
#
#   debug_data 실측: 차렷 중력축 기준 -> 손목 3종 사이 흩어짐 RMS 7.3도
#                    비틀기 축 기준   -> 흩어짐 RMS 4.5도, 두 축 차이 16.6도
#
#   교정을 어느 손목 자세로 했는지에 따른 정확도 (교정에 쓴 자세는 제외하고 채점)
#       차렷 중력축   몸쪽 100.0%   앞쪽  71.2%   뒤쪽  84.4%
#       비틀기 축     몸쪽 100.0%   앞쪽 100.0%   뒤쪽 100.0%
#
# 팔을 내린 채 손바닥을 안팎으로 비틀면, 그 회전의 축이 곧 팔 장축이다.
# 센서가 어떻게 붙었든 상관없이 회전축은 물리적으로 팔을 따라간다.

def axis_from_stream_pre(qs, stride=8):
    """
    axis_from_stream 과 같지만 회전을 앞쪽에 곱해서 축을 '기준 프레임'에서 얻는다.
    r_i 가 B->A 사상일 때 r_j = D * r_i 이므로 D = r_j * conj(r_i) 는 A 프레임의 회전이다.
    팔꿈치 경첩축을 상완 프레임에서 얻을 때 쓴다.
    """
    acc = [0.0, 0.0, 0.0]
    sweep = 0.0
    ref = None
    for i in range(0, len(qs) - stride, stride):
        dq = qmul(qs[i + stride], qconj(qs[i]))
        if dq[0] < 0:
            dq = tuple(-c for c in dq)
        v = (dq[1], dq[2], dq[3])
        n = norm(v)
        if n < 1e-9:
            continue
        ang = math.degrees(2 * math.atan2(n, dq[0]))
        if ang < 0.5:
            continue
        ax = unit(v)
        if ref is None:
            ref = ax
        if dot(ax, ref) < 0:
            ax = scale(ax, -1.0)
        acc = [acc[j] + ax[j] * ang for j in range(3)]
        sweep += ang
    if sweep < 1e-6:
        return None, 0.0
    return unit(tuple(acc)), sweep


def axis_from_stream(qs, stride=8):
    """
    연속된 쿼터니언에서 공통 회전축을 뽑는다.
    각 구간의 회전축을 회전량으로 가중해 더한 뒤 정규화한다.
    반환: (축, 총 회전량(도))
    """
    acc = [0.0, 0.0, 0.0]
    sweep = 0.0
    ref = None
    for i in range(0, len(qs) - stride, stride):
        dq = qrel(qs[i], qs[i + stride])
        if dq[0] < 0:
            dq = tuple(-c for c in dq)
        v = (dq[1], dq[2], dq[3])
        n = norm(v)
        if n < 1e-9:
            continue
        ang = math.degrees(2 * math.atan2(n, dq[0]))
        if ang < 0.5:                 # 노이즈 구간은 버린다
            continue
        ax = unit(v)
        if ref is None:
            ref = ax
        if dot(ax, ref) < 0:          # 왕복 운동이라 부호가 뒤집힌다
            ax = scale(ax, -1.0)
        acc = [acc[j] + ax[j] * ang for j in range(3)]
        sweep += ang
    if sweep < 1e-6:
        return None, 0.0
    return unit(tuple(acc)), sweep


REC_SEC = {"zero": 3.0, "twist": 3.0, "elbow": 4.0}
REC_MSG = {
    "zero":  ("팔을 내린 채로 손바닥을 안쪽<->바깥쪽으로 두세 번 크게 비트세요.",
              "(가만히 서 있어도 됩니다. 그러면 차렷 중력방향을 장축으로 씁니다.)"),
    "twist": ("팔을 내린 채로 손바닥을 안쪽<->바깥쪽으로 두세 번 크게 비트세요.",
              "(상완 장축만 다시 잡습니다.)"),
    "elbow": ("팔꿈치를 완전히 폈다가 최대로 굽히기를 두세 번 반복하세요.",
              "(상완은 몸에 붙이고, 손목은 돌리지 말고 고정한 채로.)"),
}


def start_rec(mode):
    """차렷 영점(zero) / 상완 장축(twist) / 팔꿈치 경첩축(elbow) 녹화."""
    if mode != "zero" and calib["u_ua"] is None:
        print("[x] 먼저 'zero'(차렷)를 하세요.")
        return False
    if mode == "elbow" and calib["a_fa"] is None and calib["u_fa"] is None:
        print("[x] 전완 센서 데이터가 없습니다.")
        return False
    now = time.time()
    dur = REC_SEC.get(mode, 3.0)
    rec.update({"on": True, "mode": mode, "t0": now, "until": now + dur,
                "sh": [], "ua": [], "fa": []})
    a, b = REC_MSG[mode]
    print(f"[..] {dur:.0f}초간 녹화합니다. {a}")
    print(f"     {b}")
    return True


def _settle_mean(stream, t0, span=0.5):
    """녹화 앞부분(아직 비틀기 전) 평균 쿼터니언. 정지 자세 기준값으로 쓴다."""
    head = [q for t, q in stream if t - t0 <= span] or [q for _, q in stream[:5]]
    if not head:
        return None
    m = tuple(sum(q[i] for q in head)/len(head) for i in range(4))
    n = math.sqrt(sum(c*c for c in m))
    return None if n < 1e-9 else tuple(c/n for c in m)


def _long_axis(stream, u_ref, label):
    """녹화된 비틀기에서 장축을 뽑는다. 조건 미달이면 None."""
    if len(stream) < 10:
        return None
    a, sweep = axis_from_stream([q for _, q in stream])
    if a is None or sweep < TWIST_MIN_SWEEP:
        return None
    if dot(a, u_ref) < 0:
        a = scale(a, -1.0)
    tilt = angle_between(a, u_ref)
    if tilt > TWIST_MAX_TILT:
        print(f"[!] {label} 장축이 차렷 방향과 {tilt:.0f}도나 벌어져 무시했습니다. "
              "팔을 들지 말고 제자리에서 비틀기만 하세요.")
        return None
    print(f"[ok] {label} 장축 확정 {tuple(round(c,3) for c in a)} "
          f"(총 {sweep:.0f}도 비틀림, 차렷 중력축과 {tilt:.1f}도 차이)")
    return a


def _pair_stream():
    """상완/전완 샘플을 시각으로 짝지어 상대회전 열을 만든다."""
    fa = rec["fa"]
    if not fa:
        return []
    out, j = [], 0
    for t, qu in rec["ua"]:
        while j + 1 < len(fa) and abs(fa[j + 1][0] - t) <= abs(fa[j][0] - t):
            j += 1
        if abs(fa[j][0] - t) < 0.08:
            out.append(qrel(qu, fa[j][1]))
    return out


def finish_rec():
    mode = rec["mode"]
    rec["on"] = False
    t0 = rec["t0"]

    if mode == "elbow":
        rels = _pair_stream()
        if len(rels) < 15:
            print("[x] 경첩축 측정 실패: 상완/전완 샘플이 부족합니다.")
            return
        h, sweep = axis_from_stream_pre(rels)
        if h is None or sweep < 60.0:
            print(f"[x] 경첩축 측정 실패: 회전량 {sweep:.0f}도 (60도 이상 필요). "
                  "더 크게 굽혔다 펴세요.")
            return
        # 부호: 굽힘이 양수가 되도록 맞춘다
        f0 = qrot(rels[0], calib["a_fa"] or calib["u_fa"])
        ext = max(rels, key=lambda r: angle_between(
            qrot(r, calib["a_fa"] or calib["u_fa"]), f0))
        if signed_angle(f0, qrot(ext, calib["a_fa"] or calib["u_fa"]), h) < 0:
            h = scale(h, -1.0)
        calib["h_ua"] = h
        tilt = angle_between(h, arm_axis())
        print(f"[ok] 팔꿈치 경첩축 확정 {tuple(round(c,3) for c in h)} "
              f"(총 {sweep:.0f}도 굴곡, 상완 장축과 {tilt:.1f}도)")
        if abs(tilt - 90.0) > 30.0:
            print(f"     [!] 장축과 {tilt:.0f}도입니다. 보통 90도 근처여야 합니다. "
                  "상완을 몸에 붙이고 팔꿈치만 움직여 다시 재보세요.")
        save_calib()
        return

    if mode == "twist":
        a = _long_axis(rec["ua"], calib["u_ua"], "상완")
        if a is None:
            print("[x] 장축 재측정 실패: 더 크게(합계 40도 이상) 비틀고 다시 하세요.")
            return
        calib["a_ua"] = a
        calib["w_ua"] = perp_to(a)
        q0s = _settle_mean(rec["sh"], t0) or fresh(SENSORS["shoulder"])
        q0u = _settle_mean(rec["ua"], t0) or fresh(SENSORS["upperarm"])
        if q0s and q0u:
            r0 = qrel(q0s, q0u)
            calib["d0"] = unit(qrot(r0, a))
            calib["w0"] = unit(qrot(r0, calib["w_ua"]))
        if calib["u_fa"]:
            calib["a_fa"] = _long_axis(rec["fa"], calib["u_fa"], "전완") or calib["a_fa"]
        anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                       "last": None, "obs": [], "bout": [], "n": 0})
        if calib["e_lat"] is not None:
            print("     [!] 장축이 바뀌었으니 fwd / side / diag 를 다시 잡으세요.")
            calib["e_fwd"] = calib["e_lat"] = None
            calib["az_side"] = calib["az_diag"] = None
        save_calib()
        return

    # mode == "zero"
    q_sh = _settle_mean(rec["sh"], t0)
    q_ua = _settle_mean(rec["ua"], t0)
    q_fa = _settle_mean(rec["fa"], t0)
    if not q_sh or not q_ua:
        print("[x] 영점 실패: 어깨/상완 센서 데이터를 받지 못했습니다.")
        return

    calib["u_ua"] = unit(gravity_in_sensor(q_ua))
    calib["down_sh"] = unit(gravity_in_sensor(q_sh))
    calib["u_fa"] = unit(gravity_in_sensor(q_fa)) if q_fa else None
    calib["a_ua"] = _long_axis(rec["ua"], calib["u_ua"], "상완")
    calib["a_fa"] = (_long_axis(rec["fa"], calib["u_fa"], "전완")
                     if calib["u_fa"] else None)
    calib["w_ua"] = perp_to(arm_axis())
    r0 = qrel(q_sh, q_ua)
    calib["d0"] = unit(qrot(r0, arm_axis()))
    calib["w0"] = unit(qrot(r0, calib["w_ua"]))
    calib["h_ua"] = None
    if calib["a_fa"] or calib["u_fa"]:
        r0f = qrel(q_ua, q_fa)
        calib["p_fa"] = perp_to(calib["a_fa"] or calib["u_fa"])
        calib["f0_ua"] = unit(qrot(r0f, calib["a_fa"] or calib["u_fa"]))
        calib["p0_ua"] = unit(qrot(r0f, calib["p_fa"]))
        r0b = qrel(q_sh, q_fa)
        calib["f0_body"] = unit(qrot(r0b, calib["a_fa"] or calib["u_fa"]))
        calib["p0_body"] = unit(qrot(r0b, calib["p_fa"]))
    calib["e_fwd"] = calib["e_lat"] = None
    calib["az_side"] = calib["az_diag"] = None
    gravity_ema.clear()
    cal_time["t"] = None
    anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                   "last": None, "obs": [], "bout": [], "n": 0})

    print(f"[ok] 차렷 영점 완료. 차렷 중력축 {tuple(round(c,3) for c in calib['u_ua'])}")
    if calib["a_ua"] is None:
        print("     [!] 비틀기가 감지되지 않아 차렷 중력방향을 장축으로 씁니다.")
        print("         이 경우 fwd/side 를 반드시 '손바닥이 허벅지를 향한 상태'로 잡으세요.")
    print("     이제 팔을 앞으로 90도 들고 'fwd' 를 입력하세요.")
    save_calib()


# ── 드리프트 자동 재앵커 ───────────────────────────────────────

def _fit_drift(now):
    """
    관측된 표류량들에 직선을 맞춘다.

    자이로 바이어스에서 오는 표류는 시간에 거의 비례한다. 반면
    차렷 때 손바닥을 어떻게 두었느냐(상완 회선)는 매번 제각각인 잡음이다.
    관측 하나를 그대로 믿으면 그 잡음이 그대로 들어오지만, 여러 관측에
    직선을 맞추면 잡음은 상쇄되고 기울기만 남는다.
    관측이 끊긴 뒤에도 이 직선을 연장해 계속 보정할 수 있다.
    """
    obs = [(t, v) for t, v in anchor["obs"] if now - t <= ANCHOR_WINDOW]
    anchor["obs"] = obs[-ANCHOR_MAX_OBS:]
    obs = anchor["obs"]
    if not obs:
        return
    if len(obs) < ANCHOR_MIN_FIT:
        anchor["rate"] = 0.0
        anchor["theta"] = sum(v for _, v in obs) / len(obs)
        return

    n = len(obs)
    mt = sum(t for t, _ in obs) / n
    mv = sum(v for _, v in obs) / n
    sxx = sum((t - mt) ** 2 for t, _ in obs)
    sxy = sum((t - mt) * (v - mv) for t, v in obs)
    slope = sxy / sxx if sxx > 1e-6 else 0.0
    if abs(slope) * 60.0 > ANCHOR_MAX_RATE:        # 말이 안 되는 기울기는 버린다
        slope = 0.0
    anchor["rate"] = slope * 60.0
    anchor["theta"] = mv + slope * (now - mt)


def update_anchor(elev, q_sh, q_ua, now):
    """
    팔이 내려와 있으면(참값을 아는 자세) 그때의 표류량을 관측으로 남긴다.

    팔이 아래를 향할 때 상완센서의 롤축(w_ua)은 몸통 수평면에 놓인다.
    그 방위가 기준(anchor["ref"])에서 벗어난 만큼이 표류량 관측치다.
    관측치 하나하나는 손바닥 방향 때문에 흔들리므로 그대로 쓰지 않고,
    휴식 한 번당 중앙값 1건으로 줄인 뒤 _fit_drift 로 직선을 맞춘다.
    """
    if not anchor["enabled"] or not ready():
        return

    if elev is None or elev > ANCHOR_ELEV_DEG:
        # 휴식이 끝났다. 모은 값이 있으면 중앙값 1건으로 확정한다.
        if anchor["bout"]:
            vals = sorted(anchor["bout"])
            anchor["obs"].append((anchor["since"] or now, vals[len(vals) // 2]))
            anchor["n"] += 1
            anchor["last"] = now
            anchor["bout"] = []
        anchor["since"] = None
        if anchor["obs"]:
            _fit_drift(now)          # 관측이 끊겨도 기울기로 계속 외삽한다
        return

    if anchor["since"] is None:
        anchor["since"] = now
        return
    if now - anchor["since"] < ANCHOR_HOLD_SEC:
        return

    az_w, mag = horizontal_azimuth(q_sh, q_ua, calib["w_ua"])
    if az_w is None or mag < 0.5:
        return

    if anchor["ref"] is None:
        anchor.update({"ref": az_w, "theta": 0.0, "rate": 0.0,
                       "obs": [(now, 0.0)], "n": 1, "last": now})
        print("\n[ok] 드리프트 앵커 설정. 이후 팔을 내릴 때마다 표류를 추정합니다.")
        return

    # 직전 추정값 근처로 풀어서(unwrap) ±180 경계에서 튀지 않게 한다
    anchor["bout"].append(
        anchor["theta"] + wrap180(az_w - anchor["ref"] - anchor["theta"]))

def reset_anchor(q_sh=None, q_ua=None):
    q_sh = q_sh or fresh(SENSORS["shoulder"])
    q_ua = q_ua or fresh(SENSORS["upperarm"])
    if not ready() or not q_sh or not q_ua:
        print("[x] 앵커 실패: 교정을 먼저 끝내세요.")
        return False
    az_w, mag = horizontal_azimuth(q_sh, q_ua, calib["w_ua"])
    if az_w is None or mag < 0.5:
        print("[x] 앵커 실패: 팔을 내린 차렷 자세에서 다시 시도하세요.")
        return False
    now = time.time()
    anchor.update({"ref": az_w, "theta": 0.0, "rate": 0.0,
                   "since": None, "last": now, "obs": [(now, 0.0)],
                   "bout": [], "n": 1})
    print(f"[ok] 드리프트 앵커 재설정 (기준 {az_w:.1f}도)")
    return True


# ── 교정 ───────────────────────────────────────────────────────

def fresh(sensor_id, now=None):
    now = now or time.time()
    d = latest.get(sensor_id)
    if d and (now - d["t"] < FRESH_SEC):
        return (d["qw"], d["qx"], d["qy"], d["qz"])
    return None


def capture_rest(q_sh=None, q_ua=None, q_fa=None):
    """정지 쿼터니언으로 즉시 영점. (replay 및 비틀기 없이 쓸 때의 경로)"""
    q_sh = q_sh or fresh(SENSORS["shoulder"])
    q_ua = q_ua or fresh(SENSORS["upperarm"])
    q_fa = q_fa or fresh(SENSORS["forearm"])
    if not q_sh or not q_ua:
        print("[x] 영점 실패: 어깨/상완 센서 데이터를 기다리는 중")
        return False
    calib["u_ua"] = unit(gravity_in_sensor(q_ua))
    calib["down_sh"] = unit(gravity_in_sensor(q_sh))
    calib["u_fa"] = unit(gravity_in_sensor(q_fa)) if q_fa else None
    calib["a_ua"] = calib["a_fa"] = None      # 비틀기 교정은 다시 해야 한다
    calib["w_ua"] = perp_to(calib["u_ua"])
    r0 = qrel(q_sh, q_ua)
    calib["d0"] = unit(qrot(r0, calib["u_ua"]))
    calib["w0"] = unit(qrot(r0, calib["w_ua"]))
    calib["h_ua"] = None
    if q_fa and calib["u_fa"]:
        r0f = qrel(q_ua, q_fa)
        calib["p_fa"] = perp_to(calib["u_fa"])
        calib["f0_ua"] = unit(qrot(r0f, calib["u_fa"]))
        calib["p0_ua"] = unit(qrot(r0f, calib["p_fa"]))
        r0b = qrel(q_sh, q_fa)
        calib["f0_body"] = unit(qrot(r0b, calib["u_fa"]))
        calib["p0_body"] = unit(qrot(r0b, calib["p_fa"]))
    calib["e_fwd"] = calib["e_lat"] = None
    calib["az_side"] = calib["az_diag"] = None
    gravity_ema.clear()
    cal_time["t"] = None
    anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                   "last": None, "obs": [], "bout": [], "n": 0})
    print(f"[ok] 차렷 영점 완료. 상완 장축 {tuple(round(c,3) for c in calib['u_ua'])}")
    print("     이제 팔을 앞으로 90도 들고 'fwd' 를 입력하세요.")
    return True


def capture_dir(kind, q_sh=None, q_ua=None):
    if calib["u_ua"] is None:
        print("[x] 먼저 'zero'(차렷)를 하세요.")
        return False
    q_sh = q_sh or fresh(SENSORS["shoulder"])
    q_ua = q_ua or fresh(SENSORS["upperarm"])
    if not q_sh or not q_ua:
        print("[x] 교정 실패: 센서 데이터를 기다리는 중")
        return False

    d = unit(qrot(qrel(q_sh, q_ua), arm_axis()))
    down = calib["down_sh"]
    h = sub(d, scale(down, dot(d, down)))
    mag = norm(h)
    if mag < 0.35:
        print(f"[x] 팔이 충분히 들리지 않았습니다 (수평성분 {mag:.2f}). 90도로 들고 다시.")
        return False
    h = unit(h)

    if kind == "forward":
        # 교정이 끝나 있으면 빠른 재교정. 폭은 유지하고 앞 방향만 다시 잡는다.
        if calib["e_lat"] is not None and calib["az_side"] is not None:
            drift = math.degrees(math.atan2(dot(h, calib["e_lat"]),
                                            dot(h, calib["e_fwd"])))
            new_lat = unit(sub(calib["e_lat"], scale(h, dot(calib["e_lat"], h))))
            if norm(new_lat) < 1e-6:
                print("[x] 재교정 실패: 정면으로 뻗고 다시 시도하세요.")
                return False
            calib["e_fwd"] = h
            calib["e_lat"] = new_lat
            anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                           "last": None, "obs": [], "bout": [], "n": 0})
            cal_time["t"] = time.time()
            print(f"[ok] 앞 방향 재교정. 지난 교정 이후 {drift:+.1f}도 틀어져 있었습니다 "
                  f"(폭 {calib['az_side']:.1f}도 유지). 앵커는 다음 차렷에서 다시 잡힙니다.")
            save_calib()
            return True

        calib["e_fwd"] = h
        calib["e_lat"] = calib["az_side"] = calib["az_diag"] = None
        print("[ok] 앞 방향 기준 설정. 이제 옆으로 90도 들고 'side' 를 입력하세요.")
        return True

    if calib["e_fwd"] is None:
        print("[x] 먼저 'fwd'(앞으로 90도)를 하세요.")
        return False

    if kind == "side":
        lat = unit(sub(h, scale(calib["e_fwd"], dot(h, calib["e_fwd"]))))
        if norm(lat) < 1e-6:
            print("[x] 앞 방향과 구분되지 않습니다. 옆으로 확실히 벌리고 다시.")
            return False
        calib["e_lat"] = lat
        calib["az_side"] = math.degrees(
            math.atan2(dot(h, lat), dot(h, calib["e_fwd"])))
        b = bounds()
        print(f"[ok] 옆 방향 기준 설정. 앞->옆 폭 {calib['az_side']:.1f}도, "
              f"경계 {b[0]:.1f} / {b[1]:.1f}")
        if calib["az_side"] < 20.0:
            print("     [!] 폭이 20도 미만입니다. 앞/옆 자세를 더 뚜렷하게 잡고 다시 교정하세요.")
        print("     대각선 90도에서 'diag' 까지 잡으면 경계가 실측 중점으로 바뀝니다 (권장).")
        print("     팔을 내리고 1초만 가만히 있으면 드리프트 앵커가 자동으로 잡힙니다.")
        anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                       "last": None, "obs": [], "bout": [], "n": 0})
        cal_time["t"] = time.time()
        save_calib()
        return True

    if kind == "diag":
        if calib["e_lat"] is None:
            print("[x] 먼저 'side'(옆으로 90도)를 하세요.")
            return False
        calib["az_diag"] = math.degrees(
            math.atan2(dot(h, calib["e_lat"]), dot(h, calib["e_fwd"])))
        b = bounds()
        print(f"[ok] 대각선 기준 {calib['az_diag']:.1f}도 "
              f"(폭의 {100*calib['az_diag']/calib['az_side']:.0f}%). "
              f"경계 {b[0]:.1f} / {b[1]:.1f}")
        save_calib()
        return True

    return False


def calib_state():
    return {
        "rest": calib["u_ua"] is not None,
        "fwd":  calib["e_fwd"] is not None,
        "side": calib["e_lat"] is not None,
        "diag": calib["az_diag"] is not None,
        "twist": calib["a_ua"] is not None,
        "elbow": calib["h_ua"] is not None,
        "anchor": anchor["ref"] is not None,
    }


def ready():
    return calib["u_ua"] is not None and calib["e_lat"] is not None


def save_calib():
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump({k: (list(v) if isinstance(v, tuple) else v)
                       for k, v in calib.items()}, f, indent=2)
    except OSError as e:
        print(f"[x] 저장 실패: {e}")


def load_calib():
    if not os.path.exists(CALIB_FILE):
        return
    try:
        with open(CALIB_FILE) as f:
            d = json.load(f)
        for k in calib:
            v = d.get(k)
            calib[k] = tuple(v) if isinstance(v, list) else v
        if calib["u_ua"] is not None:
            calib["w_ua"] = perp_to(arm_axis())
        if ready():
            b = bounds()
            print(f"[ok] 교정값 불러옴. 앞->옆 폭 {calib['az_side']:.1f}도, "
                  f"경계 {b[0]:.1f} / {b[1]:.1f}")
            print("     [!] 센서를 다시 붙였다면 반드시 zero 부터 다시 하세요.")
    except (OSError, ValueError) as e:
        print(f"[x] 불러오기 실패: {e}")


def reset_calib():
    for k in calib:
        calib[k] = None
    cal_time["t"] = None
    anchor.update({"ref": None, "theta": 0.0, "rate": 0.0, "since": None,
                   "last": None, "obs": [], "bout": [], "n": 0})
    gravity_ema.clear()
    print("[ok] 교정 초기화. 'zero' 부터 다시 하세요.")


# ── 센서 접속 현황 ─────────────────────────────────────────────

def note_sample(sensor_id, t):
    st = sensor_stat.setdefault(sensor_id, {"t": 0.0, "n": 0, "hz": 0.0,
                                            "_wt": t, "_wn": 0, "first": t})
    st["t"] = t
    st["n"] += 1
    st["_wn"] += 1
    dt = t - st["_wt"]
    if dt >= 1.0:                      # 1초 창으로 수신율 갱신
        st["hz"] = st["_wn"] / dt
        st["_wt"] = t
        st["_wn"] = 0


def sensor_report(now=None):
    now = now or time.time()
    out = []
    ids = SENSOR_ORDER + [i for i in sorted(sensor_stat) if i not in SENSOR_ORDER]
    for sid in ids:
        st = sensor_stat.get(sid)
        if st is None:
            out.append({"id": sid, "known": True, "ok": False,
                        "hz": 0.0, "age": None, "n": 0})
            continue
        age = now - st["t"]
        if age > 2.0:
            st["hz"] = 0.0
        out.append({"id": sid, "known": sid in SENSOR_ORDER,
                    "ok": age < FRESH_SEC, "hz": round(st["hz"], 1),
                    "age": round(age, 1), "n": st["n"]})
    return out


def sensor_line(now=None):
    parts = []
    for s in sensor_report(now):
        short = s["id"].replace("R_", "")
        if s["ok"]:
            parts.append(f"O {short} {s['hz']:4.1f}Hz")
        elif s["age"] is None:
            parts.append(f"X {short} 없음     ")
        else:
            parts.append(f"! {short} {s['age']:4.1f}s전")
    tail = f" | 보정 {anchor['theta']:+5.1f}도" if ready() else ""
    return "  ".join(parts) + tail


# ── 상태 산출 ──────────────────────────────────────────────────

def analyze():
    now = time.time()
    q_sh = fresh(SENSORS["shoulder"], now)
    q_ua = fresh(SENSORS["upperarm"], now)
    q_fa = fresh(SENSORS["forearm"], now)

    out = {
        "status": "OK",
        "pose": "UNKNOWN",
        "elev": 0.0, "az": 0.0, "mag": 0.0,
        "flex": None, "flex_drift": 0.0, "roll": None,
        "flex_hinge": None, "fa_elev": None, "fa_az": None, "pron": None, "fa_roll": None,
        "az_side": calib["az_side"],
        "drift": round(anchor["theta"], 1) if ready() else None,
        "anchor_age": (None if anchor["last"] is None
                       else round(now - anchor["last"])),
        "auto_anchor": anchor["enabled"],
        "drift_rate": round(anchor["rate"], 1) if ready() else None,
        "anchor_n": anchor["n"],
        "recording": rec["on"],
        "cal_age": (None if cal_time["t"] is None else round(now - cal_time["t"])),
        "calib": calib_state(),
        "sensors": sensor_report(now),
        "quat": {},
    }

    missing = [k for k, v in (("어깨", q_sh), ("상완", q_ua)) if v is None]
    if missing:
        out["status"] = "센서 대기: " + ", ".join(missing)
        return out
    if rec["on"]:
        out["status"] = (f"녹화 중 {max(0.0, rec['until']-now):.1f}초 - "
                         "팔 내린 채 손바닥을 안팎으로 크게 비트세요")
    if calib["u_ua"] is None:
        out["status"] = "차렷 영점(zero) 필요"
        return out

    out["quat"] = {"sh": list(q_sh), "ua": list(q_ua),
                   "fa": list(q_fa) if q_fa else None}

    elev = elevation_deg(q_ua)
    update_anchor(elev, q_sh, q_ua, now)
    az, mag = azimuth_deg(q_sh, q_ua)
    out["elev"] = round(elev, 2)
    out["mag"] = round(mag, 3)
    if az is not None:
        out["az"] = round(az, 2)
    out["drift"] = round(anchor["theta"], 1) if ready() else None

    roll = arm_roll_deg(q_sh, q_ua)
    out["roll"] = None if roll is None else round(roll, 1)

    if q_fa and calib["u_fa"]:
        flex, fdrift = elbow_flex(q_ua, q_fa)          # v3.4 중력 클램프 방식
        out["flex"] = round(flex, 2)
        out["flex_drift"] = round(fdrift, 2)

        pr = forearm_pron_deg(q_ua, q_fa)
        out["pron"] = None if pr is None else round(pr, 1)

        fr = forearm_roll_deg(q_sh, q_ua, q_fa)
        out["fa_roll"] = None if fr is None else round(fr, 1)

        hf = elbow_hinge_flex(q_ua, q_fa)              # 경첩축 방식
        if hf is not None:
            out["flex_hinge"] = round(hf, 2)

        # 전완 방향: 경첩 교정이 있으면 모델로, 없으면 센서에서 직접
        fdir = (forearm_dir_body(q_sh, q_ua, hf) if hf is not None
                else unit(qrot(forearm_frame_body(q_sh, q_ua, q_fa),
                               calib["a_fa"] or calib["u_fa"])))
        fe, fa_az = dir_to_body_angles(fdir)
        if fe is not None:
            out["fa_elev"] = round(fe, 2)
            out["fa_az"] = round(fa_az, 2)

    if not ready():
        out["pose"] = "REST" if elev < REST_ELEV_DEG else "UNKNOWN"
        out["status"] = "방향 교정 필요 (fwd -> side)"
        return out

    out["pose"] = classify(elev, az)
    if anchor["ref"] is None:
        out["status"] = "팔을 내리고 1초 정지하면 드리프트 앵커가 잡힙니다"
    return out


# ── CSV 자가채점 ───────────────────────────────────────────────

def replay(path):
    import csv as _csv
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in _csv.DictReader(f):
            try:
                rows.append({
                    "pose": r["pose"],
                    "sh": (float(r["sh_w"]), float(r["sh_x"]), float(r["sh_y"]), float(r["sh_z"])),
                    "ua": (float(r["ua_w"]), float(r["ua_x"]), float(r["ua_y"]), float(r["ua_z"])),
                    "fa": (float(r["fa_w"]), float(r["fa_x"]), float(r["fa_y"]), float(r["fa_z"])),
                })
            except (KeyError, ValueError):
                continue
    if not rows:
        print("[x] 읽을 수 있는 행이 없습니다.")
        return

    def mean_q(sel, key):
        s = [r[key] for r in rows if sel(r["pose"])]
        if not s:
            return None
        m = tuple(sum(q[i] for q in s)/len(s) for i in range(4))
        n = math.sqrt(sum(c*c for c in m))
        return tuple(c/n for c in m)

    def mean_pose(tag, key):
        return mean_q(lambda p: p == tag, key)

    def twist_axis_from_csv():
        """
        CSV 안에 같은 자세를 손바닥 방향만 바꿔 찍은 행이 있으면,
        그 사이의 상대 회전축이 곧 팔 장축이다. 실측 비틀기와 같은 원리.
        """
        wrists = ["손바닥뒤쪽", "손바닥몸쪽", "손바닥앞쪽"]
        acc, sweep, ref = [0.0, 0.0, 0.0], 0.0, None
        for base in ("앞으로90", "대각선90", "옆으로90"):
            qs = [mean_pose(f"{base}_{w}", "ua") for w in wrists]
            qs = [q for q in qs if q]
            for i in range(len(qs)):
                for j in range(i + 1, len(qs)):
                    dq = qrel(qs[i], qs[j])
                    if dq[0] < 0:
                        dq = tuple(-c for c in dq)
                    v = (dq[1], dq[2], dq[3])
                    n = norm(v)
                    if n < 1e-9:
                        continue
                    ang = math.degrees(2 * math.atan2(n, dq[0]))
                    ax = unit(v)
                    if ref is None:
                        ref = ax
                    if dot(ax, ref) < 0:
                        ax = scale(ax, -1.0)
                    acc = [acc[k] + ax[k] * ang for k in range(3)]
                    sweep += ang
        return (unit(tuple(acc)), sweep) if sweep > 1e-6 else (None, 0.0)

    saved, saved_anchor = dict(calib), dict(anchor)
    reset_calib()
    anchor["enabled"] = False        # 채점 중에는 자동 보정을 끈다

    is_rest = lambda p: p.startswith("REST")
    if mean_q(is_rest, "ua") is None:
        print("[!] 이 CSV 에는 REST 행이 없어 영점을 잡을 수 없습니다.")
        calib.update(saved); anchor.update(saved_anchor)
        return
    capture_rest(mean_q(is_rest, "sh"), mean_q(is_rest, "ua"), mean_q(is_rest, "fa"))

    for kind, tag in (("forward", "앞으로90_손바닥몸쪽"), ("side", "옆으로90_손바닥몸쪽")):
        sel = (lambda t: (lambda p: p == t))(tag)
        qs, qu = mean_q(sel, "sh"), mean_q(sel, "ua")
        if qs is None:
            print(f"[x] '{tag}' 행이 없습니다.")
            calib.update(saved); anchor.update(saved_anchor)
            return
        capture_dir(kind, qs, qu)

    def truth(p):
        if p.startswith("REST"):
            return "REST"
        if p.startswith("앞으로"):
            return "앞으로 90도"
        if p.startswith("대각선"):
            return "대각선 90도"
        return "옆으로 90도"

    axis, sweep = twist_axis_from_csv()
    if axis is not None and sweep >= TWIST_MIN_SWEEP:
        if dot(axis, calib["u_ua"]) < 0:
            axis = scale(axis, -1.0)
        tilt = angle_between(axis, calib["u_ua"])
        if tilt <= TWIST_MAX_TILT:
            calib["a_ua"] = axis
            calib["w_ua"] = perp_to(axis)
            # 장축이 바뀌면 폭도 달라진다. 방향 교정을 처음부터 다시 잡아야 한다.
            calib["e_fwd"] = calib["e_lat"] = None
            calib["az_side"] = calib["az_diag"] = None
            print(f"[ok] CSV 안의 손목 3종에서 팔 장축 추정 "
                  f"(차렷 중력축과 {tilt:.1f}도 차이). 방향 교정을 다시 잡습니다.")
            for kind, tag in (("forward", "앞으로90_손바닥몸쪽"),
                              ("side", "옆으로90_손바닥몸쪽"),
                              ("diag", "대각선90_손바닥몸쪽")):
                sel = (lambda t: (lambda p: p == t))(tag)
                if mean_q(sel, "sh"):
                    capture_dir(kind, mean_q(sel, "sh"), mean_q(sel, "ua"))

    ok, conf = 0, {}
    for r in rows:
        gravity_ema.clear()
        elev = elevation_deg(r["ua"])
        az, _ = azimuth_deg(r["sh"], r["ua"])
        pred = classify(elev, az)
        t = truth(r["pose"])
        conf.setdefault(t, {}).setdefault(pred, 0)
        conf[t][pred] += 1
        ok += (pred == t)

    print(f"\n=== {os.path.basename(path)} 재분류 ===")
    print(f"정확도 {ok}/{len(rows)} = {100.0*ok/len(rows):.1f}%")
    for t in sorted(conf):
        print(f"  {t:12s} -> " + ", ".join(f"{k} {v}" for k, v in sorted(conf[t].items())))
    print()
    calib.update(saved); anchor.update(saved_anchor)


# ── 통신 ───────────────────────────────────────────────────────

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
                note_sample(data["id"], t)
                if rec["on"]:
                    q = (data["qw"], data["qx"], data["qy"], data["qz"])
                    for key, sid in (("sh", SENSORS["shoulder"]),
                                     ("ua", SENSORS["upperarm"]),
                                     ("fa", SENSORS["forearm"])):
                        if data["id"] == sid:
                            rec[key].append((t, q))
                continue
            cmd = data.get("cmd")
            try:
                pass
            except Exception:
                pass
            if cmd == "zero":
                start_rec("zero")
            elif cmd == "cal":
                capture_dir(data.get("pose", "forward"))
            elif cmd == "twist":
                start_rec("twist")
            elif cmd == "elbow":
                start_rec("elbow")
            elif cmd == "twist":
                start_rec("twist")
            elif cmd == "elbow":
                start_rec("elbow")
            elif cmd == "anchor":
                reset_anchor()
            elif cmd == "auto":
                anchor["enabled"] = not anchor["enabled"]
                print(f"\n[ok] 자동 재앵커 {'켬' if anchor['enabled'] else '끔'}")
            elif cmd == "reset":
                reset_calib()
    finally:
        connected.discard(ws)


async def broadcast_loop():
    """
    한 프레임 계산이 실패해도 서버 전체가 멈추면 안 된다.
    (예외가 여기서 새어나가면 asyncio.gather 가 무너져 모든 연결이 끊긴다)
    """
    last_err = None
    while True:
        try:
            if rec["on"] and time.time() >= rec["until"]:
                print()
                finish_rec()
            msg = json.dumps(analyze())
        except Exception as e:                      # noqa: BLE001
            key = f"{type(e).__name__}:{e}"
            if key != last_err:                     # 같은 오류를 도배하지 않는다
                last_err = key
                print(f"\n[!] 상태 계산 오류: {key}")
                traceback.print_exc()
            msg = json.dumps({"status": f"서버 계산 오류: {key}",
                              "pose": "UNKNOWN", "elev": 0.0, "az": 0.0,
                              "calib": calib_state(), "sensors": sensor_report()})
        else:
            last_err = None
        if connected:
            await asyncio.gather(*(w.send(msg) for w in connected),
                                 return_exceptions=True)
        await asyncio.sleep(0.1)


async def sensor_status_loop():
    """콘솔 한 줄을 계속 덮어써서 접속 현황을 실시간으로 보여준다."""
    while True:
        sys.stdout.write("\r" + sensor_line().ljust(78))
        sys.stdout.flush()
        await asyncio.sleep(0.5)


async def console_loop():
    loop = asyncio.get_event_loop()
    table = {"zero": lambda: capture_rest(),
             "fwd": lambda: capture_dir("forward"),
             "side": lambda: capture_dir("side"),
             "diag": lambda: capture_dir("diag"),
             "twist": lambda: start_rec("twist"),
             "elbow": lambda: start_rec("elbow"),
             "anchor": lambda: reset_anchor(),
             "reset": reset_calib,
             "save": save_calib,
             "load": load_calib}
    while True:
        line = (await loop.run_in_executor(None, sys.stdin.readline)).strip()
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        cmd = cmd.lower()
        print()
        if cmd in table:
            table[cmd]()
        elif cmd == "auto":
            anchor["enabled"] = not anchor["enabled"]
            print(f"[ok] 자동 재앵커 {'켬' if anchor['enabled'] else '끔'}")
        elif cmd == "sensors":
            for s in sensor_report():
                tag = "" if s["known"] else "  (등록되지 않은 ID)"
                if s["age"] is None:
                    print(f"  [ ] {s['id']:12s} 수신 없음{tag}")
                else:
                    mark = "O" if s["ok"] else "!"
                    print(f"  [{mark}] {s['id']:12s} {s['hz']:5.1f}Hz  "
                          f"마지막 {s['age']:.1f}초 전  누적 {s['n']}{tag}")
        elif cmd == "debug":
            r = analyze()
            print(f"pose={r['pose']}  elev={r['elev']}  az={r['az']}  roll={r['roll']}")
            print(f"       flex(중력클램프)={r['flex']}  flex(경첩축)={r['flex_hinge']}  "
                  f"전완 elev={r['fa_elev']} az={r['fa_az']}")
            if ready():
                b = bounds()
                print(f"       폭 {calib['az_side']:.1f}도, 경계 {b[0]:.1f} / {b[1]:.1f}, "
                      f"자동재앵커 {'켬' if anchor['enabled'] else '끔'}")
        elif cmd == "replay":
            replay(arg.strip() or "debug_data_2026-08-12.csv")
        else:
            print("명령: zero / twist / elbow / fwd / side / diag / anchor / auto / "
                  "sensors / debug / reset / save / load / replay <csv>")


async def main():
    print("웨어러블 서버 v4.1 (오른팔)")
    print(f"포트 {PORT}\n")
    print("교정:  차렷 zero(3초, 손바닥 비틀기) -> elbow(4초, 굽혔다 펴기)")
    print("       -> fwd -> side -> diag")
    print("       그 뒤 팔을 내리고 1초 정지하면 드리프트 앵커가 자동으로 잡힙니다.\n")
    load_calib()
    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.gather(broadcast_loop(), sensor_status_loop(), console_loop())


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "replay":
        replay(sys.argv[2])
    elif websockets is None:
        print("pip install websockets 가 필요합니다.")
    else:
        asyncio.run(main())
