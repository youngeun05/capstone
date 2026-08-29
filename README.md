# Capstone

ESP32-S3 기반 상체(어깨·양팔) 자세 추적 및 3D 시각화 프로젝트입니다.

## 프로젝트 구성

### 펌웨어 (ESP32-S3)
보드별로 센서 값을 읽어 서버로 전송하는 아두이노 코드입니다.

| 파일 | 설명 |
|---|---|
| `arm_left.ino` | 왼팔 ESP32-S3 보드 펌웨어 |
| `arm_right.ino` | 오른팔 ESP32-S3 보드 펌웨어 |
| `shoulder.ino` | 어깨 ESP32-S3 보드 펌웨어 |

### 서버
| 파일 | 설명 |
|---|---|
| `server_upperbody.py` | 각 보드에서 전송된 데이터를 취합·처리하는 상체 서버 코드 |

### 뷰어
| 파일 | 설명 |
|---|---|
| `pose_full.html` | 수집된 상체 자세 데이터를 3D로 시각화하는 웹 뷰어 |

## 시스템 구조

```
[왼팔 보드]   [오른팔 보드]   [어깨 보드]
 arm_left      arm_right      shoulder
    │              │              │
    └──────────────┴──────────────┘
                   │
           server_upperbody.py
                   │
             pose_full.html
             (3D 뷰어)
```

각 ESP32-S3 보드가 관절/자세 센서 값을 측정하여 서버로 전송하면, `server_upperbody.py`가 데이터를 수신·가공하고, `pose_full.html`이 이를 실시간 3D 모델로 렌더링합니다.

## 사용 방법

1. `arm_left.ino`, `arm_right.ino`, `shoulder.ino`를 각 ESP32-S3 보드에 업로드합니다.
2. `server_upperbody.py`를 실행하여 서버를 구동합니다.
3. `pose_full.html`을 브라우저에서 열어 실시간 상체 자세를 확인합니다.
