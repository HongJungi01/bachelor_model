# 빌드 & 실행 가이드

> Unity 가상 D455 → TCP → RTAB-Map (RGB+Depth+IMU) → 3D Map / 2D Occupancy Grid
> 실 카메라(RealSense2) 모드도 그대로 사용 가능
>
> OS: Windows 10/11 | 빌드: Visual Studio 2022 + CMake

---

## 목차

1. [사전 조건](#1-사전-조건)
2. [vcpkg 의존성 설치](#2-vcpkg-의존성-설치)
3. [RTAB-Map 빌드](#3-rtab-map-빌드)
4. [실행](#4-실행)
5. [Unity TCP 모드로 SLAM 돌리기](#5-unity-tcp-모드로-slam-돌리기)
6. [Semantic SLAM (Florence-2 + SAM + Gemini 사이드카)](#6-semantic-slam-florence-2--sam--gemini-사이드카)
7. [Unity 측 설정 (`unityCar`)](#7-unity-측-설정-unitycar)
8. [트러블슈팅](#8-트러블슈팅)
9. [프로젝트 구조](#9-프로젝트-구조)

---

## 1. 사전 조건

| 항목          | 버전/사양                            | 비고                                             |
| ------------- | ------------------------------------ | ------------------------------------------------ |
| Visual Studio | 2022 Community (MSVC 19.44+)         | "C++를 사용한 데스크톱 개발" 워크로드 포함       |
| Windows SDK   | 10.0.22621.0 이상                    | VS Installer에 포함                              |
| CMake         | 3.20+                                | VS2022 번들 사용 가능 (별도 설치 가능)           |
| Unity         | 2022.3 LTS (DX11)                    | Unity TCP 모드만 사용                            |
| Python 3.11+  | Semantic 사이드카용                  | venv 자동 생성, GPU torch는 cu121 휠 권장        |
| NVIDIA GPU    | Semantic SLAM 사용 시                 | 4GB+ (Florence-2 + SAM 동시 로드)               |
| Gemini API 키 | L3 분류 사용 시                       | Google AI Studio에서 발급, `.env.bat`에 저장    |

---

## 2. vcpkg 의존성 설치

RTAB-Map 공식 릴리스에서 사전빌드된 vcpkg export를 사용한다 (`C:\dev\vcpkg_export`).

```powershell
cd C:\dev
$url = "https://github.com/introlab/rtabmap/releases/download/0.23.1/vcpkg-export-66c0373d-x64-vs2022.7z"
Invoke-WebRequest -Uri $url -OutFile "vcpkg_export.7z"
& "C:\Program Files\7-Zip\7z.exe" x "vcpkg_export.7z" -o"C:\dev\vcpkg_export" -y
```

설치 결과 — `C:\dev\vcpkg_export\installed\x64-windows-release\` 에 Qt6, VTK, PCL, OpenCV, librealsense, g2o, Eigen, Boost 등이 들어 있음.

> **주의**: `CMAKE_TOOLCHAIN_FILE`로 vcpkg toolchain을 쓰면 manifest mode가 활성화되어 export 패키지의 triplet을 인식 못한다. **반드시 `CMAKE_PREFIX_PATH`만 사용**한다 (아래 빌드 절차).

---

## 3. RTAB-Map 빌드

### 3-1. Visual Studio 개발자 환경에서 CMake configure

`psapi.lib` 같은 시스템 라이브러리는 VS의 `LIB` 환경변수가 있어야 찾는다. **반드시 `vcvars64.bat` 환경에서** 실행한다.

```powershell
# PowerShell에서 한 줄로 실행 (vcvars + cmake)
$vs   = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
$repo = "C:\dev\rtabmap"
$bld  = "C:\dev\rtabmap\build"
$pre  = "C:/dev/vcpkg_export/installed/x64-windows-release"

cmd /c "`"$vs`" && cmake -S $repo -B $bld -G `"Visual Studio 17 2022`" -A x64 ^
  -DCMAKE_PREFIX_PATH=`"$pre`" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_APP=ON -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF ^
  -DWITH_REALSENSE2=ON"
```

성공 시 출력:

```
-- With Qt (Qt6) = YES
-- With RealSense2 = YES
-- Configuring done
-- Build files have been written to: C:/dev/rtabmap/build
```

### 3-2. 전체 빌드

```powershell
$vs  = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
$bld = "C:\dev\rtabmap\build"
cmd /c "`"$vs`" && cmake --build $bld --config Release --parallel"
```

소요 시간: 첫 빌드 5~15분, 증분 빌드는 수정 파일에 따라 10초~3분.

빌드 산출물:

```
C:\dev\rtabmap\build\bin\RTABMap.exe
C:\dev\rtabmap\build\bin\rtabmap_core.dll
C:\dev\rtabmap\build\bin\rtabmap_gui.dll
```

### 3-3. GUI만 재빌드 (semantic 헤더 수정 후)

`guilib/` 산하 파일만 수정했을 때 — `rtabmap_gui` 타깃만 빌드하면 `rtabmap_core`를 건드리지 않아 빠르다.

```powershell
$vs  = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
$bld = "C:\dev\rtabmap\build"
cmd /c "`"$vs`" && cmake --build $bld --config Release --target rtabmap_gui --parallel"
```

주로 수정하는 파일:

| 파일 | 재빌드 타깃 |
|------|------------|
| `guilib/include/rtabmap/gui/semantic/*.h` | `rtabmap_gui` |
| `guilib/src/MainWindow.cpp` | `rtabmap_gui` |
| `corelib/include/` 또는 `corelib/src/` | 전체 (`--parallel`) |

> **팁**: `RTABMap.exe`가 실행 중이면 `rtabmap_gui.dll`을 덮어쓸 수 없어 `LNK1104` 에러가 난다. 빌드 전에 GUI를 닫는다.

### 3-4. ⚠️ 빌드 실패 시 확인 — stub 헤더 정리

이전 세션에서 임시로 만든 stub `*_export.h` / `Version.h`가 **source tree**에 남아 있으면 CMake가 자동 생성한 정확한 헤더를 가려서 다음 에러가 난다:

```
error C2491: 'ULogger::instance_': dllimport 정적 데이터 멤버를 정의할 수 없습니다.
error C2065: 'RTABMAP_PCL_INDEX': 선언되지 않은 식별자입니다.
```

해결 — 다음 stub들이 **untracked 상태로** 있으면 삭제 (CMake가 build 디렉토리에 정확한 버전을 자동 생성한다):

```powershell
Remove-Item -Force `
  C:\dev\rtabmap\corelib\include\rtabmap\core\rtabmap_core_export.h, `
  C:\dev\rtabmap\corelib\include\rtabmap\core\Version.h, `
  C:\dev\rtabmap\utilite\include\rtabmap\utilite\utilite_export.h, `
  C:\dev\rtabmap\utilite\include\rtabmap\utilite\rtabmap_utilite_export.h `
  -ErrorAction SilentlyContinue
```

그리고 `rtabmap/build` 디렉토리의 `CMakeCache.txt`/`CMakeFiles`가 손상돼 보이면 (configure가 dependencies를 못 찾음) 그 둘만 삭제하고 3-1부터 재실행한다 (`build/bin`은 보존됨).

---

## 4. 실행

```powershell
C:\dev\run_rtabmap.bat
```

`run_rtabmap.bat`이 PATH (vcpkg bin + rtabmap bin), `QT_PLUGIN_PATH`를 잡아주고 GUI를 띄운다.

GUI 패널: **3D Map**, **Graph view (2D occupancy grid)**, **Loop closure detection**, **Odometry**.

---

## 5. Unity TCP 모드로 SLAM 돌리기

### 5-1. RTABMap 설정 (한 번만)

`Edit → Preferences → Source`:

| 설정 | 값 |
|------|-----|
| Source type | **RGB-D** |
| Driver | **Unity TCP** |
| Unity TCP Camera → Listen Port | **7778** |
| Input rate | 0.0 Hz (= as fast as possible) |

Apply → OK.

### 5-2. Unity 씬 준비 ([7절](#7-unity-측-설정-unitycar) 참고)

`unityCar` 패키지를 Unity 프로젝트 `Assets/`에 복사하고 `Car 1.prefab`을 씬에 배치.

### 5-3. 실행 순서 (자유)

`init()`이 non-blocking이므로 어느 쪽을 먼저 켜도 자동 연결된다.

1. RTABMap **Start (▶)** — Console에 `listening on port 7778 (Unity may connect anytime)`
2. Unity **Play** — Unity Console에 `[Streamer] connected → 127.0.0.1:7778` + `calibration sent`
3. RTABMap의 3D Map / Graph view에 SLAM 결과가 실시간으로 그려진다

### 5-4. 휘는 맵 (drift) 보정 — Loop closure 임계값 조정

지하주차장처럼 시각적으로 반복되는 환경은 loop closure가 거부되기 쉽다 (`Loop hypothesis N rejected!` 메시지). Preferences에서:

| 파라미터 | 기본값 | 추천 |
|---|---|---|
| `Rtabmap/LoopThr` | 0.11 | **0.05** |
| `RGBD/ProximityBySpace` | true | true |
| `RGBD/ProximityMaxGraphDepth` | 50 | **0** (무제한) |
| `RGBD/ProximityPathFilteringRadius` | 1.0 | **2.0** |
| `Vis/MinInliers` | 20 | **15** |
| `RGBD/OptimizeMaxError` | 3.0 | **0** (무제한) |

---

## 6. Semantic SLAM (Florence-2 + SAM + Gemini 사이드카)

주차장 내 의미 있는 구조물(기둥, 주차선, 출구 표지 등)을 감지해 2D occupancy grid에 레이어로 태깅한다. 3단계 파이프라인으로 구성되며, 사이드카 프로세스(`run_semantic.bat`)를 켜야 동작한다.

### 6-1. 파이프라인 구조

```
[RTAB-Map GUI]
   │
   │  POST /detect  (keyframe JPEG)
   ▼
[semantic_service (FastAPI, port 7788)]
   │
   ├─ L1+L2: Florence-2 (CAPTION_TO_PHRASE_GROUNDING) → 바운딩 박스
   │          SAM → 박스에서 픽셀 마스크 생성
   │          ↳ 응답: boxes (좌표) + mask_png_b64
   │
   └─ L3:  POST /classify_batch  (JPEG + 박스 ID, 최대 5 프레임 묶음)
           Gemini 3.1 Flash Lite → 박스별 label + confidence + visual_angle
           ↳ 응답: classifications [{box_id, label, confidence, visual_angle}]

[SemanticWorker (C++)]
   ├─ /detect 응답: 박스를 PendingFrame 버퍼에 적재
   └─ 5프레임 도달 or 1500ms 타임아웃 → /classify_batch 배치 전송
      응답 도착 시 ResultCallback 호출 → SemanticMaskStore에 LabeledBox 등록

[SemanticMaskStore]
   ├─ LabeledBox → 레이블별 grid code 변환
   │     pillar / parking_line / traffic_cone / no_entry_sign / construction_sign → 1
   │     exit_area → 10
   └─ applyTo(grid): pose × camPoints → 2D 셀에 코드 기록
```

**핵심 설계**: L1+L2(Florence-2+SAM)는 영역만 분리하고 레이블을 붙이지 않는다. 레이블 할당은 L3(Gemini)만 한다. L3가 비동기로 도착하기 전까지는 이전 결과가 그대로 유지된다.

### 6-2. Grid 셀 값

| 값  | 의미 | 색상 (grid_client.py) |
|-----|------|----------------------|
| -1  | 미탐색 (unknown)       | 회색 (128,128,128) |
|  0  | 빈 공간 (free)         | 흰색 (255,255,255) |
|  1  | 벽면형 구조물          | 빨강 BGR (40,40,220) |
| 10  | 목적지 (출구 구역)     | 초록 BGR (60,200,60) |
| 100 | 동적 장애물 (RTAB-Map) | 검정 (0,0,0) |

### 6-3. 사전 준비 — Gemini API 키 설정

L3(Gemini 분류)를 사용하려면 API 키가 필요하다. 없으면 L1+L2 박스 감지만 동작한다.

1. [Google AI Studio](https://aistudio.google.com/apikey)에서 키 발급
2. `semantic_service/.env.bat.example`을 복사해서 `.env.bat` 생성:

```powershell
Copy-Item C:\dev\semantic_service\.env.bat.example C:\dev\semantic_service\.env.bat
```

3. `.env.bat` 열어서 키 입력:

```bat
@echo off
set GEMINI_API_KEY=YOUR_KEY_HERE
```

`.env.bat`은 `.gitignore`에 등록되어 있어 커밋되지 않는다.

### 6-4. 사이드카 실행

```powershell
C:\dev\semantic_service\run_semantic.bat
```

첫 실행 시 `semantic_service\.venv`를 만들고 `requirements.txt`를 설치한다. HuggingFace 가중치는 첫 추론 시 자동 다운로드:
- `microsoft/Florence-2-base` (~1.5GB)
- `facebook/sam-vit-base` (~360MB)

기동 완료 로그:

```
[semantic_service] L1+L2 loaded on device=cuda (florence=microsoft/Florence-2-base, sam=facebook/sam-vit-base)
[semantic_service] L3 Gemini classifier loaded (model=gemini-3.1-flash-lite-preview)
INFO:     Uvicorn running on http://127.0.0.1:7788
```

L3가 비활성(키 없음)이면:

```
[semantic_service] L3 disabled — set GEMINI_API_KEY (or GOOGLE_API_KEY) to enable /classify_batch
```

기본 venv는 **CPU torch**가 깔린다. GPU(권장)를 쓰려면 한 번만 수동 교체:

```powershell
cd C:\dev\semantic_service
.venv\Scripts\activate
pip uninstall -y torch
pip install torch --index-url https://download.pytorch.org/whl/cu121
```

### 6-5. GUI 연결

`run_rtabmap.bat`이 `RTABMAP_SEMANTIC_URL=http://127.0.0.1:7788` 환경변수를 세팅하므로 GUI는 사이드카가 켜져 있으면 자동 연결한다. 끄려면 `run_rtabmap.bat`의 해당 줄을 주석 처리하거나 빈 값으로 두면 된다.

### 6-6. 디버그 보기

| 엔드포인트 | 설명 |
|-----------|------|
| `http://127.0.0.1:7788/debug_image` | 마지막 keyframe에 박스 + SAM 마스크 오버레이 (JPEG, 새로 고침으로 갱신) |
| `http://127.0.0.1:7788/debug_json`  | 마지막 /detect 결과 요약 (JSON) |
| `http://127.0.0.1:7788/healthz`     | 서비스 상태 + 모델 로드 여부 |

### 6-7. 프롬프트 / 임계치 변경

`app.py`의 `DetectRequest` 기본값을 수정하면 Florence-2가 찾는 객체 유형을 바꿀 수 있다:

```python
prompt = (
    "parking stall lines, exit signs, pillars, traffic cones, "
    "no entry signs, construction signs, floor direction arrows"
)
box_threshold  = 0.3
text_threshold = 0.25
```

Florence-2 CAPTION_TO_PHRASE_GROUNDING은 쉼표(`,`)로 구문을 구분한다.

### 6-8. 점유 그리드 실시간 확인 (grid_client.py)

RTAB-Map GUI가 TCP 7777 포트로 송출하는 occupancy grid를 별도 창으로 시각화한다:

```powershell
pip install opencv-python numpy
python C:\dev\grid_client.py
```

`q` 또는 `ESC`로 종료. 셀 색상은 [6-2](#6-2-grid-셀-값) 참고.

---

## 7. Unity 측 설정 (`unityCar`)

`c:\dev\unityCar` 디렉토리는 그대로 Unity 자산 패키지로 사용 가능 (`.meta` 포함).

### 7-1. 폴더 통째로 import

1. 기존 Unity 프로젝트의 `Assets/Car/`가 있으면 **삭제** (구버전 GUID 충돌 방지)
2. `c:\dev\unityCar` 폴더를 `Assets/`로 통째로 복사 → `Assets/unityCar/`
3. Unity 자동 import 대기

### 7-2. 씬에 배치

`Assets/unityCar/Car 1.prefab`을 Hierarchy로 드래그.

자식 `D455_Virtual_Camera` GameObject에 다음 3개 스크립트가 자동 attach 되어 있어야 함:

- **RGBD_Data Collector** — RGB+Depth 캡처 (1280×720 @ 10fps)
- **IMU Sensor** — gyro/accel (rad/s, m/s², body frame)
- **RTAB Map Streamer** — TCP 클라이언트 (host=127.0.0.1, port=7778)

### 7-3. 패킷 프로토콜 (참고)

```
Header (5B): [type:u8][payloadSize:u32]   ── little-endian
  Type 1 Calib (136B):   w(u32) h(u32) fx fy cx cy(f64×4) localTransform(f64×12)
  Type 2 IMU   (56B):    stamp(f64) gx,gy,gz(f64) ax,ay,az(f64)
  Type 3 RGBD:           stamp(f64) w(u32) h(u32) rgb[W*H*3] depth[W*H*2 mm uint16]
```

좌표계:
- RGB/Depth: top-left origin (OpenCV)
- Calibration `localTransform`: optical(X-right,Y-down,Z-forward) → base_link(X-forward,Y-left,Z-up)
- IMU: Unity body frame (X-right, Y-up, Z-forward) → rtabmap이 base_link로 회전

---

## 8. 트러블슈팅

### 빌드: `psapi.lib`를 못 찾음

→ vcvars64.bat 환경에서 cmake를 실행 안 한 경우. [3-1](#3-1-visual-studio-개발자-환경에서-cmake-configure) 참고.

### 빌드: dllimport C2491 에러 / `RTABMAP_PCL_INDEX` 미정의

→ source tree에 stub `*_export.h` / `Version.h`가 남아 generated 헤더를 가리는 경우. [3-4](#3-4-️-빌드-실패-시-확인--stub-헤더-정리) 참고.

### 빌드: `LNK1104: rtabmap_gui.dll 파일을 열 수 없습니다`

→ RTABMap.exe가 실행 중이라 dll을 못 덮어씀. RTABMap을 닫고 재빌드.

### 실행: Qt platform plugin 에러

→ `QT_PLUGIN_PATH`가 비어 있음. `run_rtabmap.bat`을 사용하거나 직접 환경변수 설정.

### Unity TCP: `[Streamer] connected`만 반복, `calibration sent`가 안 뜸

→ 이전 버그. RTABMapStreamer가 worker thread에서 Unity API를 호출했음. 최신 버전은 메인 스레드에서 calibration을 캐싱하므로 unityCar를 최신본으로 갱신.

### Unity TCP: rtabmap이 응답 없음 (Start 직후)

→ 이전 버그. `init()`이 메인 스레드에서 calibration을 무한 대기했음. 최신 버전은 non-blocking — rtabmap_core.dll을 다시 빌드해서 갱신.

### SLAM: 한 바퀴 돌면 맵이 휨

→ Loop closure가 거부되고 있음. [5-4](#5-4-휘는-맵-drift-보정--loop-closure-임계값-조정) 참고.

### Semantic: GUI에 빨간/초록 셀이 안 그려짐

→ ① `semantic_service`가 안 켜져 있음 — `run_semantic.bat` 콘솔에 `Uvicorn running on ...:7788`이 떠야 한다. ② `RTABMAP_SEMANTIC_URL`이 `run_rtabmap.bat`에서 비어 있음. ③ keyframe이 아직 없음 — Start 직후엔 정상.

### Semantic: `/classify_batch` 502 에러

→ Gemini API 키 문제. 원인:
- `.env.bat`가 없음 → [6-3](#6-3-사전-준비--gemini-api-키-설정) 참고
- 키가 만료/유출됨 → Google AI Studio에서 새 키 발급 후 `.env.bat` 수정
- 키 발급 직후 활성화까지 수십 초 걸릴 수 있음

`semantic_service` 콘솔에 `403 PERMISSION_DENIED` 또는 `reported as leaked`가 보이면 키를 새로 발급해야 한다.

### Semantic: GPU 안 잡힘 (`device=cpu` 로그)

→ venv의 torch가 CPU 휠. [6-4](#6-4-사이드카-실행) 의 cu121 휠 수동 교체 절차 참고.

### Semantic: 모델 다운로드가 너무 느림

→ HuggingFace 네트워크 속도 문제. `HUGGINGFACE_HUB_CACHE` 환경변수로 캐시 경로를 빠른 드라이브로 지정하거나, 다운로드 완료 후 재시작하면 캐시에서 로드된다.

---

## 9. 프로젝트 구조

```
C:\dev\
├── SETUP_GUIDE.md               ← 이 문서
├── run_rtabmap.bat              ← RTABMap GUI 실행 (semantic URL 환경변수 포함)
├── grid_client.py               ← TCP 7777 occupancy grid 실시간 뷰어
│
├── rtabmap/                     ← RTAB-Map 0.23.4 fork
│   ├── corelib/
│   │   ├── include/rtabmap/core/camera/CameraUnityTCP.h  ★
│   │   └── src/camera/CameraUnityTCP.cpp                  ★
│   ├── guilib/
│   │   ├── include/rtabmap/gui/
│   │   │   ├── PreferencesDialog.h     (kSrcUnityTCP enum)
│   │   │   ├── MainWindow.h            (semantic worker/store 멤버)
│   │   │   └── semantic/                ★ Semantic SLAM
│   │   │       ├── SemanticLabeledBox.h     (LabeledBox 구조체: label, confidence, visual_angle)
│   │   │       ├── SemanticWorker.h         (HTTP 비동기 워커: /detect + /classify_batch 배치)
│   │   │       ├── SemanticMaskStore.h      (camPoints 캐시 + applyTo, 코드 1/10)
│   │   │       └── SemanticBackproject.h    (depth/ray-plane 헬퍼)
│   │   └── src/
│   │       ├── MainWindow.cpp           (semantic 통합 + grid raster)
│   │       ├── PreferencesDialog.cpp    (Unity TCP UI)
│   │       └── CMakeLists.txt           (nlohmann_json 링크)
│   └── build/                   ← (gitignore) cmake build
│
├── semantic_service/            ★ Florence-2 + SAM + Gemini 사이드카 (FastAPI)
│   ├── run_semantic.bat         (venv + uvicorn 자동 부팅, .env.bat 로드)
│   ├── .env.bat                 (gitignore) GEMINI_API_KEY 설정
│   ├── .env.bat.example         키 설정 템플릿 (커밋됨)
│   ├── app.py                   (POST /detect, POST /classify_batch, GET /debug_image)
│   ├── florence_runner.py       (L2: Florence-2 CAPTION_TO_PHRASE_GROUNDING)
│   ├── gemini_runner.py         (L3: Gemini 분류기, label+confidence+visual_angle)
│   ├── sam_runner.py            (L2: SAM 박스→픽셀 마스크)
│   └── requirements.txt
│
├── unityCar/                    ← Unity 자산 (Assets/에 복사)
│   ├── Car 1.prefab
│   ├── CarController.cs
│   └── DepthCamera/             (RGBD/IMU/Streamer 스크립트 + prefab + shader)
│
└── vcpkg_export/                ← (gitignore) RTAB-Map 릴리스 첨부 추출
```
