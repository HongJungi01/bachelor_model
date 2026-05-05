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
6. [Semantic SLAM (1-stage LLM + SAM 사이드카)](#6-semantic-slam-1-stage-llm--sam-사이드카)
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
| NVIDIA GPU    | Semantic SLAM 사용 시                 | 2GB+ (SAM만 로컬, LLM은 클라우드)              |
| LLM API 키    | Semantic 사용 시 필수                 | Gemini(기본) 또는 Anthropic, `.env.bat`에 저장 |

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

## 6. Semantic SLAM (1-stage LLM + SAM 사이드카)

주차장 내 의미 있는 구조물(기둥, 주차선, 출구, 화살표 등)을 감지해 2D occupancy grid에 레이어로 태깅한다. LLM 한 번 호출에 박스+라벨+방향을 모두 받는 단일-호출 파이프라인이며, 사이드카 프로세스(`run_semantic.bat`)를 켜야 동작한다.

### 6-1. 파이프라인 구조

```
[RTAB-Map GUI]
   │
   │  POST /detect  (keyframe JPEG, request_id = nodeId)
   ▼
[semantic_service (FastAPI, port 7788)]
   │
   ├─ LLM (Gemini 3 Flash 기본 / Claude Sonnet 4.6 옵션)
   │     입력: 1장의 keyframe JPEG
   │     출력: detections[{box, label, confidence, visual_angle?}]
   │
   └─ SAM 후처리: 각 box → 픽셀 마스크, union을 PNG로 인코딩
        ↳ 응답: detections + mask_png_b64

[SemanticWorker (C++)]
   ├─ submit()는 latest-only — 처리 중에는 들어오는 keyframe이 자연스레 throttle됨
   └─ /detect 응답 → ResultCallback → SemanticMaskStore.setLabeledBoxes()

[SemanticMaskStore]
   ├─ resolveGridCode(label, visual_angle, robotYaw)
   │     pillar / parking_line / traffic_cone / no_entry_sign / construction_sign → 1
   │     exit_area                                                                 → 10
   │     exit_sign / floor_arrow (+ 방향 bin 1..8)                                 → 11..18
   │     one_way_marker (+ 방향 bin 1..8)                                          → 21..28
   │     lane_divider / intersection / 그 외                                       → 0 (skip)
   └─ applyTo(grid): pose × camPoints → 2D 셀에 코드 기록
```

**핵심 설계**: LLM 한 번이 박스 검출 + 라벨링 + 방향 추정을 모두 처리한다. 별도 클래시파이어 단계가 없어 단순하고, 모호한 사례(주차선 vs 중앙 분리선) 판단을 LLM 추론력에 직접 맡길 수 있다. 트레이드오프는 매 호출이 1~3초가 걸린다는 점 — `submit()` latest-only 시멘틱이 자연스레 keyframe rate를 제한한다.

**방향 코드**: `visual_angle`은 image-plane 각도(0=위, 시계방향)다. C++ 쪽 `imageAngleToWorldDirBin()`이 robot pose의 yaw로 회전시켜 world-frame 8-bin(prompt.md 컨벤션: bin 1=북, 시계방향)으로 양자화한다.

### 6-2. Grid 셀 값

| 값       | 의미                                 | 색상 (grid_client.py)       |
|----------|--------------------------------------|-----------------------------|
| -1       | 미탐색 (unknown)                     | 회색 (128,128,128)          |
|  0       | 빈 공간 (free)                       | 흰색 (255,255,255)          |
|  1       | 벽면형 구조물                        | 빨강 BGR (40,40,220)        |
| 10       | 목적지 (exit_area)                   | 초록 BGR (60,200,60)        |
| 11..18   | 목적지 방향성 표지 (10 + dirBin)     | 시안 BGR (220,200,60)       |
| 21..28   | 일방통행 영역 (20 + dirBin)          | 마젠타 BGR (200,60,200)     |
| 100      | 동적 장애물 (RTAB-Map)               | 검정 (0,0,0)                |

dirBin 컨벤션: 1=(0,+Y), 2=(+X,+Y), 3=(+X,0), 4=(+X,-Y), 5=(0,-Y), 6=(-X,-Y), 7=(-X,0), 8=(-X,+Y) — 시계방향, world frame.

### 6-3. 사전 준비 — LLM API 키 설정

LLM 호출이 파이프라인의 전부이므로 API 키 없이는 `/detect`가 503을 돌려준다. Gemini(기본) 또는 Anthropic 중 하나만 있으면 된다.

1. **Gemini (기본)** — [Google AI Studio](https://aistudio.google.com/apikey)에서 키 발급
   **Anthropic (옵션)** — [Anthropic Console](https://console.anthropic.com/)에서 키 발급
2. `semantic_service/.env.bat.example`을 복사해서 `.env.bat` 생성:

```powershell
Copy-Item C:\dev\semantic_service\.env.bat.example C:\dev\semantic_service\.env.bat
```

3. `.env.bat`을 열어서 사용할 provider의 키만 채운다:

```bat
@echo off
rem -- Gemini (기본) --
set GEMINI_API_KEY=YOUR_KEY_HERE

rem -- Claude로 바꾸려면 위는 비우고 아래 두 줄을 활성화 --
rem set LLM_PROVIDER=anthropic
rem set ANTHROPIC_API_KEY=YOUR_KEY_HERE
```

`.env.bat`은 `.gitignore`에 등록되어 있어 커밋되지 않는다.

### 6-4. 사이드카 실행

```powershell
C:\dev\semantic_service\run_semantic.bat
```

첫 실행 시 `semantic_service\.venv`를 만들고 `requirements.txt`를 설치한다. SAM 가중치는 첫 추론 시 자동 다운로드:
- `facebook/sam-vit-base` (~360MB)

LLM 자체는 클라우드에서 도는 외부 호출이라 로컬 다운로드가 없다.

기동 완료 로그:

```
[semantic_service] SAM loaded on device=cuda (model=facebook/sam-vit-base)
[semantic_service] LLM ready: gemini (gemini-3-flash)
INFO:     Uvicorn running on http://127.0.0.1:7788
```

키가 없거나 잘못된 provider면:

```
[semantic_service] LLM disabled — set GEMINI_API_KEY (LLM_PROVIDER=gemini) to enable /detect
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

### 6-7. 카탈로그 / 프롬프트 변경

검출 카테고리 정의는 `semantic_service/llm_runner.py`의 `_SYSTEM_PROMPT`와 `LABELS` 튜플에 모여 있다. 라벨을 추가하려면:

1. `LABELS`에 새 라벨 문자열 추가
2. `_SYSTEM_PROMPT`의 "Category catalog" 섹션에 정의/디스앰비귀에이션 규칙 작성
3. (방향성 라벨이라면) `DIRECTIONAL_LABELS`에도 추가
4. C++ `SemanticMaskStore::resolveGridCode()`에 새 라벨 → 그리드 코드 매핑 추가
5. (필요시) `priorityOf()`에 우선순위 추가

라벨 카탈로그와 그리드 코드 매핑은 항상 짝으로 변경해야 한다 (Python은 무시되는 라벨을 보내고, C++은 받지 못한 라벨을 그릴 수 없다).

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

### Semantic: `/detect` 502/503 에러

→ LLM API 문제. 원인:
- `.env.bat`가 없거나 키 빈 값 → [6-3](#6-3-사전-준비--llm-api-키-설정) 참고. `/healthz`가 `model_loaded=false`면 키가 안 잡힌 것.
- `LLM_PROVIDER`가 `anthropic`인데 `ANTHROPIC_API_KEY`가 없거나 그 반대 — `.env.bat` 다시 확인
- 키가 만료/유출됨 → 콘솔에서 새 키 발급 후 `.env.bat` 수정
- 키 발급 직후 활성화까지 수십 초 걸릴 수 있음

`semantic_service` 콘솔에 `403 PERMISSION_DENIED`(Gemini) 또는 `authentication_error`(Anthropic)가 보이면 키를 새로 발급한다.

### Semantic: GPU 안 잡힘 (`device=cpu` 로그)

→ venv의 torch가 CPU 휠. [6-4](#6-4-사이드카-실행) 의 cu121 휠 수동 교체 절차 참고.

### Semantic: SAM 다운로드가 너무 느림

→ HuggingFace 네트워크 속도 문제. `HUGGINGFACE_HUB_CACHE` 환경변수로 캐시 경로를 빠른 드라이브로 지정하거나, 다운로드 완료 후 재시작하면 캐시에서 로드된다. SAM-base는 ~360MB.

### Semantic: `/detect`가 너무 느려서 차량을 따라가지 못함

→ LLM 호출 1회당 1~3초 — 정상이다. SemanticWorker는 `submit()` latest-only이므로 처리 중 들어오는 keyframe은 자연스레 폐기된다. 효과적인 semantic 업데이트 주기는 약 0.3~0.5 Hz. 더 빠른 응답이 필요하면 더 가벼운 모델(예: `GEMINI_MODEL=gemini-3-flash-lite-preview`)로 교체.

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
│   │   │       ├── SemanticLabeledBox.h     (POD: label, confidence, visualAngle)
│   │   │       ├── SemanticWorker.h         (HTTP 워커: /detect 단일 호출)
│   │   │       ├── SemanticMaskStore.h      (camPoints 캐시 + applyTo, 코드 1/10/11..18/21..28)
│   │   │       └── SemanticBackproject.h    (depth/ray-plane + imageAngleToWorldDirBin)
│   │   └── src/
│   │       ├── MainWindow.cpp           (semantic 통합 + grid raster)
│   │       ├── PreferencesDialog.cpp    (Unity TCP UI)
│   │       └── CMakeLists.txt           (nlohmann_json 링크)
│   └── build/                   ← (gitignore) cmake build
│
├── semantic_service/            ★ 1-stage LLM + SAM 사이드카 (FastAPI)
│   ├── run_semantic.bat         (venv + uvicorn 자동 부팅, .env.bat 로드)
│   ├── .env.bat                 (gitignore) LLM API 키 설정
│   ├── .env.bat.example         키 설정 템플릿 (커밋됨)
│   ├── app.py                   (POST /detect, GET /debug_image, /debug_json, /healthz)
│   ├── llm_runner.py            (Gemini 3 Flash / Claude Sonnet 4.6 단일 호출, 카탈로그 정의)
│   ├── sam_runner.py            (SAM 박스→픽셀 마스크)
│   └── requirements.txt
│
├── unityCar/                    ← Unity 자산 (Assets/에 복사)
│   ├── Car 1.prefab
│   ├── CarController.cs
│   └── DepthCamera/             (RGBD/IMU/Streamer 스크립트 + prefab + shader)
│
└── vcpkg_export/                ← (gitignore) RTAB-Map 릴리스 첨부 추출
```
