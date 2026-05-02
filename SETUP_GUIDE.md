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
6. [Semantic SLAM (Grounded-SAM 사이드카)](#6-semantic-slam-grounded-sam-사이드카)
7. [Unity 측 설정 (`unityCar`)](#7-unity-측-설정-unitycar)
8. [TCP 패킷 검증 도구](#8-tcp-패킷-검증-도구)
9. [트러블슈팅](#9-트러블슈팅)
10. [프로젝트 구조](#10-프로젝트-구조)

---

## 1. 사전 조건

| 항목          | 버전/사양                            | 비고                                             |
| ------------- | ------------------------------------ | ------------------------------------------------ |
| Visual Studio | 2022 Community (MSVC 19.44+)         | "C++를 사용한 데스크톱 개발" 워크로드 포함       |
| Windows SDK   | 10.0.22621.0 이상                    | VS Installer에 포함                              |
| CMake         | 3.20+                                | VS2022 번들 사용 가능 (별도 설치 가능)           |
| Unity         | 2022.3 LTS (DX11)                    | Unity TCP 모드만 사용                            |
| Python 3.10+  | Semantic 사이드카 + 검증 도구용      | venv 자동 생성, GPU torch는 cu121 휠 권장        |
| NVIDIA GPU    | Semantic SLAM 사용 시                 | 4GB+ (GroundingDINO-tiny + SAM-base 동시 로드)   |

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

### 3-2. 빌드

```powershell
cmd /c "`"$vs`" && cmake --build $bld --config Release --parallel"
```

소요 시간: 첫 빌드 5~15분, 증분 빌드는 수정 파일에 따라 10초~3분.

빌드 산출물:

```
C:\dev\rtabmap\build\bin\RTABMap.exe
C:\dev\rtabmap\build\bin\rtabmap_core.dll
C:\dev\rtabmap\build\bin\rtabmap_gui.dll
```

### 3-3. ⚠️ 빌드 실패 시 확인 — stub 헤더 정리

이전 세션에서 누군가 임시로 만든 stub `*_export.h` / `Version.h`가 **source tree**에 남아 있으면 CMake가 자동 생성한 정확한 헤더를 가려서 다음 에러가 난다:

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

### 5-2. Unity 씬 준비 ([6절](#6-unity-측-설정-unitycar) 참고)

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

## 6. Semantic SLAM (Grounded-SAM 사이드카)

주차선·차로 표시 같은 텍스트 라벨로 지정 가능한 객체를 픽셀 단위로 찾아내 2D occupancy grid에 obstacle(셀 값 **80**)로 태깅한다. 옵션 기능이며 사이드카 프로세스를 켜야 동작한다.

### 6-1. 구성

```
[RTAB-Map GUI]  ─── HTTP POST ──▶  [semantic_service (FastAPI)]
                                       ├─ GroundingDINO  (text-prompted detection)
                                       └─ SAM            (pixel mask from boxes)
       ◀──── JSON {boxes, mask_png_b64} ────
```

- GUI (`SemanticWorker`)가 keyframe RGB를 latest-only 큐로 사이드카에 보내고, 응답 마스크를 `SemanticMaskStore`에 캐시한다.
- 픽셀 → 카메라 좌표계 3D 포인트 변환은 마스크 도착 시 **딱 한 번** 수행. 매 publish 호출은 `pose × camPoints` 행렬 곱만 — keyframe이 쌓여도 비용이 일정하다.
- Loop closure로 pose가 정정되면 다음 publish에서 자동 반영.

### 6-2. 사이드카 실행

```powershell
C:\dev\semantic_service\run_semantic.bat
```

첫 실행 시 `semantic_service\.venv`를 만들고 `requirements.txt`(fastapi · uvicorn · torch · transformers · pillow · numpy)를 설치한다. 이후 실행은 venv를 활성화만 하고 uvicorn을 띄운다. HuggingFace 가중치는 첫 추론 시 자동 다운로드 — `IDEA-Research/grounding-dino-tiny` (~700MB), `facebook/sam-vit-base` (~360MB).

기본 venv는 **CPU torch**가 깔린다. GPU(권장)를 쓰려면 한 번만 수동 교체:

```powershell
cd C:\dev\semantic_service
.venv\Scripts\activate
pip uninstall -y torch
pip install torch --index-url https://download.pytorch.org/whl/cu121
```

기동 로그에 `device=cuda`가 보이면 OK.

```
[semantic_service] models loaded on device=cuda (gdino=..., sam=facebook/sam-vit-base)
INFO:     Uvicorn running on http://127.0.0.1:7788
```

### 6-3. GUI 연결

`run_rtabmap.bat`이 `RTABMAP_SEMANTIC_URL=http://127.0.0.1:7788/detect`를 환경변수로 세팅하므로 GUI는 사이드카가 켜져 있으면 자동 연결한다. 끄려면 `run_rtabmap.bat`의 해당 줄을 주석 처리하거나 빈 값으로 두면 됨.

### 6-4. 디버그 도구

- `python C:\dev\semantic_viewer.py` — 사이드카의 `/debug_image`를 0.5초마다 폴링해서 박스+마스크 오버레이를 띄운다. SAM이 박스에 비해 얼마나 정확하게 라인을 잡아냈는지 시각적으로 확인 가능.
- `python C:\dev\grid_client.py` — RTAB-Map GUI의 TCP 그리드 스트림을 실시간 렌더. 셀 값 **80(semantic)** 은 빨간색, **100(structural wall)** 은 검은색으로 구분돼 보임.

### 6-5. 프롬프트 / 임계치 변경

`semantic_service/app.py`의 `DetectRequest` 기본값을 수정하거나, GUI 측 `SemanticWorker.h::httpDetect()`의 JSON body에서 `prompt`/`box_threshold`를 바꾸면 된다.

```python
prompt = "parking space line . lane marking"
box_threshold  = 0.3
text_threshold = 0.25
```

GroundingDINO 프롬프트는 마침표(`. `)로 클래스를 구분한다.

---

## 7. Unity 측 설정 (`unityCar`)

`c:\dev\unityCar` 디렉토리는 그대로 Unity 자산 패키지로 사용 가능 (`.meta` 포함).

### 6-1. 폴더 통째로 import

1. 기존 Unity 프로젝트의 `Assets/Car/`가 있으면 **삭제** (구버전 GUID 충돌 방지)
2. `c:\dev\unityCar` 폴더를 `Assets/`로 통째로 복사 → `Assets/unityCar/`
3. Unity 자동 import 대기

### 6-2. 씬에 배치

`Assets/unityCar/Car 1.prefab`을 Hierarchy로 드래그.

자식 `D455_Virtual_Camera` GameObject에 다음 3개 스크립트가 자동 attach 되어 있어야 함:

- **RGBD_Data Collector** — RGB+Depth 캡처 (1280×720 @ 10fps)
- **IMU Sensor** — gyro/accel (rad/s, m/s², body frame)
- **RTAB Map Streamer** — TCP 클라이언트 (host=127.0.0.1, port=7778)

### 6-3. 패킷 프로토콜 (참고)

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

## 8. TCP 패킷 검증 도구

Unity 측만 단독으로 시험하려면 (rtabmap 끄고) Python 서버를 사용한다.

```powershell
# 콘솔만
python C:\dev\test_unity_tcp.py

# RGB / Depth 시각화
python C:\dev\test_unity_tcp.py --show
```

정상 출력 예시:

```
[server] listening on 0.0.0.0:7778
[server] client connected from ('127.0.0.1', 5xxxx)
[Calib #1]
  size = 1280 x 720
  fx=648.93 fy=648.93 cx=640.00 cy=360.00
  localTransform: [[ 0  0  1  0][−1  0  0  0][ 0 −1  0  0]]
[IMU #1] t=0.012s gyro=(...) accel=(0, 9.81, 0) |a|=9.81 m/s²
[RGBD #1] t=0.123s 1280x720 payload=4608016B [OK]
--- stats: 5s Calib=1 IMU=234 (46 Hz) RGBD=68 (13.6 fps) ---
```

`Calib`이 한 번 오고 그 후 IMU/RGBD가 계속 흐르면 Unity 측은 정상.

---

## 9. 트러블슈팅

### 빌드: `psapi.lib`를 못 찾음

→ vcvars64.bat 환경에서 cmake를 실행 안 한 경우. [3-1](#3-1-visual-studio-개발자-환경에서-cmake-configure) 참고.

### 빌드: dllimport C2491 에러 / `RTABMAP_PCL_INDEX` 미정의

→ source tree에 stub `*_export.h` / `Version.h`가 남아 generated 헤더를 가리는 경우. [3-3](#3-3-️-빌드-실패-시-확인--stub-헤더-정리) 참고.

### 빌드: `LNK1104: rtabmap_gui.dll 파일을 열 수 없습니다`

→ RTABMap.exe가 실행 중이라 dll을 못 덮어씀. RTABMap을 닫고 재빌드.

### 실행: Qt platform plugin 에러

→ `QT_PLUGIN_PATH`가 비어 있음. `run_rtabmap.bat`을 사용하거나 직접 환경변수 설정.

### Unity TCP: `[Streamer] connected`만 반복, `calibration sent`가 안 뜸

→ 이전 버그. RTABMapStreamer가 worker thread에서 Unity API를 호출했음. 최신 버전은 메인 스레드에서 calibration을 캐싱하므로 unityCar를 최신본으로 갱신.

### Unity TCP: rtabmap이 응답 없음 (Start 직후)

→ 이전 버그. `init()`이 메인 스레드에서 calibration을 무한 대기했음. 최신 버전은 non-blocking — rtabmap_core.dll을 다시 빌드해서 갱신.

### Unity TCP: `RTAB-Map: Camera initialization failed`

→ 30초 timeout 버전을 쓰고 있음. 위와 같은 fix 적용된 build로 교체.

### SLAM: 한 바퀴 돌면 맵이 휨

→ Loop closure가 거부되고 있음. [5-4](#5-4-휘는-맵-drift-보정--loop-closure-임계값-조정) 참고.

### Semantic: GUI에 빨간 셀이 안 그려짐

→ ① `semantic_service`가 안 켜져 있음 — `run_semantic.bat` 콘솔에 `Uvicorn running on ...:7788`이 떠야 한다. ② `RTABMAP_SEMANTIC_URL`이 `run_rtabmap.bat`에서 비어 있음. ③ keyframe이 아직 없음 — Start 직후엔 정상.

### Semantic: 사이드카 첫 요청에서 500 에러 / `cv2 not found`

→ 옛 버전의 `app.py`에 남아 있던 cv2 의존성. 최신본은 PIL 만으로 처리한다. `git pull` 후 재시작.

### Semantic: GPU 안 잡힘 (`device=cpu` 로그)

→ venv의 torch가 CPU 휠. [6-2](#6-2-사이드카-실행) 의 cu121 휠 수동 교체 절차 참고.

---

## 10. 프로젝트 구조

```
C:\dev\
├── SETUP_GUIDE.md               ← 이 문서
├── run_rtabmap.bat              ← RTABMap GUI 실행 (semantic URL 환경변수 포함)
├── run_pipeline.bat             ← rtabmap_pipeline 실행 (Unity / RealSense × Mock / HTTP)
├── test_unity_tcp.py            ← Unity TCP 검증용 Python 서버
├── grid_client.py               ← GUI TCP 그리드 스트림 라이브 뷰어
├── semantic_viewer.py           ← semantic_service /debug_image 라이브 뷰어
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
│   │   │       ├── SemanticWorker.h         (HTTP 비동기 워커)
│   │   │       ├── SemanticMaskStore.h      (camPoints 캐시 + applyTo)
│   │   │       └── SemanticBackproject.h    (depth/ray-plane 헬퍼)
│   │   └── src/
│   │       ├── MainWindow.cpp           (semantic 통합 + grid raster)
│   │       ├── PreferencesDialog.cpp    (Unity TCP UI)
│   │       └── CMakeLists.txt           (nlohmann_json 링크)
│   └── build/                   ← (gitignore) cmake build
│
├── rtabmap_pipeline/            ← Standalone CLI (RealSense / Unity TCP × Mock / HTTP)
│
├── semantic_service/            ★ Grounded-SAM 사이드카 (FastAPI)
│   ├── run_semantic.bat         (venv + uvicorn 자동 부팅)
│   ├── app.py                   (POST /detect, GET /debug_image)
│   ├── gdino_runner.py          (GroundingDINO 텍스트→박스)
│   ├── sam_runner.py            (SAM 박스→픽셀 마스크)
│   ├── test_detect.py           (스모크 테스트)
│   └── requirements.txt
│
├── unityCar/                    ← Unity 자산 (Assets/에 복사)
│   ├── Car 1.prefab
│   ├── CarController.cs
│   └── DepthCamera/             (RGBD/IMU/Streamer 스크립트 + prefab + shader)
│
└── vcpkg_export/                ← (gitignore) RTAB-Map 릴리스 첨부 추출
```
