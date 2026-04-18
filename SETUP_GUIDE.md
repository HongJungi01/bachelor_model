# 빌드 & 실행 가이드

> RealSense D455f → RTAB-Map SLAM → 2D Occupancy Grid → TCP 스트리밍 (Unity)
>
> OS: Windows 10/11 | 빌드: Visual Studio 2022 + CMake

---

## 목차

1. [사전 조건](#1-사전-조건)
2. [vcpkg 의존성 설치](#2-vcpkg-의존성-설치)
3. [RTAB-Map 소스 빌드](#3-rtab-map-소스-빌드)
4. [실행](#4-실행)
5. [카메라 연결 & SLAM 시작](#5-카메라-연결--slam-시작)
6. [TCP Grid Streaming (Unity 연동)](#6-tcp-grid-streaming-unity-연동)
7. [소스 수정 & 재빌드](#7-소스-수정--재빌드)
8. [트러블슈팅](#8-트러블슈팅)
9. [프로젝트 구조](#9-프로젝트-구조)

---

## 1. 사전 조건

| 항목          | 버전/사양                       | 설치 방법                                               |
| ------------- | ------------------------------- | ------------------------------------------------------- |
| Visual Studio | 2022 Community (MSVC 19.44+)    | [visualstudio.com](https://visualstudio.microsoft.com/) |
| C++ 워크로드  | "C++를 사용한 데스크톱 개발"    | VS Installer에서 체크                                   |
| Windows SDK   | 10.0.22621.0 이상               | VS Installer에 포함                                     |
| 7-Zip         | 최신                            | `choco install 7zip -y`                                 |
| Git           | 최신                            | `winget install Git.Git`                                |
| 카메라        | Intel RealSense D455f (USB 3.0) | USB 3.0 포트에 직접 연결                                |

> **CMake**는 VS2022에 번들 포함되어 있어 별도 설치 불필요.

---

## 2. vcpkg 의존성 설치

RTAB-Map 공식 릴리스에서 사전빌드된 vcpkg export를 다운로드한다.

```powershell
cd C:\dev

# 다운로드 (~2.5GB)
$url = "https://github.com/introlab/rtabmap/releases/download/0.23.1/vcpkg-export-66c0373d-x64-vs2022.7z"
Invoke-WebRequest -Uri $url -OutFile "vcpkg_export.7z"

# 압축 해제
& "C:\Program Files\7-Zip\7z.exe" x "vcpkg_export.7z" -o"C:\dev\vcpkg_export" -y
```

압축 해제 후 `C:\dev\vcpkg_export\installed\x64-windows-release\` 에 라이브러리 설치됨:

| 라이브러리        | 버전   |
| ----------------- | ------ |
| Qt                | 6.10.0 |
| VTK               | 9.3    |
| PCL               | 1.15.1 |
| OpenCV            | 4.12.0 |
| librealsense      | 2.56.2 |
| g2o, Eigen, Boost | 최신   |

---

## 3. RTAB-Map 소스 빌드

### 3-1. 소스 가져오기

```powershell
cd C:\dev
git clone <이 레포 URL> .
# 또는 이미 있으면:
# git pull
```

### 3-2. CMake 구성

```powershell
$cmakeBin = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$vcpkg = "C:\dev\vcpkg_export\installed\x64-windows-release"
$psapi = "C:\Program Files (x86)\Windows Kits\10\Lib\10.0.26100.0\um\x64\psapi.lib"

cd C:\dev\rtabmap
New-Item -ItemType Directory -Force -Path build
cd build

& $cmakeBin .. `
  -Wno-dev `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_PREFIX_PATH="$vcpkg" `
  -DCMAKE_INSTALL_PREFIX="C:\dev\rtabmap\install" `
  -DWITH_QT=ON `
  -DBUILD_APP=ON `
  -DWITH_REALSENSE2=ON `
  -DBUILD_TOOLS=ON `
  -DBUILD_EXAMPLES=OFF `
  -DPSAPI_LIBRARIES="$psapi"
```

> ⚠️ `$psapi` 경로의 Windows SDK 버전(`10.0.26100.0`)은 본인 환경에 맞게 수정.
> 확인: `Get-ChildItem "C:\Program Files (x86)\Windows Kits\10\Lib" | Select-Object Name`

CMake 출력에서 확인할 항목:

```
--   With RealSense2  = YES
--   With Qt (Qt6)    = YES
--   With VTK         = YES
```

### 3-3. 빌드

```powershell
& $cmakeBin --build . --config Release --parallel
```

> 첫 빌드: 약 5~10분 소요 (CPU에 따라 다름)

빌드 완료 시:

```
rtabmap_app.vcxproj -> C:\dev\rtabmap\build\bin\RTABMap.exe
```

---

## 4. 실행

### 방법 A: 배치 파일 (추천)

```powershell
C:\dev\run_rtabmap.bat
```

더블클릭 또는 PowerShell에서 실행.

### 방법 B: PowerShell 직접 실행

```powershell
$env:PATH = "C:\dev\vcpkg_export\installed\x64-windows-release\bin;C:\dev\rtabmap\build\bin;$env:PATH"
$env:QT_PLUGIN_PATH = "C:\dev\vcpkg_export\installed\x64-windows-release\Qt6\plugins"
& "C:\dev\rtabmap\build\bin\RTABMap.exe"
```

### GUI 패널 구성

| 패널         | 내용                                |
| ------------ | ----------------------------------- |
| 3D Map       | 3D 포인트클라우드 + 카메라 경로     |
| Graph View   | **2D Occupancy Grid** (점유 그리드) |
| Loop Closure | 루프 클로저 탐지 시각화             |
| Odometry     | 프레임간 특징점 매칭                |

---

## 5. 카메라 연결 & SLAM 시작

1. RealSense D455f를 **PC USB 3.0 포트에 직접 연결** (허브 사용 금지)
2. RTABMap 실행
3. **Edit → Preferences → Source type** 섹션:
   - Camera → `RealSense2` 선택
   - OK
4. **Detection → Start (▶)** 클릭
5. 카메라를 들고 천천히 이동 → 3D Map + 2D Grid 실시간 생성

### 자동 적용되는 D455f 최적 파라미터

Start 시 `getCustomParameters()`에서 자동 적용됨 (별도 설정 불필요):

| 파라미터                 | 값     | 의미                       |
| ------------------------ | ------ | -------------------------- |
| `Grid/CellSize`          | `0.05` | 5cm 해상도                 |
| `Grid/RangeMax`          | `6.0`  | D455f depth 상한 6m        |
| `Grid/RangeMin`          | `0.3`  | D455f depth 하한 0.3m      |
| `Grid/RayTracing`        | `true` | 센서~장애물 사이 free 마킹 |
| `Grid/DepthDecimation`   | `2`    | 해상도 2배 축소 (성능)     |
| `Grid/MaxObstacleHeight` | `1.5`  | 1.5m 이상 장애물 무시      |
| `Grid/MaxGroundHeight`   | `0.15` | 바닥 판정 최대 15cm        |
| `Optimizer/GravitySigma` | `0.3`  | IMU gravity constraint     |
| `Odom/AlignWithGround`   | `true` | IMU 기반 중력 정렬         |
| `Reg/Strategy`           | `0`    | Visual registration        |
| `Vis/MinInliers`         | `15`   | 최소 inlier 수             |

---

## 6. TCP Grid Streaming (Unity 연동)

### 활성화

1. RTABMap GUI에서 **Tools → TCP Grid Streaming (port 7777)** 체크
2. 상태바에 "TCP Grid Streaming: listening on port 7777" 표시
3. Start(▶)로 SLAM 시작 → 클라이언트에 자동 전송 (10Hz)

### 패킷 프로토콜 (little-endian)

```
[Header 32 bytes]
  int32  width       — 그리드 가로 셀 수
  int32  height      — 그리드 세로 셀 수
  float  xMin        — 왼쪽 하단 X좌표 (m)
  float  yMin        — 왼쪽 하단 Y좌표 (m)
  float  cellSize    — 셀 크기 (m, 기본 0.05)
  float  poseX       — 로봇 X좌표 (m)
  float  poseY       — 로봇 Y좌표 (m)
  float  poseYaw     — 로봇 방향 (rad)

[Body: width × height bytes]
  int8 per cell: -1=unknown, 0=free, 100=obstacle
```

### 좌표 변환

```
gridX = (poseX - xMin) / cellSize
gridY = (poseY - yMin) / cellSize
```

### Unity C# 예시

```csharp
TcpClient client = new TcpClient("127.0.0.1", 7777);
NetworkStream stream = client.GetStream();

byte[] header = new byte[32];
stream.Read(header, 0, 32);

int width  = BitConverter.ToInt32(header, 0);
int height = BitConverter.ToInt32(header, 4);
float xMin = BitConverter.ToSingle(header, 8);
// ...

byte[] body = new byte[width * height];
stream.Read(body, 0, body.Length);
// body[i]: -1=unknown, 0=free, 100=obstacle
```

### Python 테스트

```powershell
cd C:\dev\rtabmap_2d_pipeline
python test_client.py
```

---

## 7. 소스 수정 & 재빌드

### 우리가 수정/추가한 파일

| 파일                                           | 내용                                               |
| ---------------------------------------------- | -------------------------------------------------- |
| `guilib/include/rtabmap/gui/GridTcpStreamer.h` | **신규** — TCP 서버 클래스 (QTcpServer 기반)       |
| `guilib/src/GridTcpStreamer.cpp`               | **신규** — 구현 (멀티 클라이언트, 10Hz rate limit) |
| `guilib/include/rtabmap/gui/MainWindow.h`      | 수정 — 멤버 변수, 슬롯, getCustomParameters 선언   |
| `guilib/src/MainWindow.cpp`                    | 수정 — TCP 통합, D455f 파라미터, Graph View 기본   |
| `guilib/src/CMakeLists.txt`                    | 수정 — GridTcpStreamer 등록 + Qt6::Network 추가    |

### 빠른 재빌드 (GUI만)

```powershell
$cmakeBin = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
cd C:\dev\rtabmap\build

# GUI 라이브러리만 재빌드 (~30초)
& $cmakeBin --build . --config Release --target rtabmap_gui --parallel

# 앱도 재빌드 (~10초)
& $cmakeBin --build . --config Release --target rtabmap_app --parallel
```

### 새 소스 파일 추가 시

1. `guilib/src/CMakeLists.txt` 에 헤더/소스 등록
2. CMake 재구성 + 전체 빌드:

```powershell
& $cmakeBin .. -Wno-dev
& $cmakeBin --build . --config Release --parallel
```

---

## 8. 트러블슈팅

### Qt platform plugin 에러

```
qt.qpa.plugin: Could not find the Qt platform plugin "windows" in ""
```

→ `QT_PLUGIN_PATH` 설정 필요. `run_rtabmap.bat` 사용 추천.

```powershell
$env:QT_PLUGIN_PATH = "C:\dev\vcpkg_export\installed\x64-windows-release\Qt6\plugins"
```

### DLL을 찾을 수 없음

```
The code execution cannot proceed because XXX.dll was not found
```

→ PATH에 DLL 경로 추가:

```powershell
$env:PATH = "C:\dev\vcpkg_export\installed\x64-windows-release\bin;C:\dev\rtabmap\build\bin;$env:PATH"
```

### psapi.lib 링크 에러

```
LINK : fatal error LNK1181: cannot open input file 'psapi.lib'
```

→ CMake 구성 시 Windows SDK 버전 확인 후 경로 수정:

```powershell
Get-ChildItem "C:\Program Files (x86)\Windows Kits\10\Lib" | Select-Object Name
# → 본인 버전으로 $psapi 경로 업데이트
```

### RealSense 카메라 미감지

- USB 3.0 포트에 **직접** 연결 (허브 사용 금지)
- 케이블 뺐다 다시 꽂기
- 장치 관리자에서 "Intel RealSense D455" 인식 확인

---

## 9. 프로젝트 구조

```
C:\dev\
├── .gitignore
├── run_rtabmap.bat          ← 실행 배치 파일
├── plan.md                  ← 프로젝트 계획
├── SETUP_GUIDE.md           ← 이 문서
│
├── rtabmap/                 ← RTAB-Map 0.23.4 소스 (수정됨)
│   ├── CMakeLists.txt
│   ├── app/src/main.cpp     ← RTABMap.exe 진입점
│   ├── corelib/             ← SLAM 엔진
│   ├── guilib/              ← Qt GUI (★ 우리 수정 여기)
│   │   ├── include/rtabmap/gui/
│   │   │   ├── GridTcpStreamer.h    ★ 신규
│   │   │   └── MainWindow.h        ★ 수정
│   │   └── src/
│   │       ├── GridTcpStreamer.cpp  ★ 신규
│   │       ├── MainWindow.cpp      ★ 수정
│   │       └── CMakeLists.txt      ★ 수정
│   ├── utilite/             ← 유틸리티 라이브러리
│   ├── tools/               ← CLI 도구 (calibration, export 등)
│   └── cmake_modules/       ← CMake Find 모듈
│
├── rtabmap_2d_pipeline/     ← 테스트 유틸리티
│   ├── test_client.py       ← TCP 수신 테스트 (Python)
│   ├── main.cpp             ← headless 파이프라인 (참고용)
│   ├── GridPublisher.h      ← TCP 프로토콜 정의 (참고용)
│   └── CMakeLists.txt
│
├── vcpkg_export/            ← ⬇️ 별도 다운로드 (gitignore)
└── vcpkg-export.7z          ← ⬇️ 별도 다운로드 (gitignore)
```
