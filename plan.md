# Plan: RealSense D455f → RTAB-Map SLAM → 2D Occupancy Grid + 시각화 실시간 파이프라인

> **내 담당 범위**: RealSense D455f 캡처 → RTAB-Map SLAM 매핑 → 2D occupancy grid 변환 + GUI 시각화 → TCP로 같은 PC의 Unity에 실시간 전달
> **현재 상태**: Windows 네이티브 빌드 완료 ✅, RTAB-Map GUI + TCP 스트리밍 통합 완료
> **카메라**: Intel RealSense D455f (내장 6축 IMU, USB 3.0, 유효 depth 0.3–6.0m, 1280×720@30fps)
> **접근 방식**: RTAB-Map 소스 직접 수정 — GUI 기반 (3D Map + 2D Grid + Loop Closure + Odometry 패널 활용)
> **향후 확장**: Semantic SLAM (LLM/VLM) — 확정 시 별도 작업

---

## 내가 할 일 — 단계별 체크리스트

### Step 0-W. Windows 네이티브 GUI 빌드 (현재 방식)

> WSL2 headless 빌드에서 **Windows 네이티브 GUI 빌드**로 전환.
> RTAB-Map 소스를 직접 수정하여 TCP 스트리밍 기능을 GUI에 통합.

- [x] **0-W-1. VS2022 + CMake 확인** ✅
  - Visual Studio 2022 Community (MSVC 19.44)
  - CMake 3.31.6-msvc6 (VS 번들)
  - Windows SDK 10.0.26100.0

- [x] **0-W-2. vcpkg 사전빌드 의존성 설치** ✅
  - RTAB-Map 공식 릴리스 첨부: `vcpkg-export-66c0373d-x64-vs2022.7z`
  - 추출 위치: `C:\dev\vcpkg_export\installed\x64-windows-release\`
  - 주요 라이브러리: Qt 6.10.0, VTK 9.3, PCL 1.15.1, OpenCV 4.12.0

- [x] **0-W-3. RTAB-Map 빌드 (WITH_QT=ON)** ✅

  ```powershell
  cd C:\dev\rtabmap\build
  cmake .. -Wno-dev  # CMake 캐시에 vcpkg 경로 등 설정 완료
  cmake --build . --config Release --parallel
  ```

  - 빌드 결과: `C:\dev\rtabmap\build\bin\RTABMap.exe`
  - 주요 옵션: WITH_QT=ON, BUILD_APP=ON, WITH_REALSENSE2=ON

- [x] **0-W-4. RTAB-Map 소스 수정 — TCP Grid Streaming** ✅
  - **새 파일**: `guilib/include/rtabmap/gui/GridTcpStreamer.h` — Qt TCP 서버
  - **새 파일**: `guilib/src/GridTcpStreamer.cpp` — 구현 (멀티 클라이언트, 10Hz 제한)
  - **수정**: `guilib/include/rtabmap/gui/MainWindow.h` — 멤버/슬롯 추가
  - **수정**: `guilib/src/MainWindow.cpp` — TCP 스트리밍 통합 (Tools 메뉴)
  - **수정**: `guilib/src/CMakeLists.txt` — GridTcpStreamer + Qt6::Network

- [x] **0-W-5. D455f 최적 파라미터 하드코딩** ✅
  - `MainWindow::getCustomParameters()` 오버라이드
  - Grid, Optimizer, Odometry 파라미터 자동 적용 (Start 시)

- [x] **0-W-6. Graph View (2D Grid) 기본 표시** ✅
  - `setDefaultViews()`에서 `dockWidget_graphViewer->setVisible(true)` 변경

- [x] **0-W-7. GUI 실행 확인** ✅
  - RTABMap.exe 정상 실행, 크래시 없음

### Step 1. Grid 파라미터 튜닝

- [ ] **1-1. 실제 환경에서 기본 파라미터로 매핑 테스트**

  | 파라미터                   | 초기값 | 의미                                               |
  | -------------------------- | ------ | -------------------------------------------------- |
  | `RGBD/CreateOccupancyGrid` | `true` | 2D grid 생성 활성화 **(반드시 true)**              |
  | `Optimizer/GravitySigma`   | `0.3`  | **D455f IMU** gravity constraint (g2o/GTSAM 전용)  |
  | `Odom/AlignWithGround`     | `true` | **D455f IMU** 초기 pose 중력 정렬                  |
  | `Grid/CellSize`            | `0.05` | 셀 크기 5cm (작을수록 정밀, 클수록 빠름)           |
  | `Grid/MaxObstacleHeight`   | `1.5`  | 이 높이 이상 장애물 무시 (사람/가구)               |
  | `Grid/MaxGroundHeight`     | `0.15` | 바닥 판정 최대 높이 15cm                           |
  | `Grid/RangeMax`            | `6.0`  | D455f 유효 depth 상한 6m                           |
  | `Grid/RangeMin`            | `0.3`  | D455f 실용 depth 하한                              |
  | `Grid/DepthDecimation`     | `2`    | D455f 1280×720 → 640×360 (2배 축소)                |
  | `Grid/NormalsSegmentation` | `true` | 법선 기반 지면/장애물 분리 (IMU gravity 보정 활용) |
  | `Grid/MaxGroundAngle`      | `45`   | 바닥 판정 최대 법선 각도 (도)                      |
  | `Grid/RayTracing`          | `true` | 센서~장애물 사이를 free로 마킹                     |

- [ ] **1-2. 환경에 맞게 파라미터 조정**
  - 바닥이 장애물로 나옴 → `MaxGroundHeight`↑ 또는 `MaxGroundAngle`↑
  - 맵이 거침 → `CellSize`↓ 또는 `DepthDecimation`↓
  - 처리 느림 → `DepthDecimation`↑ 또는 `RangeMax`↓
  - 벽이 두꺼움 → `CellSize`↓
  - 루프 클로저 미감지 → `Vis/FeatureType` 변경 또는 `Kp/MaxFeatures`↑

### Step 2. 커스텀 파이프라인 코드 작성

> `examples/RGBDMapping/` 예제를 기반으로 Qt GUI를 제거하고, 대신 2D grid 추출 + 출력 모듈(GridPublisher)로 교체

- [x] **2-1. 프로젝트 생성** ✅ 완료

  ```
  rtabmap_2d_pipeline/
  ├── CMakeLists.txt       ← rtabmap::core, rtabmap::utilite + OpenCV highgui 링크 (Qt 불필요)
  ├── main.cpp             ← 스레드 파이프라인 구성 + 이벤트 연결 (D455f IMU 파라미터 포함)
  ├── GridPublisher.h      ← UEventsHandler: 2D grid 추출 + OpenCV 시각화 + TCP + 파일 출력
  └── test_client.py       ← Python TCP 수신 테스트 (OpenCV 시각화)
  ```

- [x] **2-2. main.cpp 작성** ✅ 완료 — 스레드 파이프라인 구성
  - 참조: `examples/RGBDMapping/main.cpp`
  - 구현 내용:
    1. `CameraRealSense2` 생성 + `init()` (자동 감지)
    2. D455f 파라미터: `Optimizer/GravitySigma=0.3`, `Odom/AlignWithGround=true`, Grid/\* 파라미터
    3. `SensorCaptureThread`, `OdometryThread`, `RtabmapThread` 생성
    4. `GridPublisher` 생성 (TCP 7777, 10Hz, output/ 디렉토리) + 이벤트 핸들러 등록
    5. 이벤트 파이프: Camera→Odometry→RTAB-Map
    6. Ctrl+C 시그널로 종료

- [x] **2-3. GridPublisher.h 작성** ✅ 완료 — 2D grid 추출 + **시각화** + 출력
  - `UEventsHandler` 상속, `handleEvent()` 오버라이드
  - **핵심 흐름** (참조: `examples/RGBDMapping/MapBuilder.h` processStatistics):

    ```
    [RtabmapEvent 수신]
    1. stats.getLastSignatureData().sensorData()에서
       uncompressDataConst()로 ground/obstacle/empty cells 추출
    2. LocalGridCache에 add()로 캐싱
    3. OccupancyGrid.update(stats.poses())로 글로벌 맵 갱신
    4. OccupancyGrid.getMap(xMin, yMin)으로 cv::Mat(CV_8S) 추출
       → -1=unknown, 0=free, 100=obstacle

    [OdometryEvent 수신]
    → 현재 로봇 pose (x, y, yaw) 업데이트
    ```

  - **2D 시각화** (OpenCV, 실시간):
    1. `util3d::convertMap2Image8U(map)` → grayscale (black=장애물, light gray=free, dark gray=unknown)
    2. `cvtColor(BGR)` → 로봇 pose 위치에 빨간 원 + 방향 화살표 오버레이
    3. HUD 텍스트 (grid 크기, cell size, pose 좌표)
    4. `cv::imshow("2D Occupancy Grid")` + `waitKey(1)` 비차단 표시

  - 출력 모듈:
    - **TCP 소켓** (같은 PC Unity 연동 — 127.0.0.1:7777, non-blocking accept)
    - **파일 출력** (50프레임마다 PGM+YAML 자동 저장)
  - rate limiting: ~10Hz (0.1초 최소 간격)

- [x] **2-4. CMakeLists.txt 작성** ✅ 완료
  - `find_package(RTABMap REQUIRED COMPONENTS core)` → `rtabmap::core`, `rtabmap::utilite` 링크
  - `find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)` → 시각화용
  - Qt/GUI 의존성 없음

- [ ] **2-5. 이동 테스트 + 그리드 맵 검증**
  - 카메라를 들고 방 안을 이동하며 실행 (5cm+ 이동 필요)
  - 콘솔에 "Grid published: WxH" 로그 출력 확인
  - `output/map.pgm` 파일이 주기적으로 갱신되는지 확인
  - 방을 돌아서 루프 클로저 → 맵 보정 확인

### Step 3. 팀원 연동 (Unity, 같은 PC)

- [ ] **3-1. TCP 프로토콜 확정 + 문서화**
  - 패킷 구조 (little-endian):
    ```
    [Header 32 bytes]
      width(i32), height(i32), xMin(f32), yMin(f32),
      cellSize(f32), poseX(f32), poseY(f32), poseYaw(f32)
    [Body: width × height bytes]
      int8 per cell: -1=unknown, 0=free, 100=obstacle
    ```
  - 좌표 변환: `gridX = (poseX - xMin) / cellSize`
  - Unity 측 접속: `127.0.0.1:7777`

- [ ] **3-2. test_client.py로 TCP 수신 검증**
  - Python에서 TCP 접속 → 헤더 파싱 → OpenCV로 2D 맵 시각화
  - 로봇 위치(빨간 점) + 방향(화살표) 오버레이

- [ ] **3-3. 팀원 Unity 클라이언트와 연동 테스트**
  - localhost TCP 연결 확인
  - grid 데이터 파싱 + 경로 계획 입력으로 사용 가능한지 확인
  - 전송 주기(5~10Hz)와 맵 크기 동적 변화 대응 확인

---

## 참고: 시스템 아키텍처

### 전체 데이터 흐름 (Windows 네이티브 GUI)

```
RealSense D455f (USB3, 30fps)
     │
     ▼
SensorCaptureThread ──SensorEvent──▶ OdometryThread ──OdometryEvent──▶ RtabmapThread
(RGBD 프레임 캡처)                  (프레임간 pose 추정)              (매핑 + 루프클로저)
                                                                           │
                                                                    ┌──────┴──────┐
                                                                    ▼             ▼
                                                             MainWindow    GridTcpStreamer
                                                          (Qt GUI 패널)   (TCP port 7777)
                                                                               │
                                                          3D CloudViewer       ▼
                                                          2D GraphViewer    Unity (경로계획)
                                                          Loop Closure      → HENES 차량 제어
                                                          Odometry View
```

### RTAB-Map 내부 3D→2D 변환 과정

RTAB-Map이 매 키프레임마다 자동으로 수행:

```
depth 이미지 → 3D 포인트클라우드
     │
     ▼
LocalGridMaker::createLocalMap()
  - 법선(normal) 기반으로 ground / obstacle / empty 분류
  - Grid/MaxObstacleHeight 이상 → 무시
  - Grid/MaxGroundAngle 이하 법선 → 바닥
  - 분류된 3D 점을 XY 평면에 투영 → cellSize 해상도의 로컬 그리드
     │
     ▼
OccupancyGrid::assemble()
  - 각 노드의 LocalGrid를 optimized pose로 변환
  - 겹치는 셀은 log-odds 확률 모델로 합산
  - 루프 클로저 보정 시: 변경된 pose만 재계산 (incremental)
     │
     ▼
cv::Mat (CV_8S) — 2D Occupancy Grid
  -1 = unknown,  0 = free,  100 = obstacle
```

### 스레드 & 타이밍

| 스레드                  | 역할                                                         | 주기                   |
| ----------------------- | ------------------------------------------------------------ | ---------------------- |
| **SensorCaptureThread** | RealSense RGBD 프레임 캡처                                   | 30Hz (33ms)            |
| **OdometryThread**      | Visual Odometry (상대 pose 추정)                             | 매 프레임 30-50ms      |
| **RtabmapThread**       | 키프레임 삽입 + 루프 클로저 + 그래프 최적화 + LocalGrid 생성 | 키프레임마다 100-300ms |
| **GridPublisher**       | RtabmapEvent → 2D grid 추출 → 출력                           | 5-10Hz (이벤트 기반)   |

---

## 참고: 핵심 소스코드 참조 테이블

| 참조 대상              | 파일                                                      | 핵심 내용                                                                                |
| ---------------------- | --------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| 스레드 파이프라인 구성 | `examples/RGBDMapping/main.cpp`                           | Camera→Odom→SLAM 스레드 + 이벤트 파이프 구성, RealSense2는 driver=11                     |
| **2D grid 추출 패턴**  | `examples/RGBDMapping/MapBuilder.h` processStatistics()   | `uncompressDataConst()` → `LocalGridCache.add()` → `OccupancyGrid.update()` → `getMap()` |
| OccupancyGrid API      | `corelib/include/rtabmap/core/global_map/OccupancyGrid.h` | `getMap(xMin,yMin)` → CV_8SC1, `update(poses)`, `getCellSize()`                          |
| **2D 시각화 변환**     | `corelib/src/util3d_mapping.cpp` L896                     | `convertMap2Image8U(map8S)` → CV_8U (0→178=free, 100→0=obstacle, -1→89=unknown)          |
| PGM/YAML 저장          | `tools/Export/main.cpp` saveMap()                         | `convertMap2Image8U()` → `imwrite()` + YAML metadata                                     |
| RealSense2 드라이버    | `corelib/include/rtabmap/core/camera/CameraRealSense2.h`  | `setResolution()`, `setEmitterEnabled()`, init 시 640×480 30fps 기본                     |
| **D455f IMU 처리**     | `corelib/src/Odometry.cpp` L314-342                       | IMU orientation alignment, gravity 기반 pose 초기화                                      |
| Statistics 데이터      | `corelib/include/rtabmap/core/Statistics.h`               | `poses()`, `getLastSignatureData()`, `mapCorrection()`                                   |
| LocalGrid 생성         | `corelib/src/LocalGridMaker.cpp`                          | `createLocalMap()` — depth→3D→ground/obstacle/empty 분류→2D 투영                         |
| **roll/pitch 보정**    | `corelib/include/rtabmap/core/impl/LocalMapMaker.hpp` L89 | segmentCloud()에서 pose의 roll/pitch로 cloud 회전 보정 (yaw 제외)                        |

---

## 참고: 리스크 & 대안

| 리스크                     | 영향               | 대안                                                |
| -------------------------- | ------------------ | --------------------------------------------------- |
| RTAB-Map Windows 빌드 실패 | 전체 차질          | 프리빌드 릴리스 사용 또는 WSL2/Docker               |
| RealSense depth 품질 불량  | 맵 품질 저하       | IR emitter 활성화, depth 필터 조정, RangeMax 줄이기 |
| 루프 클로저 미감지         | 맵 drift           | `Kp/MaxFeatures`↑, 카메라 이동 속도 줄이기          |
| 실시간 성능 부족           | 지연               | `DepthDecimation`↑, `CellSize`↑, `RangeMax`↓        |
| TCP 전송 지연              | Unity 맵 갱신 느림 | 전송 주기↓, grid 해상도↓, 압축 추가                 |

---

## 참고: 팀원 전달 사항

1. **TCP 접속**: `127.0.0.1:7777` (같은 PC)
2. **패킷 파싱**: 32바이트 헤더 + width×height 바이트 body (상세: Step 3-1)
3. **좌표 변환**: `gridX = (poseX - xMin) / cellSize`, `gridY = (poseY - yMin) / cellSize`
4. **grid 값**: -1=unknown, 0=free, 100=obstacle
5. **갱신 주기**: 5~10Hz, 맵 크기 동적 변화
6. **HENES 제어**: Serial 115200bps, `V<speed>,A<angle>\n`

---

## 의사결정 로그

| 날짜       | 결정 사항                                         | 근거                                                                                                                                                                                                   |
| ---------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2026-04-15 | TCP 소켓 확정, 공유 메모리 검토 중                | 같은 PC 내 Unity 연동, 공유 메모리는 성능 이점 있으나 복잡도↑                                                                                                                                          |
| 2026-04-15 | 실시간 필수 (후처리 불가)                         | 차량이 주행 중 경로계획에 grid 필요                                                                                                                                                                    |
| 2026-04-17 | D455f 전용 파라미터 추가                          | 내장 IMU 활용: GravitySigma=0.3, AlignWithGround=true                                                                                                                                                  |
| 2026-04-20 | **WSL2 → Windows 네이티브 GUI로 전환**            | vcpkg 사전빌드 export로 libarchive 버그 우회, GUI(3D+2D+LoopClosure) 활용                                                                                                                              |
| 2026-04-20 | RTAB-Map 소스 직접 수정 방식 채택                 | GUI 기능 활용 + TCP 스트리밍 통합, 별도 바이너리 불필요                                                                                                                                                |
| 2026-04-20 | GridTcpStreamer 클래스 추가 (guilib)              | MainWindow에 TCP 서버 통합, Tools 메뉴에서 on/off                                                                                                                                                      |
| 2026-04-20 | getCustomParameters()에 D455f 파라미터            | Start 시 자동 적용, Preferences 대화상자보다 우선                                                                                                                                                      |
| 2026-04-20 | Graph View 기본 표시                              | 2D Grid(점유 그리드) 패널을 기본으로 표시하여 즉시 확인 가능                                                                                                                                           |
| 2026-04-18 | **Unity 가상 카메라 오프라인 연동 추가**          | Unity 시뮬레이션 데이터셋을 RTAB-Map으로 후처리하여 2D 점유 맵 생성                                                                                                                                    |
| 2026-04-18 | `rtabmap_unity_bridge/` 프로젝트 생성             | RealSense 물리 카메라 파이프라인과 분리, CameraRGBDImages 기반 오프라인 배치                                                                                                                           |
| 2026-04-18 | 가상 IMU 생성 (Unity Transform 기반)              | FixedUpdate에서 rotation/position 변화량으로 gyro/accel 계산, CSV 출력                                                                                                                                 |
| 2026-04-18 | `rtabmap-rgbd_dataset` 대신 커스텀 파이프라인     | 기존 도구가 TUM/Bonn만 지원 + depthScale 하드코딩 + IMU 미지원                                                                                                                                         |
| 2026-04-18 | 인라인 IMU 파싱 (CidSimsDataset 패턴)             | 오프라인 배치에 적합, IMUThread 비동기 방식보다 정밀한 타이밍 동기화                                                                                                                                   |
| 2026-04-19 | **커스텀 파이프라인 → RTABMap GUI 방식으로 변경** | RTABMap GUI가 RGBD Images 소스 + IMU CSV를 네이티브 지원 (IMUThread). 별도 C++ 빌드 불필요, 기존 run_rtabmap.bat과 동일한 GUI 유지. rtabmap_unity_bridge/ 프로젝트는 보존하되 주력은 GUI 방식으로 전환 |

---

## Step 4. Unity 가상 카메라 → RTAB-Map 연동

> Unity 시뮬레이션에서 캡처한 RGBD + IMU 데이터셋을 RTABMap GUI에서 오프라인 처리.
> 기존 `run_rtabmap.bat`과 동일한 GUI 유지, 소스만 RGBD Images + IMU CSV로 변경.
>
> **방식 변경 (2026-04-19)**: 커스텀 C++ 파이프라인(`rtabmap_unity_bridge/`) 대신
> RTABMap.exe GUI의 내장 RGBD Images 소스 + IMUThread를 활용.

### 4-A. GUI 방식 (현재 주력) ✅

**원리**: RTABMap GUI가 이미 `CameraRGBDImages` + `IMUThread`를 네이티브 지원.
별도 빌드 없이 GUI Preferences에서 경로만 설정하면 됨.

**실행**: `run_rtabmap_unity.bat` → RTABMap.exe 실행 → GUI에서 Start

**GUI 설정 (최초 1회)**:

| 설정 항목                | 위치                                | 값                              |
| ------------------------ | ----------------------------------- | ------------------------------- |
| Source Type              | Preferences → Source → RGB-D Camera | **RGBD Images**                 |
| RGB path                 | 같은 패널                           | `데이터셋/rgb_sync/`            |
| Depth path               | 같은 패널                           | `데이터셋/depth_sync/`          |
| Depth scale factor       | 같은 패널                           | **1000** (16-bit mm → meter)    |
| Filenames are timestamps | 같은 패널                           | ✅ 체크                         |
| IMU path                 | Preferences → Source → Images 옵션  | `데이터셋/imu.csv`              |
| IMU local transform      | 같은 패널                           | `0 0 1 0 -1 0 0 0 0 -1 0 0`     |
| IMU rate                 | 같은 패널                           | `0` (무제한)                    |
| Calibration              | 데이터셋 폴더 내                    | `d455_virtual.yaml` (자동 감지) |

**IMU 포맷 호환성**:

- Unity `RGBD_DataCollector.cs`의 `imu.csv` 출력: `timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z`
- RTABMap `IMUThread.cpp` 입력: 동일한 EuRoC CSV 형식 (첫 줄 헤더 skip, 쉼표 구분)
- timestamp에 `.` 포함 → epoch seconds로 파싱 (상대 시간 OK)

### 4-B. 커스텀 파이프라인 (보존, 백업용)

### 4-0. 아키텍처

```
Unity (RGBD_DataCollector)   → 디스크 저장
  │  rgb_sync/*.png (1280×720, 8-bit RGB)
  │  depth_sync/*.png (1280×720, 16-bit mm)
  │  imu.csv (timestamp,gx,gy,gz,ax,ay,az)
  │  d455_virtual.yaml (캘리브레이션)
  ▼
rtabmap_unity_offline.exe    → 오프라인 배치 처리
  CameraRGBDImages(디스크) → SensorCaptureThread(IMU필터) → Odometry → Rtabmap
                                                                  │
                                                           GridPublisher
                                                         ┌───────┼────────┐
                                                         ▼       ▼        ▼
                                                      OpenCV   TCP     PGM+YAML
                                                      Window   7777    output/
```

### 4-1. 프로젝트 구조

- [x] **`rtabmap_unity_bridge/`** ✅ 생성 완료

  ```
  rtabmap_unity_bridge/
  ├── CMakeLists.txt          ← 빌드 설정 (rtabmap::core + OpenCV, Qt 불필요)
  ├── main.cpp                ← 오프라인 배치 파이프라인 (CameraRGBDImages + IMU)
  ├── GridPublisher.h         ← 2D grid 추출 + 시각화 + TCP + 파일 (processOffline 추가)
  ├── run_offline.bat         ← 실행 스크립트
  └── README.md               ← 사용법 + 데이터셋 포맷 문서
  ```

### 4-2. Unity 측 수정

- [x] **RGBD_DataCollector.cs — IMU CSV 출력 추가** ✅

  | 추가 항목                | 설명                                                     |
  | ------------------------ | -------------------------------------------------------- |
  | `FixedUpdate()` IMU 기록 | `transform.rotation/position` 변화량에서 gyro/accel 계산 |
  | `imu.csv` StreamWriter   | `timestamp,gx,gy,gz,ax,ay,az` 포맷, ~50Hz (FixedUpdate)  |
  | 중력 보상                | specific force = physical_accel + (0, 9.81, 0)           |
  | 좌표 변환                | 월드 프레임 → 카메라 로컬 프레임 (Inverse rotation)      |

- [x] **verify_dataset.py — IMU 검증 추가** ✅

  | 검증 항목       | 내용                                                     |
  | --------------- | -------------------------------------------------------- |
  | imu.csv 존재    | 선택적 — 없으면 --no-imu로 실행 가능                     |
  | 헤더 형식       | `timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z` |
  | 샘플 수/율/기간 | 50Hz 근처, 이미지 기간과 매칭                            |
  | 중력 참조       | 정지 시 accel_y ≈ 9.81                                   |

### 4-3. C++ 파이프라인

- [x] **main.cpp 작성** ✅ — CidSimsDataset 패턴 기반 오프라인 배치 루프

  핵심 구현:
  1. `CameraRGBDImages(rgb_sync, depth_sync, depthScaleFactor=1.0)` 생성
  2. `setTimestamps(true, "", false)` — 파일명이 타임스탬프
  3. `SensorCaptureThread(camera, params)` — postUpdate용 (스레드 아님)
  4. `enableIMUFiltering(1)` — Complementary filter
  5. `camera->init(datasetPath, "d455_virtual")` — 캘리브레이션 로드
  6. 배치 루프: IMU 인라인 파싱 → odom->process() → rtabmap.process()
  7. `GridPublisher.processOffline(stats, odomPose)` — 직접 호출

- [x] **GridPublisher.h 수정** ✅ — `processOffline()` + `saveFinalMap()` 추가

### 4-4. 좌표계 변환

| 변환                        | 행렬 (3×4, row-major)           | 설명                                |
| --------------------------- | ------------------------------- | ----------------------------------- |
| camera optical → base_link  | `[0,0,1,0, -1,0,0,0, 0,-1,0,0]` | d455_virtual.yaml `local_transform` |
| base_link → IMU (Unity cam) | `[0,-1,0,0, 0,0,1,0, 1,0,0,0]`  | main.cpp `baseToImu`                |

### 4-5. 빌드 & 실행

- [ ] **빌드 확인**

  ```powershell
  cd C:\dev\rtabmap_unity_bridge\build
  cmake .. -DCMAKE_PREFIX_PATH="C:/dev/vcpkg_export/installed/x64-windows-release;C:/dev/rtabmap/build/install"
  cmake --build . --config Release
  ```

- [ ] **Unity에서 데이터셋 수집 → 검증 → 실행**

  ```powershell
  python verify_dataset.py C:\D455_Dataset
  run_offline.bat C:\D455_Dataset
  ```

- [ ] **TCP + test_client.py로 그리드 수신 확인**
