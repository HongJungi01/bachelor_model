# Plan: RealSense D455f → RTAB-Map SLAM → 2D Occupancy Grid + 시각화 실시간 파이프라인

> **내 담당 범위**: RealSense D455f 캡처 → RTAB-Map SLAM 매핑 → 2D occupancy grid 변환 + OpenCV 시각화 → TCP로 같은 PC의 Unity에 실시간 전달
> **현재 상태**: 빌드 미완, RealSense D455f 보유(미테스트), 커스텀 C++ 코드 작성 완료
> **카메라**: Intel RealSense D455f (내장 6축 IMU, USB 3.0, 유효 depth 0.3–6.0m, 1280×720@30fps)

---

## 내가 할 일 — 단계별 체크리스트

### Step 0. 환경 구축

- [x] **0-1. Visual Studio 2022 설치** ✅ (Windows 빌드 포기, WSL2로 전환)

- [x] **0-2. WSL2 Ubuntu 22.04 설치** ✅ 완료
  - VirtualMachinePlatform + WSL 기능 활성화 → 재부팅 → Ubuntu-22.04 설치
  - 사용자: `dev` / 비밀번호: `1234` / NOPASSWD sudo
  - 커널: 6.6.87.2-microsoft-standard-WSL2

- [x] **0-3. WSL2에서 RTAB-Map 소스 빌드** ✅ 완료
  - apt 의존성 695패키지 설치 (OpenCV 4.5.4, PCL 1.12.1, Boost 1.74)
  - **librealsense2 2.57.7**: 소스 빌드 with `-DFORCE_RSUSB_BACKEND=ON` (WSL2 UVC 모듈 없음)
  - RTAB-Map 0.23.4: RealSense2=YES, OctoMap=YES, Qt=OFF → `/usr/local` 설치
  - 커스텀 파이프라인: `~/dev/rtabmap_2d_pipeline/build/rtabmap_2d_pipeline`

  ```bash
  # librealsense RSUSB 빌드 (WSL2 전용 — V4L2 불가)
  cd ~/librealsense/build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF
  make -j4 && sudo make install

  # RTAB-Map 빌드
  cd ~/dev/rtabmap/build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DWITH_QT=OFF -DWITH_REALSENSE2=ON \
    -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF
  make -j4 && sudo make install

  # 파이프라인 빌드
  cd ~/dev/rtabmap_2d_pipeline/build
  cmake .. && make -j4
  ```

- [x] **0-4. RealSense USB passthrough (usbipd-win)** ✅ 완료
  - usbipd-win 5.3.0 설치, linux-tools-generic + hwdata 설치
  - RealSense D455F: BUSID **5-3** (8086:0b5c), USB 3.2
  - ⚠️ USB 권한: 매번 `sudo chmod 666 /dev/bus/usb/002/*` 필요 (udev 미작동)

  ```powershell
  # Windows: WSL 시작 후 attach
  wsl -d Ubuntu-22.04 -u dev -- bash -c "sleep 120" &
  usbipd bind --busid 5-3   # 최초 1회
  usbipd attach --wsl --busid 5-3
  ```

  ```bash
  # WSL2: USB 권한 설정 + 실행
  sudo chmod 666 /dev/bus/usb/002/*
  cd ~/dev/rtabmap_2d_pipeline/build
  ./rtabmap_2d_pipeline
  ```

- [x] **0-5. 파이프라인 단독 실행 확인** ✅ 완료 (Qt GUI 없이 직접 테스트)
  - RealSense D455F 감지: S/N 254122300022, FW 5.15.1.55, IMU BMI085
  - Odometry: ~30fps, lost=false, features 300-730, inliers 90-165
  - ⚠️ IMU orientation 미설정 경고 → visual odometry만 사용 중 (정상 동작)

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

### 전체 데이터 흐름

```
RealSense D4xx (USB3, 30fps)
     │
     ▼
SensorCaptureThread ──SensorEvent──▶ OdometryThread ──OdometryEvent──▶ RtabmapThread
(RGBD 프레임 캡처)                  (프레임간 pose 추정)              (매핑 + 루프클로저)
                                                                           │
                                                                           ▼
                                                                     RtabmapEvent
                                                                   (Statistics 포함)
                                                                           │
                                                                           ▼
                                                                    GridPublisher
                                                                  (2D grid 추출 + 전송)
                                                                      │         │
                                                                      ▼         ▼
                                                                 TCP 소켓    파일 출력
                                                              (localhost:7777) (PGM+YAML)
                                                                      │
                                                                      ▼
                                                                Unity (경로계획)
                                                                → HENES 차량 제어
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

| 날짜       | 결정 사항                                   | 근거                                                                         |
| ---------- | ------------------------------------------- | ---------------------------------------------------------------------------- |
| 2026-04-15 | 커스텀 C++ 코드 작성 (기존 도구만으로 부족) | RTAB-Map GUI는 grid TCP 출력 미지원, 실시간 스트리밍 필요                    |
| 2026-04-15 | TCP 소켓 확정, 공유 메모리 검토 중          | 같은 PC 내 Unity 연동, 공유 메모리는 성능 이점 있으나 복잡도↑                |
| 2026-04-15 | 실시간 필수 (후처리 불가)                   | 차량이 주행 중 경로계획에 grid 필요                                          |
| 2026-04-18 | librealsense RSUSB 백엔드 소스 빌드         | WSL2 커널에 uvcvideo 모듈 없음 → V4L2 불가 → FORCE_RSUSB_BACKEND=ON          |
| 2026-04-18 | USB 권한: 수동 chmod 666 (udev 미작동)      | WSL2에서 udevd 미실행, 매 attach 시 `sudo chmod 666 /dev/bus/usb/002/*` 필요 |
| 2026-04-18 | make -j4 (nproc 대신)                       | WSL2 메모리 제한으로 -j$(nproc) 시 OOM kill 발생                             |
| 2026-04-15 | RGBDMapping 예제 기반 확정                  | MapBuilder.h에 2D grid 추출 패턴이 이미 구현되어 있음                        |
| 2026-04-17 | D455f 전용 파라미터 추가                    | 내장 IMU 활용: GravitySigma=0.3, AlignWithGround=true                        |
| 2026-04-17 | OpenCV 시각화 추가 (Qt 불필요)              | convertMap2Image8U() + imshow로 동작 확인, GUI 의존성 제거                   |
| 2026-04-17 | 커스텀 코드 4개 파일 작성 완료              | main.cpp, GridPublisher.h, CMakeLists.txt, test_client.py                    |
| 2026-04-18 | Windows 빌드 포기 → WSL2 전환               | CMake 3.31 libarchive tar 버그 + MAX_PATH 문제 → vcpkg 빌드 불가             |
| 2026-04-18 | Ubuntu 22.04 (Jammy) 선택                   | RTAB-Map Docker 레시피 Jammy 기반, librealsense2 apt 패키지 지원             |
| 2026-04-18 | GridPublisher.h Linux 헤더 수정             | fcntl.h, sys/stat.h 추가 (fcntl(), mkdir() POSIX 함수용)                     |
