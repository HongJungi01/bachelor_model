# RTAB-Map Pipeline — Real-time 2D Occupancy Grid

D455 RGBD → RTAB-Map SLAM → 2D 점유 그리드(Occupancy Grid)를 생성하는 단일 파이프라인.
**실시간 RealSense D455f**와 **Unity 가상 D455 (TCP 스트리밍)** 를 같은 바이너리에서 처리합니다.

## 아키텍처

```
                                                    ┌──── --source realsense ────┐
                                                    │   (실시간, 스레드 이벤트)      │
                                  RealSense D455f ─▶│   CameraThread → OdometryThread → RtabmapThread
                                                    │                                       │
                                                    └───────────────────────────────────────┤
                                                                                            ▼
                                                                                      GridPublisher
                                                                                    ┌───────┼────────┐
                                                                                    ▼       ▼        ▼
                                                                                 OpenCV   TCP     PGM+YAML
                                                                                 Window   7777    output/
                                                                                            ▲
                                                    ┌──── --source unity ────────┐         │
                                                    │   (실시간 TCP 스트리밍)       │         │
   Unity 시뮬레이터 ──TCP 7778──▶ CameraUnityTCP ──▶│   takeData() loop, IMU drain         │
   (RGB+Depth+IMU)                                  │   (단일 스레드)               ────────┘
                                                    └────────────────────────────┘
```

| 모드      | 입력 소스               | 흐름                                       | 용도                |
| --------- | ----------------------- | ------------------------------------------ | ------------------- |
| realsense | `CameraRealSense2`      | UEventsManager 기반 스레드 파이프라인       | 실제 D455f 주행     |
| unity     | `CameraUnityTCP` (서버) | TCP에서 RGBD/IMU 수신 → 단일 스레드 SLAM 루프 | Unity 가상 시뮬레이션 |

두 모드는 동일한 SLAM/그리드 파라미터(`makeParams`)와 동일한 `GridPublisher`를 공유합니다.

## 빌드

### 사전 요구사항

- Visual Studio 2022 (MSVC) / Linux GCC 9+
- CMake 3.14+
- RTAB-Map 빌드 + install (`C:\dev\rtabmap\build\install`)
- vcpkg 의존성 (`C:\dev\vcpkg_export\installed\x64-windows-release\`)
- RealSense2 SDK (realsense 모드 사용 시: RTAB-Map을 `-DWITH_REALSENSE2=ON`으로 빌드)

### 빌드 명령

```powershell
cd C:\dev\rtabmap_pipeline
mkdir build
cd build

cmake .. -DCMAKE_PREFIX_PATH="C:/dev/vcpkg_export/installed/x64-windows-release;C:/dev/rtabmap/build/install"
cmake --build . --config Release
```

## 실행

### Live (RealSense D455f)

```powershell
run_realsense.bat
run_realsense.bat --db C:\maps\session1.db
run_realsense.bat --output C:\maps --tcp-port 8888
```

### Unity TCP 스트리밍

```powershell
REM 1. 먼저 파이프라인을 띄움 (Unity 연결 대기)
run_unity.bat

REM 2. Unity Play → RTABMapStreamer 가 자동 연결
```

옵션:

```
--db <path>           DB 경로 (기본: rtabmap.db)
--output <dir>        PGM/YAML 출력 (기본: output)
--tcp-port <port>     Grid 출력 포트 (기본: 7777)
--sensor-port <port>  Unity 센서 입력 포트 (기본: 7778)
```

## Unity 측 설정

### 필수 컴포넌트

| GameObject       | 컴포넌트                                                                                                       |
| ---------------- | ------------------------------------------------------------------------------------------------------------ |
| Car (root)       | `Rigidbody` (kinematic, useGravity off) · `CarController` · `RTABMapStreamer` · `IMUSensor`                  |
| D455_VirtualCam  | `Camera` · `RGBD_DataCollector` · DepthMaterial(`DepthGrayscale.shader`)                                     |

`RTABMapStreamer`는 `RGBD_DataCollector`와 `IMUSensor`를 자동 검색합니다 (Inspector에서 수동 할당도 가능).

### 데이터 흐름 (Unity 내부)

```
CarController (kinematic Rigidbody)
        │
        ▼
IMUSensor.FixedUpdate()  ─── body-frame gyro/accel ──┐
                                                      ▼
RGBD_DataCollector.OnRenderImage() ── RGB+Depth ────▶ RTABMapStreamer ──TCP──▶ rtabmap_pipeline
```

- IMU 측정값은 `Rigidbody.linearVelocity` / `angularVelocity` (있을 때) 또는 transform 미분으로 계산
- `transform.InverseTransformDirection()`으로 IMU **body frame**으로 변환 (실제 IMU와 동일)
- 중력은 body frame에 투영되어 specific force로 더해짐

## 좌표계

| 프레임             | X      | Y    | Z   |
| ------------------ | ------ | ---- | --- |
| Unity 월드         | 오른쪽 | 위   | 앞  |
| RTAB-Map base_link | 앞     | 왼쪽 | 위  |
| Unity 카메라/IMU body | 오른쪽 | 위   | 앞  |
| OpenCV optical     | 오른쪽 | 아래 | 앞  |

### 변환 관계

- **calibration `local_transform`** (`RGBD_DataCollector.GetCalibration()` → 첫 패킷): OpenCV optical → base_link
  ```
  [0, 0, 1, 0,  -1, 0, 0, 0,  0, -1, 0, 0]
  ```
- **IMU `baseToImu`** (`CameraUnityTCP::handleCalib`): base_link → Unity body frame
  ```
  [0, -1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0]
  ```

## 출력물

1. **OpenCV 시각화 창** — 실시간 점유 그리드, 빨간 점/화살표가 현재 카메라 위치/방향
2. **TCP 스트리밍** (127.0.0.1:7777, **출력**) — `test_client.py`로 수신 테스트
3. **PGM + YAML** — 50 키프레임마다 자동 저장, 종료 시 최종 맵 저장
4. **RTAB-Map DB** — RTABMap GUI에서 열어 3D 재생 가능 (또는 `rtabmap-reprocess`)

### Grid TCP 패킷 (포트 7777, 출력)

```
Header (32 bytes, little-endian):
  width(i32), height(i32), xMin(f32), yMin(f32),
  cellSize(f32), poseX(f32), poseY(f32), poseYaw(f32)
Body:
  width × height bytes — int8 per cell:
    -1 = unknown, 0 = free, 100 = obstacle
```

### Sensor TCP 패킷 (포트 7778, 입력 — Unity → 파이프라인)

공통 헤더: `[type:u8][payloadSize:u32]` (5 bytes, little-endian)

| Type | 이름  | Payload                                                                                  |
| ---- | ----- | ---------------------------------------------------------------------------------------- |
| 1    | Calib | `width:u32` `height:u32` `fx,fy,cx,cy:f64` `localTransform[12]:f64` (연결 직후 1회)        |
| 2    | IMU   | `stamp:f64` `gx,gy,gz:f64` `ax,ay,az:f64` (~50–200 Hz, FixedUpdate 주기)                  |
| 3    | RGBD  | `stamp:f64` `width:u32` `height:u32` `rgb[W*H*3]:u8` `depth[W*H*2]:u8(uint16 mm LE)`     |

이미지는 **top-left origin** (OpenCV/PNG 컨벤션). RGB는 RGB24 raw, Depth는 uint16 mm 단위.

## 파일 구조

```
rtabmap_pipeline/
├── CMakeLists.txt          ← 단일 바이너리
├── main.cpp                ← CLI + makeParams + runLive/runUnity
├── GridPublisher.h         ← UEventsHandler + processOffline() (그리드 송출)
├── CameraUnityTCP.h        ← rtabmap::Camera 서브클래스 (센서 수신)
├── test_client.py          ← Grid TCP 수신 테스트
├── run_realsense.bat       ← realsense 모드 단축 스크립트
├── run_unity.bat           ← unity 모드 단축 스크립트
└── build/                  ← (gitignored) CMake 빌드 출력
```
