# RTAB-Map Pipeline — 2D Occupancy Grid (live + offline)

D455 RGBD → RTAB-Map SLAM → 2D 점유 그리드(Occupancy Grid)를 생성하는 단일 파이프라인.
실시간 RealSense D455f와 Unity 가상 D455 데이터셋을 같은 바이너리에서 처리합니다.

## 아키텍처

```
                     ┌──── --source realsense ────┐
                     │   (실시간, 스레드 이벤트)      │
RealSense D455f ────▶│   CameraThread → OdometryThread → RtabmapThread
                     │                                       │
                     └───────────────────────────────────────┤
                                                             ▼
                                                       GridPublisher
                                                     ┌───────┼────────┐
                                                     ▼       ▼        ▼
                                                  OpenCV   TCP     PGM+YAML
                                                  Window   7777    output/
                                                     ▲       ▲        ▲
                     ┌───────────────────────────────┤
Unity rgb_sync/ ────▶│   CameraRGBDImages → 배치 루프 → IMU 보간
depth_sync/          │   (--source images <path>, 비스레드)
imu.csv (선택)       │
d455_virtual.yaml    └───────────────────────────────┘
```

두 모드는 동일한 SLAM/그리드 파라미터(`makeParams`)와 동일한 `GridPublisher`를 사용합니다.
차이점은 카메라 소스와 제어 흐름뿐입니다.

| 모드     | 소스                 | 흐름                                     | 용도                |
| -------- | -------------------- | ---------------------------------------- | ------------------- |
| live     | `CameraRealSense2`   | RTAB-Map UEventsManager 기반 스레드 파이프라인 | 실제 D455f 주행     |
| offline  | `CameraRGBDImages`   | `takeData()` 동기 루프, IMU CSV 보간       | Unity 가상 데이터셋 |

## 빌드

### 사전 요구사항

- Visual Studio 2022 (MSVC)
- CMake 3.14+
- RTAB-Map 빌드 + install (`C:\dev\rtabmap\build\install`)
- vcpkg 의존성 (`C:\dev\vcpkg_export\installed\x64-windows-release\`)
- RealSense2 SDK (live 모드 사용 시: RTAB-Map을 `-DWITH_REALSENSE2=ON`으로 빌드)

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

### Offline (Unity 데이터셋)

```powershell
run_offline.bat C:\D455_Dataset
run_offline.bat C:\D455_Dataset --no-imu
run_offline.bat C:\D455_Dataset --tcp-port 8888 --output C:\maps
```

### 직접 호출

```powershell
rtabmap_pipeline.exe --source realsense [options]
rtabmap_pipeline.exe --source images <dataset_path> [options]

Options:
  --db <path>        DB 경로 (기본: rtabmap.db | <dataset>/rtabmap.db)
  --output <dir>     PGM/YAML 출력 (기본: output | <dataset>/output)
  --tcp-port <port>  TCP 포트 (기본: 7777)
  --no-imu           IMU 무시 (offline 모드 전용)
```

## Unity 데이터셋 구조 (offline 모드)

`RGBD_DataCollector` 컴포넌트가 생성:

```
{dataset_path}/
├── rgb_sync/           ← 8-bit RGB PNG (1280×720)
│   ├── 0.000000.png
│   └── ...
├── depth_sync/         ← 16-bit unsigned PNG, mm 단위 (CV_16UC1)
│   ├── 0.000000.png
│   └── ...
├── imu.csv             ← 가상 IMU (선택, 50Hz)
└── d455_virtual.yaml   ← OpenCV YAML 캘리브레이션
```

### IMU CSV 포맷

```csv
timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
0.000000,0.000000,0.000000,0.000000,0.000000,9.810000,0.000000
0.020000,0.001234,-0.000567,0.002345,0.012000,9.808000,-0.005000
```

- timestamp: 초 단위 (이미지와 동일 시간 기준, 0부터 시작)
- gyro: rad/s, Unity 카메라 로컬 프레임
- accel: m/s², Unity 카메라 로컬 프레임 (중력 포함)
- Unity 카메라: X-오른쪽, Y-위, Z-앞

## 출력물

1. **OpenCV 시각화 창** — 실시간 점유 그리드, 빨간 점/화살표가 현재 카메라 위치/방향
2. **TCP 스트리밍** (127.0.0.1:7777) — `test_client.py`로 수신 테스트
3. **PGM + YAML** — 50 키프레임마다 자동 저장, 종료 시 최종 맵 저장
4. **RTAB-Map DB** — RTABMap GUI에서 열어 3D 재생 가능

### TCP 패킷

```
Header (32 bytes, little-endian):
  width(i32), height(i32), xMin(f32), yMin(f32),
  cellSize(f32), poseX(f32), poseY(f32), poseYaw(f32)
Body:
  width × height bytes — int8 per cell:
    -1 = unknown, 0 = free, 100 = obstacle
```

## 좌표계

| 프레임             | X      | Y    | Z   |
| ------------------ | ------ | ---- | --- |
| Unity 월드         | 오른쪽 | 위   | 앞  |
| RTAB-Map base_link | 앞     | 왼쪽 | 위  |
| Unity 카메라 (IMU) | 오른쪽 | 위   | 앞  |
| OpenCV optical     | 오른쪽 | 아래 | 앞  |

### 변환 관계

- **calibration YAML `local_transform`**: 카메라 optical → base_link
  ```
  [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0]
  ```
- **IMU `baseToImu`**: base_link → Unity 카메라 프레임
  ```
  [0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0]
  ```

## 파일 구조

```
rtabmap_pipeline/
├── CMakeLists.txt       ← 단일 바이너리 (rtabmap_pipeline.exe)
├── main.cpp             ← CLI + makeParams + runLive/runOffline
├── GridPublisher.h      ← UEventsHandler + processOffline()
├── test_client.py       ← TCP 수신 테스트
├── run_realsense.bat    ← live 모드 단축 스크립트
├── run_offline.bat      ← offline 모드 단축 스크립트
└── build/               ← (gitignored) CMake 빌드 출력
```
