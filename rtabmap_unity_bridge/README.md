# RTAB-Map Unity Bridge — 오프라인 파이프라인

Unity 가상 D455 카메라로 촬영한 RGBD 데이터셋을 RTAB-Map으로 오프라인 처리하여
2D 점유 그리드(Occupancy Grid)를 생성하는 파이프라인입니다.

## 아키텍처

```
Unity (RGBD_DataCollector)
  │  rgb_sync/*.png (1280×720, 8-bit RGB)
  │  depth_sync/*.png (1280×720, 16-bit mm)
  │  imu.csv (timestamp,gx,gy,gz,ax,ay,az)
  │  d455_virtual.yaml (캘리브레이션)
  ▼
rtabmap_unity_offline.exe
  CameraRGBDImages → SensorCaptureThread(postUpdate) → Odometry → Rtabmap
                                                                    │
                                                             GridPublisher
                                                           ┌───────┼────────┐
                                                           ▼       ▼        ▼
                                                        OpenCV   TCP     PGM+YAML
                                                        Window   7777    output/
```

## 데이터셋 구조

Unity `RGBD_DataCollector`가 생성하는 폴더:

```
{dataset_path}/
├── rgb_sync/           ← 8-bit RGB PNG (1280×720)
│   ├── 0.000000.png
│   ├── 0.100000.png
│   └── ...
├── depth_sync/         ← 16-bit unsigned PNG, mm 단위 (CV_16UC1)
│   ├── 0.000000.png
│   ├── 0.100000.png
│   └── ...
├── imu.csv             ← 가상 IMU 데이터 (선택, 50Hz)
└── d455_virtual.yaml   ← OpenCV YAML 캘리브레이션
```

### IMU CSV 포맷

```csv
timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
0.000000,0.000000,0.000000,0.000000,0.000000,9.810000,0.000000
0.020000,0.001234,-0.000567,0.002345,0.012000,9.808000,-0.005000
```

- **timestamp**: 초 단위 (이미지와 동일한 시간 기준, 0부터 시작)
- **gyro_x/y/z**: 각속도 (rad/s, Unity 카메라 로컬 프레임)
- **accel_x/y/z**: 가속도 (m/s², Unity 카메라 로컬 프레임, 중력 포함)
- **좌표계**: Unity 카메라 — X: 오른쪽, Y: 위, Z: 앞

## 빌드 방법

### 사전 요구사항

- Visual Studio 2022 (MSVC)
- CMake 3.14+
- RTAB-Map 빌드 완료 (`C:\dev\rtabmap\build\`)
- vcpkg 의존성 (`C:\dev\vcpkg_export\installed\x64-windows-release\`)

### 빌드 명령

```powershell
cd C:\dev\rtabmap_unity_bridge
mkdir build
cd build

cmake .. -DCMAKE_PREFIX_PATH="C:/dev/vcpkg_export/installed/x64-windows-release;C:/dev/rtabmap/build/install"

cmake --build . --config Release
```

> **참고**: `CMAKE_PREFIX_PATH`에 RTAB-Map 설치 경로를 포함해야 합니다.
> RTAB-Map 빌드 후 `cmake --install . --config Release`를 실행하여
> `C:/dev/rtabmap/build/install`에 설치했는지 확인하세요.

## 실행 방법

### 기본 실행

```powershell
run_offline.bat C:\D455_Dataset
```

### 모든 옵션

```powershell
rtabmap_unity_offline.exe <dataset_path> [options]

Options:
  --output <dir>      PGM/YAML 출력 디렉토리 (기본: <dataset>/output)
  --db <path>         RTAB-Map 데이터베이스 경로 (기본: <dataset>/rtabmap.db)
  --no-imu            IMU 데이터 무시 (imu.csv 있어도 사용 안 함)
  --tcp-port <port>   TCP 포트 (기본: 7777)
  --no-viz            OpenCV 시각화 창 비활성화
```

### 실행 예시

```powershell
# 기본 실행 (IMU 포함, TCP 7777, OpenCV 시각화)
run_offline.bat C:\D455_Dataset

# IMU 없이 실행
run_offline.bat C:\D455_Dataset --no-imu

# 커스텀 TCP 포트 + 출력 경로
run_offline.bat C:\D455_Dataset --tcp-port 8888 --output C:\maps
```

## 출력물

### 1. OpenCV 시각화 창

- 실시간 2D 점유 그리드 표시
- 빨간 점: 현재 카메라 위치
- 빨간 화살표: 카메라 방향
- HUD: 그리드 크기, 셀 크기, 좌표

### 2. TCP 스트리밍 (127.0.0.1:7777)

- `test_client.py`로 수신 테스트 가능
- 패킷 구조: 32바이트 헤더 + width×height 바이트 body

### 3. PGM + YAML 파일

- 50 키프레임마다 자동 저장
- 처리 완료 시 최종 맵 저장
- `output/map.pgm` + `output/map.yaml`

### 4. RTAB-Map 데이터베이스

- `<dataset>/rtabmap.db` — RTAB-Map GUI에서 열어 3D 재생 가능

## Unity 데이터 수집 방법

1. Unity 프로젝트에서 `RGBD_DataCollector` 컴포넌트 설정:
   - `saveDirectory`: 원하는 저장 경로 (예: `C:/D455_Dataset`)
   - `captureFPS`: 10~15 권장
   - `saveImages`: ✅ 체크
   - `captureOnStart`: ✅ 체크

2. Play 모드 실행 → 시뮬레이션 주행 → 정지

3. 데이터셋 검증:

   ```powershell
   python verify_dataset.py C:\D455_Dataset
   ```

4. 오프라인 처리 실행:
   ```powershell
   run_offline.bat C:\D455_Dataset
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

## 기존 파이프라인과의 관계

| 구성 요소                   | 용도                     | 카메라 소스                 |
| --------------------------- | ------------------------ | --------------------------- |
| `rtabmap_2d_pipeline/`      | 실시간 물리 카메라       | RealSense D455f (USB)       |
| **`rtabmap_unity_bridge/`** | **오프라인 가상 카메라** | **Unity 데이터셋 (디스크)** |

GridPublisher의 TCP 프로토콜(7777)과 출력 포맷은 동일하므로,
Unity 측 맵 수신 코드는 변경 없이 재사용 가능합니다.
