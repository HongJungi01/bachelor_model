# WSL2 빌드 & 세팅 가이드

> RealSense D455f → RTAB-Map SLAM → 2D Occupancy Grid 파이프라인
>
> 작성일: 2026-04-18 | OS: Windows + WSL2 Ubuntu 22.04 (Jammy)

---

## 목차

1. [사전 조건](#1-사전-조건)
2. [WSL2 설치 및 설정](#2-wsl2-설치-및-설정)
3. [소스코드 복사](#3-소스코드-복사)
4. [apt 의존성 설치](#4-apt-의존성-설치)
5. [librealsense2 소스 빌드 (RSUSB 백엔드)](#5-librealsense2-소스-빌드-rsusb-백엔드)
6. [RTAB-Map 빌드](#6-rtab-map-빌드)
7. [커스텀 파이프라인 빌드](#7-커스텀-파이프라인-빌드)
8. [RealSense USB Passthrough (usbipd-win)](#8-realsense-usb-passthrough-usbipd-win)
9. [파이프라인 실행](#9-파이프라인-실행)
10. [트러블슈팅](#10-트러블슈팅)
11. [최종 환경 요약](#11-최종-환경-요약)

---

## 1. 사전 조건

| 항목    | 버전/사양                                      |
| ------- | ---------------------------------------------- |
| Windows | 10/11, 빌드 19041 이상                         |
| 카메라  | Intel RealSense D455f (USB 3.0 이상 포트 필수) |
| RAM     | 8GB+ 권장 (WSL2에 4GB 이상 할당)               |
| 디스크  | WSL2에 ~10GB 여유 공간                         |

### 왜 WSL2인가? (Windows 네이티브 빌드 포기 이유)

- CMake 3.31의 libarchive regression 버그로 vcpkg tar 추출 실패 (`Invalid empty pathname`)
- Windows MAX_PATH 제한으로 qtdeclarative 등 긴 경로 패키지 빌드 불가
- WSL2 Ubuntu에서는 apt로 의존성 설치가 간단하고 안정적

---

## 2. WSL2 설치 및 설정

### 2-1. Windows 기능 활성화 (관리자 PowerShell)

```powershell
# WSL 및 가상화 기능 활성화
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# ⚠️ 재부팅 필요
Restart-Computer
```

### 2-2. Ubuntu 22.04 설치 (재부팅 후)

```powershell
wsl --install Ubuntu-22.04
```

설치 중 UNIX 사용자명/비밀번호 설정 프롬프트가 나타남:

- 사용자명: `dev`
- 비밀번호: `1234` (개발용)

### 2-3. NOPASSWD sudo 설정

```powershell
# Windows에서 WSL 쉘 진입 후 설정
wsl -d Ubuntu-22.04 -u root -- bash -c "cat > /etc/wsl.conf << 'EOF'
[user]
default=dev
EOF
usermod -aG sudo dev
echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev
chmod 440 /etc/sudoers.d/dev"

# WSL 재시작으로 wsl.conf 적용
wsl --terminate Ubuntu-22.04
```

---

## 3. 소스코드 복사

Windows 소스(`C:\dev\`)를 WSL2 네이티브 파일시스템으로 복사한다.
`/mnt/c/` 경로는 9p 마운트라 빌드 속도가 극도로 느리므로 반드시 `~/dev/`로 복사.

```bash
# WSL2 내부에서 실행
mkdir -p ~/dev

# vcpkg 제외하고 복사 (Windows vcpkg는 불필요, 매우 크기 때문)
rsync -a --exclude='vcpkg/' --exclude='build/' /mnt/c/dev/rtabmap ~/dev/
rsync -a /mnt/c/dev/rtabmap_2d_pipeline ~/dev/
```

---

## 4. apt 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libsqlite3-dev libfmt-dev \
  libopencv-dev \
  libpcl-dev \
  libboost-all-dev \
  libeigen3-dev \
  liboctomap-dev \
  libgtsam-dev \
  libg2o-dev \
  libusb-1.0-0-dev \
  pkg-config
```

> 총 약 695개 패키지가 설치됨. 주요 버전:
>
> - OpenCV 4.5.4
> - PCL 1.12.1
> - Boost 1.74
> - CMake 3.22

---

## 5. librealsense2 소스 빌드 (RSUSB 백엔드)

### 왜 소스 빌드인가?

- Intel apt 저장소의 librealsense2는 **V4L2 (UVC) 백엔드**만 지원
- WSL2 커널(6.6.87.2-microsoft)에는 `uvcvideo` 커널 모듈이 **없음**
- 따라서 `FORCE_RSUSB_BACKEND=ON`으로 **libusb 직접 접근** 백엔드로 빌드해야 함

### 5-1. 소스 클론

```bash
cd ~
git clone --depth 1 --branch v2.57.7 https://github.com/IntelRealSense/librealsense.git
```

### 5-2. 빌드 & 설치

```bash
cd ~/librealsense
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_WITH_OPENMP=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local

make -j4
sudo make install
sudo ldconfig
```

> ⚠️ `make -j$(nproc)` 대신 **`make -j4`** 사용.
> WSL2 기본 메모리 제한에서 병렬 수가 너무 많으면 OOM kill 발생.

### 5-3. udev 규칙 복사 (참고용)

```bash
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
```

> WSL2에서는 udevd가 실행되지 않아 실제 효과는 없음.
> USB 권한은 매번 수동으로 설정해야 함 (섹션 9 참조).

### 5-4. 설치 확인

```bash
rs-enumerate-devices 2>&1 | head -5
# → "No device detected" (카메라 미연결 상태면 정상)
# → Device info가 뜨면 성공
```

---

## 6. RTAB-Map 빌드

### 6-1. CMake 구성

```bash
cd ~/dev/rtabmap
mkdir -p build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_QT=OFF \
  -DWITH_REALSENSE2=ON \
  -DBUILD_APP=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_EXAMPLES=OFF
```

CMake 출력에서 확인할 핵심 항목:

```
--   With RealSense2 2.57.7    = YES
--   With OctoMap               = YES
```

### 6-2. 빌드 & 설치

```bash
make -j4
sudo make install
sudo ldconfig
```

### 6-3. 설치 확인

```bash
ls /usr/local/lib/librtabmap_core.so
# → 파일 존재 확인

pkg-config --modversion rtabmap
# → 0.23.4
```

---

## 7. 커스텀 파이프라인 빌드

```bash
cd ~/dev/rtabmap_2d_pipeline
mkdir -p build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 링크 확인

```bash
ldd ./rtabmap_2d_pipeline | grep -E "rtabmap|realsense|opencv"
```

기대 출력:

```
librealsense2.so.2.57    => /usr/local/lib/librealsense2.so.2.57
librtabmap_core.so.0.23  => /usr/local/lib/librtabmap_core.so.0.23
librtabmap_utilite.so.0.23 => /usr/local/lib/librtabmap_utilite.so.0.23
libopencv_core.so.4.5    => /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5
```

> `/usr/local/lib`의 librealsense (RSUSB 백엔드)에 링크되어야 함.
> `/usr/lib/`의 apt 버전(V4L2)에 링크되면 카메라 인식 안 됨.

---

## 8. RealSense USB Passthrough (usbipd-win)

WSL2는 USB 장치에 직접 접근 불가. `usbipd-win`으로 Windows USB를 WSL2에 전달해야 함.

### 8-1. Windows 측 설치

```powershell
# 관리자 PowerShell
winget install --exact dorssel.usbipd-win
```

> 설치 버전: 5.3.0

### 8-2. WSL2 측 설치

```bash
sudo apt-get install -y linux-tools-generic hwdata
```

> WSL2 커널(6.6.x)과 linux-tools(5.15.x) 버전 불일치 경고가 나오지만,
> usbipd-win이 `vhci_hcd` 모듈을 자동 로드하므로 실제 동작에 문제없음.

### 8-3. RealSense 연결 및 확인

1. RealSense D455f를 **PC 본체 USB 3.0 포트에 직접 연결** (허브 사용 금지)
2. Windows에서 장치 확인:

```powershell
usbipd list
```

```
BUSID  VID:PID    DEVICE                                                        STATE
5-3    8086:0b5c  Intel(R) RealSense(TM) Depth Camera 455f Depth, Intel(R) ...  Not shared
```

> ⚠️ **RealSense가 목록에 안 나타나는 경우**:
>
> - 케이블을 뺐다가 다시 꽂기 (접촉 불량)
> - 다른 USB 3.0 포트 시도
> - `pnputil /enum-devices /connected /class "USB"`로 연결 상태 확인
> - "연결 끊김" 상태면 물리적으로 재연결 필요

### 8-4. WSL2에 USB 장치 전달

```powershell
# (1) WSL2가 실행 중이어야 함 — 별도 터미널에서 WSL 세션 열어두기
wsl -d Ubuntu-22.04

# (2) 최초 1회: 장치 공유 등록 (관리자 권한 필요)
usbipd bind --busid 5-3

# (3) WSL2에 연결 (관리자 권한 필요)
usbipd attach --wsl --busid 5-3
```

정상 출력:

```
usbipd: info: Using WSL distribution 'Ubuntu-22.04' to attach; the device will be available in all WSL 2 distributions.
usbipd: info: Loading vhci_hcd module.
usbipd: info: Detected networking mode 'nat'.
usbipd: info: Using IP address 172.19.0.1 to reach the host.
```

### 8-5. WSL2에서 장치 확인

```bash
lsusb
# Bus 002 Device 002: ID 8086:0b5c Intel Corp. Intel(R) RealSense(TM) Depth Camera 455f
```

---

## 9. 파이프라인 실행

### 매번 실행 시 순서

```bash
# ① USB 장치 권한 설정 (WSL2에서 udev 미작동, 매 attach 마다 필요)
sudo chmod 666 /dev/bus/usb/002/*

# ② output 디렉토리 생성
mkdir -p ~/dev/rtabmap_2d_pipeline/build/output

# ③ 파이프라인 실행
cd ~/dev/rtabmap_2d_pipeline/build
./rtabmap_2d_pipeline
```

### 정상 동작 시 출력 예시

```
[INFO] CameraRealSense2.cpp:528::init() Device "Intel RealSense D455F" with serial number 254122300022
[INFO] CameraRealSense2.cpp:624::init() Using device with Serial No: 254122300022
[INFO] CameraRealSense2.cpp:630::init() Device FW version: 5.15.1.55
[INFO] CameraRealSense2.cpp:715::init() Stereo Module was found.
[INFO] CameraRealSense2.cpp:715::init() RGB Camera was found.
[INFO] CameraRealSense2.cpp:715::init() Motion Module was found.
=== RTAB-Map 2D Pipeline running ===
  Camera : RealSense D455f
  TCP    : 127.0.0.1:7777
  Output : output/map.pgm + map.yaml
  Press Ctrl+C to stop.

[INFO] OdometryF2M.cpp:1569::computeTransform() Odom update time = 0.032s lost=false features=709 inliers=150/362
```

### 알려진 경고 (무시 가능)

```
[ERROR] (context.cpp:41) No valid configuration file found at : /home/dev/.realsense-config.json loading defaults
```

→ RealSense 설정 파일 없음. 기본값 사용, 정상 동작.

```
[WARN] Odometry.cpp:346::process() Received IMU doesn't have orientation set! It is ignored.
```

→ IMU orientation fusion이 안 되어 무시됨. Visual Odometry만으로 정상 추적.

### 종료

`Ctrl+C` → 정상 shutdown, `rtabmap.db` 저장됨.

---

## 10. 트러블슈팅

### usbipd list에 RealSense가 안 보임

| 원인                | 해결                              |
| ------------------- | --------------------------------- |
| 케이블 접촉 불량    | USB 케이블을 뽑고 다시 꽂기       |
| USB 2.0 포트 연결   | USB 3.0 (파란색) 포트에 직접 연결 |
| USB 허브 사용       | 허브 없이 PC 본체에 직접 연결     |
| pnputil "연결 끊김" | 물리적 재연결 필요                |

### RS2_USB_STATUS_ACCESS 에러

```
failed to open usb interface: 0, error: RS2_USB_STATUS_ACCESS
```

→ USB 권한 문제. `sudo chmod 666 /dev/bus/usb/002/*` 실행 후 재시도.

### make 중 OOM (Out of Memory)

```
c++: fatal error: Killed signal terminated program cc1plus
```

→ 병렬 수 줄이기: `make -j4` 또는 `make -j2` 사용.
→ WSL2 메모리 늘리기: `%UserProfile%\.wslconfig` 에서 `memory=8GB` 설정.

```ini
# %UserProfile%\.wslconfig
[wsl2]
memory=8GB
swap=4GB
```

### librealsense가 카메라를 감지 못함

```
No device detected. Is it plugged in?
```

1. `lsusb`로 USB 장치 목록 확인 — `8086:0b5c` 있어야 함
2. 없으면: Windows에서 `usbipd attach --wsl --busid 5-3` 재실행
3. 있는데 안 됨: `sudo chmod 666 /dev/bus/usb/002/*` 후 재시도
4. 링크 확인: `ldd ./rtabmap_2d_pipeline | grep realsense` → `/usr/local/lib/` (RSUSB) 인지 확인

### apt의 librealsense2와 혼동

시스템에 두 버전이 공존할 수 있음:

- `/usr/lib/x86_64-linux-gnu/librealsense2.so` — apt 설치 (V4L2 백엔드, WSL2에서 **동작 안함**)
- `/usr/local/lib/librealsense2.so` — 소스 빌드 (RSUSB 백엔드, **이것을 사용해야 함**)

바이너리가 올바른 버전에 링크되는지 항상 확인:

```bash
ldd ./rtabmap_2d_pipeline | grep realsense
# → /usr/local/lib/librealsense2.so.2.57 이어야 함
```

---

## 11. 최종 환경 요약

### 소프트웨어 버전

| 구성요소      | 버전                             | 설치 방법              |
| ------------- | -------------------------------- | ---------------------- |
| Ubuntu        | 22.04 LTS (Jammy)                | WSL2                   |
| WSL2 커널     | 6.6.87.2-microsoft-standard-WSL2 | 자동                   |
| CMake         | 3.22                             | apt                    |
| GCC           | 11.x                             | apt (build-essential)  |
| OpenCV        | 4.5.4                            | apt (libopencv-dev)    |
| PCL           | 1.12.1                           | apt (libpcl-dev)       |
| Boost         | 1.74                             | apt (libboost-all-dev) |
| librealsense2 | 2.57.7                           | **소스 빌드** (RSUSB)  |
| RTAB-Map      | 0.23.4                           | 소스 빌드              |
| usbipd-win    | 5.3.0                            | winget (Windows)       |

### 설치 경로

| 항목                | 경로                                                  |
| ------------------- | ----------------------------------------------------- |
| RTAB-Map 소스       | `~/dev/rtabmap/`                                      |
| RTAB-Map 설치       | `/usr/local/lib/`, `/usr/local/include/`              |
| librealsense 소스   | `~/librealsense/`                                     |
| librealsense 설치   | `/usr/local/lib/`, `/usr/local/bin/`                  |
| 파이프라인 소스     | `~/dev/rtabmap_2d_pipeline/`                          |
| 파이프라인 바이너리 | `~/dev/rtabmap_2d_pipeline/build/rtabmap_2d_pipeline` |
| SLAM 데이터베이스   | `~/dev/rtabmap_2d_pipeline/build/rtabmap.db`          |
| 맵 출력             | `~/dev/rtabmap_2d_pipeline/build/output/`             |

### RealSense D455F 정보

| 항목         | 값           |
| ------------ | ------------ |
| 시리얼 넘버  | 254122300022 |
| 펌웨어       | 5.15.1.55    |
| USB 타입     | 3.2          |
| Product ID   | 0x0B5C       |
| IMU          | BMI085       |
| usbipd BUSID | 5-3          |

### 빠른 실행 체크리스트

```
□ WSL2 터미널 열기
□ (Windows 관리자 PowerShell) usbipd attach --wsl --busid 5-3
□ (WSL2) sudo chmod 666 /dev/bus/usb/002/*
□ (WSL2) cd ~/dev/rtabmap_2d_pipeline/build && ./rtabmap_2d_pipeline
□ 카메라를 이동하며 맵 생성 확인
□ Ctrl+C로 종료
```
