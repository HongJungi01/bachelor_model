# Semantic SLAM 계획 — Phase-1: 주차공간 라인 → 벽 태깅

> 본 문서는 지하주차장 Semantic SLAM의 첫 번째 구현 단계(Phase-1) 계획서다.
> 내부 플랜 파일: `C:\Users\JungiHong\.claude\plans\dev-2d-polymorphic-church.md`

---

## 1. Context

현재 2D 점유 격자맵(`rtabmap_pipeline` — live + offline 통합)은 세 가지 상태만 구분한다.

| 값  | 의미                   |
| --- | ---------------------- |
| −1  | 미탐색 (unknown)       |
| 0   | 빈 공간 (free)         |
| 100 | 장애물 (wall/obstacle) |

그러나 지하주차장은 _"물리적으로는 바닥이지만 주행해서는 안 되는 영역"_ 이 다수 존재한다 — 대표적으로 **주차공간(주차면)**, 일방통행 구역 등. 본 계획의 Phase-1은 이 중 **주차공간 라인**만 다룬다.

**목표**: RGB 영상에서 주차 라인을 감지 → 지면으로 back-project → 해당 격자 셀을 **obstacle(100)** 으로 강제 태깅. 향후 다른 태그(출구, 일방통행 등)는 동일한 파이프라인에 클래스만 확장.

**확정된 설계 결정**:

| 항목             | 선택                                           | 근거                                                                              |
| ---------------- | ---------------------------------------------- | --------------------------------------------------------------------------------- |
| 추론 방식        | 로컬 VLM — **GroundingDINO**                   | 오픈 보캐뷸러리 텍스트 프롬프트로 주차선 외 태그 확장이 용이. 오프라인 실행 가능. |
| 처리 주기        | **실시간 저주파 병렬 1–2 Hz**                  | SLAM 30 Hz 그대로 유지, 별도 워커 스레드가 프레임 샘플링                          |
| 초기 검증 플랫폼 | **Unity 가상 데이터** (`rtabmap_pipeline --source images`) | Step 4 오프라인 파이프라인 재사용, 조건 제어 가능, 물리 D455f 없이 개발 |

---

## 2. Architecture

```
[Unity RGBD dataset] → rtabmap_pipeline/main.cpp (--source images)
                           │
                           ▼
                  RTAB-Map SLAM (30 Hz, 기존)
                           │ RtabmapEvent
                           ▼
                  GridPublisher::processStatistics
                           ├─→ grid_.update / getMap     (기존)
                           ├─→ semanticCells_.applyTo(map8S)   ★ 신규: sparse mask 병합
                           ├─→ worker_->submit(frame)          ★ 신규: 비동기 제출
                           └─→ visualise / sendTcp       (기존)
                                  │
                                  ▼ TCP :7777
                              Unity client

[별도 스레드] SemanticWorker
   ├─ latest-only slot에서 프레임 pop (1–2 Hz)
   ├─ JPEG 인코드 → HTTP POST → 127.0.0.1:7788/detect
   ├─ 검출 박스 → 카메라/지면 back-projection (depth + ray-plane fallback)
   └─ world 좌표 → semanticCells_[key] = conf  (mutex 보호)

[별도 프로세스] semantic_service (Python, FastAPI + GroundingDINO-T)
```

**설계 원칙**:

1. **Python VLM은 사이드카 프로세스**. 127.0.0.1 HTTP 루프백으로 C++과 분리.
   → CUDA/PyTorch 의존성을 C++ 빌드에 끌어들이지 않음. 크래시 도메인 분리.
2. **SLAM 메인 스레드는 절대 블록되지 않음**. `submit()`은 latest-only slot에 swap만 수행 (이전 프레임 drop).
3. **Semantic 마스크는 world-좌표 sparse map** (`unordered_map<uint64_t, uint8_t>`).
   → 격자가 확장되어도 재색인 불필요. 매 프레임 publish 직전 raster 적용 → RTAB-Map이 셀을 free로 갱신해도 다음 프레임에서 다시 100으로 덮어씀 (**override 보장**).

---

## 3. Components

### 3.1. Python semantic service — `c:\dev\semantic_service\`

| 파일               | 역할                                                              |
| ------------------ | ----------------------------------------------------------------- |
| `app.py`           | FastAPI, `POST /detect`, `GET /healthz`                           |
| `gdino_runner.py`  | GroundingDINO 모델 로드 및 detect 래핑                            |
| `requirements.txt` | torch(+cu), transformers, fastapi, uvicorn, pillow, opencv-python |
| `run_semantic.bat` | venv 활성화 + uvicorn 기동                                        |

**API**

요청 `POST http://127.0.0.1:7788/detect`

```json
{
  "image_jpeg_b64": "...",
  "prompt": "parking space line . lane marking",
  "box_threshold": 0.3,
  "text_threshold": 0.25,
  "request_id": 123
}
```

응답

```json
{
  "request_id": 123,
  "width": 1280, "height": 720,
  "inference_ms": 412,
  "detections": [
    { "box": [x1, y1, x2, y2], "score": 0.71, "label": "parking space line" }
  ],
  "mask_png_b64": null
}
```

초기 프롬프트: `"parking space line . lane marking"` (GroundingDINO는 `.` 으로 다중 쿼리 구분).
JPEG q=85, 긴 변 800px 리사이즈 (지연 단축).
`mask_png_b64`는 추후 SAM-2 업그레이드를 위해 예약, Phase-1에서는 `null`.

### 3.2. C++ 공유 모듈 — `c:\dev\rtabmap_shared\` (신규)

| 파일                    | 내용                                                                                                      |
| ----------------------- | --------------------------------------------------------------------------------------------------------- |
| `SemanticWorker.h`      | `std::thread` + latest-only slot + cpp-httplib. `submit(SemanticFrame)` 인터페이스                        |
| `SemanticMask.h`        | `SemanticCellStore`: sparse `unordered_map<uint64_t, uint8_t>`. `applyTo(cv::Mat&, xMin, yMin, cellSize)` |
| `SemanticBackproject.h` | 순수 함수: `backprojectPixel(...)`, `rayPlaneIntersect(...)`                                              |

**HTTP 클라이언트 선택**: **cpp-httplib** (vcpkg `cpp-httplib`).
이유: single-header, Qt 이벤트 루프 불필요, libcurl/openssl 추가 의존성 회피, 기존 winsock2 스타일과 호환.

### 3.3. 기존 파이프라인 통합 (최소 수정)

`GridPublisher.h` (양쪽 파이프라인 동일):

**추가 멤버**:

```cpp
std::unique_ptr<SemanticWorker> worker_;
SemanticCellStore semanticCells_;
std::mutex semanticMtx_;
```

**`processStatistics()` 수정** — `grid_.getMap(xMin, yMin, cellSize, map8S)` 직후에 다음 추가:

```cpp
semanticCells_.applyTo(map8S, xMin, yMin, cellSize);   // 1. world → cell raster

if (worker_) {                                         // 2. 비동기 제출 (non-blocking)
    worker_->submit({rgb, depth,
                     currentPose * cameraModels[0].localTransform(),
                     K, stamp, xMin, yMin, cellSize});
}
// 이후 기존 visualise / sendTcp 호출
```

**CLI 플래그** (`main.cpp`):

- `--semantic-url http://127.0.0.1:7788/detect` (활성)
- `--no-semantic` (기본값 — 기존 동작 보존)

**CMakeLists.txt**:

```cmake
find_package(httplib CONFIG REQUIRED)
target_include_directories(${TARGET} PRIVATE ${CMAKE_SOURCE_DIR}/../rtabmap_shared)
target_link_libraries(${TARGET} PRIVATE httplib::httplib)
```

### 3.4. Unity 테스트 씬

기존 `D455_Virtual_Camera.prefab` + `RGBD_DataCollector.cs` 재사용.

| 요소   | 스펙                                                                   |
| ------ | ---------------------------------------------------------------------- |
| 바닥   | 20×10 m 쿼드, 단색 회색                                                |
| 주차선 | 자식 쿼드에 흰 직사각형 2.5×0.1 m × 6~8개 (표준 주차 베이)             |
| 장애물 | 큐브 "차량" 2~3개 (라인 태깅이 실제 장애물과 독립적으로 동작함을 검증) |
| 주행   | `AnimationCurve`, 통로 중앙 직선 1 m/s × 30s                           |

---

## 4. Back-projection (핵심 수식)

검출 박스 내부 픽셀 `(u, v)`에 대해:

**1) 깊이 유효** (`d = D(u,v)` ∈ [0.3, 6.0]):

```
X_cam = (u − cx) · d / fx
Y_cam = (v − cy) · d / fy
Z_cam = d
P_map = T_cam2map · (X, Y, Z)
```

**2) 깊이 무효** (주차 도장면 IR 반사로 홀 발생 흔함) → **ray-plane fallback**:

```
r_cam = normalize((u−cx)/fx, (v−cy)/fy, 1)
r_map = R_cam2map · r_cam
o_map = t_cam2map
z_floor = sensorData().gridViewPoint().z        // RTAB-Map이 사용하는 지면 높이
t = (z_floor − o_map.z) / r_map.z
if t > 0:
    P_map = o_map + t · r_map
```

**3) 위생 필터**: `|P_map.z − z_floor| > 0.15 m` 면 기각 (잘못된 검출/노이즈)

**4) 격자 키 & confidence EMA**:

```
key = pack(round(P_map.x / cellSize), round(P_map.y / cellSize))
semanticCells_[key] = min(255, semanticCells_[key] + score · 255)
```

→ 단일 프레임 누락 시 소거되지 않도록 누적.

**카메라→맵 변환**: `T_cam2map = currentPose * cameraModels()[0].localTransform()`
(`processStatistics` 내에서 프레임 제출 시점에 snapshot)

---

## 5. Files to Modify / Create

### 수정 (최소 변경, 기존 파일 재사용)

| 파일                              | 변경 규모                                              |
| --------------------------------- | ------------------------------------------------------ |
| `rtabmap_pipeline/GridPublisher.h` | 멤버 3개 + `processStatistics`에 약 5줄 (live + offline 양쪽 자동 적용) |
| `rtabmap_pipeline/main.cpp`        | CLI 플래그 파싱, `GridPublisher` 생성자 인자           |
| `rtabmap_pipeline/CMakeLists.txt`  | httplib 링크, shared include                           |
| `plan.md`                          | Step 5 섹션 추가                                       |

> 2026-04-25 통합 후: live(RealSense)와 offline(Unity images) 코드 경로가 동일한 `GridPublisher`를 공유하므로 별도 미러링 단계가 사라졌다 (구 Phase 7 폐기).

### 신규

```
c:\dev\
├── semantic_service\           (Python 사이드카)
│   ├── app.py
│   ├── gdino_runner.py
│   ├── requirements.txt
│   └── run_semantic.bat
├── rtabmap_shared\             (C++ 공유 모듈)
│   ├── SemanticWorker.h
│   ├── SemanticMask.h
│   └── SemanticBackproject.h
└── semanticplan.md             (본 문서)
```

---

## 6. Verification (End-to-End Test)

1. **환경 준비**
   - Python 3.11 venv → `pip install -r semantic_service/requirements.txt`
   - vcpkg: `.\vcpkg install cpp-httplib:x64-windows`
   - `run_semantic.bat` 실행 → `curl http://127.0.0.1:7788/healthz` 200 확인

2. **Unity 데이터 캡처**: 주차선 씬 → `c:\dev\datasets\unity_parking_lines\`

3. **Baseline (semantic OFF)**:

   ```
   rtabmap_pipeline.exe --source images datasets\unity_parking_lines --no-semantic
   ```

   → `map_baseline.pgm` 저장

4. **Semantic ON**:

   ```
   rtabmap_pipeline.exe --source images datasets\unity_parking_lines ^
                        --semantic-url http://127.0.0.1:7788/detect
   ```

   → `map_semantic.pgm` 저장

5. **검증 기준**
   - `map_semantic.pgm`에서 주차선 위치가 실제 차량 장애물과 **동일한 검정(100)** 으로 태깅됨 (육안 확인)
   - `baseline` ↔ `semantic` diff 시 **추가 태깅 영역이 주차선 부근에만** 생성, 통로에는 false positive 없음
   - (Optional) OpenCV 뷰에 semantic overlay를 빨강으로 시각화 추가하여 디버깅

6. **Unity TCP 연동**: Unity 클라이언트는 **프로토콜 변경 없이** 기존 packet 그대로 수신. 주차면이 벽으로 들어옴을 확인.

---

## 7. Implementation Phases

| #   | 단계                                      | 산출물                                     |
| --- | ----------------------------------------- | ------------------------------------------ |
| 1   | Python 서비스 스켈레톤 (모의 응답)        | `app.py`, `requirements.txt`, healthz 응답 |
| 2   | GroundingDINO 통합                        | `gdino_runner.py`, 단일 이미지 CLI 테스트  |
| 3   | C++ SemanticWorker + 공유 모듈            | `rtabmap_shared/` 3개 헤더                 |
| 4   | Back-projection + SemanticMask 단위 검증  | (선택) `test_backproject.exe`              |
| 5   | `rtabmap_pipeline` GridPublisher 통합     | CLI 플래그, 통합 빌드 (live + offline 양쪽 자동) |
| 6   | Unity 씬 제작 + end-to-end 검증           | PGM diff 스크린샷                          |
| 7   | 문서화                                    | `plan.md` Step 5, 본 문서 업데이트         |

---

## 8. Risks & Out-of-Scope (Phase-1)

| 위험                                                                                                           | Phase-1 대응                                                                            | Phase-2 이후                                                                     |
| -------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| **루프 클로저 재최적화**: 그래프 재최적화 시 `semanticCells_`의 world 좌표가 pre-correction pose 기반이라 오차 | 무시 (Unity 환경에서는 loop closure 최소)                                               | `node_id` 기반 anchor 저장, 포즈 갱신 시 변환 재계산                             |
| **실제 지하주차장 라인 열화** (마모·유분·저조도)                                                               | 범위 외 (Unity 이상적 환경에서 기능 검증까지)                                           | 프롬프트 엔지니어링 + confidence EMA 튜닝 + 실측 데이터 수집                     |
| **태그 확장** (출구, 일방통행 등)                                                                              | `semanticCells_` uint8 confidence 단일 클래스                                           | value를 `{class: enum, conf: uint8}`로 확장, TCP 프로토콜에 semantic 레이어 추가 |
| **VRAM 요구**                                                                                                  | GroundingDINO-T Swin-T ~1GB (6GB+ GPU 권장). CPU fallback은 오프라인 전용(수 초/프레임) | ONNX 변환 + TensorRT/DirectML 가속 검토                                          |
| **지연**                                                                                                       | 1 Hz @ 800px 기준 300–700 ms/프레임. SLAM 무영향, 셀 반영이 0.5–1 프레임 늦음           | 정식 요구사항 정립 후 판단                                                       |

---

## 9. Critical Files (구현 시 첫 진입점)

- [rtabmap_pipeline/GridPublisher.h](rtabmap_pipeline/GridPublisher.h) — Phase-1 주 통합 지점 (live + offline 공용)
- [rtabmap_pipeline/main.cpp](rtabmap_pipeline/main.cpp) — CLI 진입점
- [plan.md](plan.md) — Step 5 섹션 추가 필요
