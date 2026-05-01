# Semantic SLAM 계획 — Phase-1: 주차공간 라인 → 벽 태깅

> 본 문서는 지하주차장 Semantic SLAM의 첫 번째 구현 단계(Phase-1) 계획서다.
> 내부 플랜 파일: `C:\Users\JungiHong\.claude\plans\dev-2d-polymorphic-church.md`

---

## 1. Context

현재 2D 점유 격자맵(`rtabmap_pipeline` — RealSense live + Unity TCP 통합)은 세 가지 상태만 구분한다.

| 값  | 의미                   |
| --- | ---------------------- |
| −1  | 미탐색 (unknown)       |
| 0   | 빈 공간 (free)         |
| 100 | 장애물 (wall/obstacle) |

그러나 지하주차장은 _"물리적으로는 바닥이지만 주행해서는 안 되는 영역"_ 이 다수 존재한다 — 대표적으로 **주차공간(주차면)**, 일방통행 구역 등. 본 계획의 Phase-1은 이 중 **주차공간 라인**만 다룬다.

**다운스트림 요구**: 별도 프로그램이 `(map, pose)` 한 쌍을 받아 **탈출 경로**를 생성한다. 우리가 publish하는 매 패킷이 *동일한 keyframe pose 그래프 스냅샷에서 파생된* 일관된 (map, pose) 여야 한다.

**목표**: RGB 영상에서 주차 라인을 감지 → 해당 keyframe의 image-space에 마스크로 anchoring → 글로벌 맵 빌드 시 그때의 corrected pose로 back-project → 격자 셀을 **obstacle(100)** 으로 강제 태깅. 향후 다른 태그(출구, 일방통행 등)는 동일한 파이프라인에 클래스만 확장.

**확정된 설계 결정**:

| 항목             | 선택                                                                        | 근거                                                                               |
| ---------------- | --------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| 추론 방식        | 로컬 VLM — **GroundingDINO**                                                | 오픈 보캐뷸러리 텍스트 프롬프트로 태그 확장 용이. 오프라인 실행 가능.              |
| 처리 주기        | **비동기 저주파 1–2 Hz**                                                    | SLAM 30 Hz 그대로 유지. Worker 스레드가 keyframe 단위로 샘플링.                    |
| **Anchoring**    | **keyframe nodeId 단위 image-space 마스크** (per-keyframe mask)             | 루프 클로저 재최적화 시 keyframe pose만 갱신되면 자동으로 따라감. World-coord sparse map의 재anchor 문제 회피. ConceptGraphs 표준 패턴. |
| 초기 검증 플랫폼 | **Unity 가상 데이터** (`rtabmap_pipeline --source unity`)                   | 단일 스레드 SLAM 루프, 조건 제어 가능, 물리 D455f 없이 개발.                       |

**선행 연구**: SemanticFusion (ICRA 2017), ConceptFusion (RSS 2023), ConceptGraphs (ICRA 2024), SEMANTIC-RTAB-MAP (2019). 모두 비동기 추론을 keyframe/노드 anchoring으로 처리. 우리 구조는 ConceptGraphs와 가장 가깝다.

---

## 2. Architecture

```
[Unity TCP RGB+Depth+IMU] → CameraUnityTCP → SensorCaptureThread
                                  │
                                  ▼
                    Odometry → rtabmap.process() → Statistics
                                  │
                                  ▼
                  GridPublisher::processStatistics
                    │
                    ├─ 새 keyframe 감지 (lastId ∉ grid_.addedNodes())
                    │     ├─ localGrids_.add(...)                  (기존)
                    │     └─ semanticWorker_->submit({nodeId, rgb})  ★ 신규
                    │
                    ├─ grid_.update(stats.poses()) → map8S          (기존)
                    │
                    ├─ semanticMasks_.applyTo(map8S, stats.poses(),
                    │                          cameraModels, xMin, yMin, cellSize)  ★ 신규
                    │     (각 keyframe의 mask를 corrected pose × localTransform 으로 raster)
                    │
                    └─ visualise / sendTcp(map8S, poseX, poseY, poseYaw)  (기존)
                              │
                              ▼ TCP :7777
                       Escape route generator / Unity client

[별도 스레드] SemanticWorker
   ├─ latest-only slot에서 (nodeId, rgb) pop (1–2 Hz)
   ├─ JPEG 인코드 → HTTP POST → 127.0.0.1:7788/detect
   ├─ 검출 박스 → image-space 마스크 (cv::Mat 1채널)
   └─ semanticMasks_[nodeId] = mask  (mutex 보호)

[별도 프로세스] semantic_service (Python, FastAPI + GroundingDINO-T)
```

**설계 원칙**:

1. **Anchoring = keyframe nodeId**. 마스크는 그 keyframe의 image-space에 머무르고, 월드 좌표는 publish 시점에 *현재 corrected pose*로 매번 재계산.
   → 루프 클로저 재최적화 시 keyframe pose가 갱신되면 다음 publish에서 자동으로 새 위치에 raster. 별도 처리 불필요.
2. **Python VLM은 사이드카 프로세스**. 127.0.0.1 HTTP 루프백으로 C++과 분리.
   → CUDA/PyTorch 의존성을 C++ 빌드에 끌어들이지 않음. 크래시 도메인 분리.
3. **SLAM 메인 스레드 비차단**. `submit()`은 latest-only slot에 swap만 수행 (이전 프레임 drop).
4. **`(map, pose)` 일관성**: Unity 경로는 단일 스레드라 `processStatistics` 안에서 map raster와 pose 추출이 순차 실행되므로 자동. Live 경로는 1프레임 odom-correction lag가 있을 수 있으나 keyframe 간격(5cm) 기준 무시 가능.

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

### 3.2. C++ 모듈 — `c:\dev\rtabmap_pipeline\` 내부

> 단일 소비자(GridPublisher)뿐이므로 별도 `rtabmap_shared/` 디렉토리 만들지 않음. 두 번째 소비자가 생기면 그때 분리.

| 파일                          | 내용                                                                                                   |
| ----------------------------- | ------------------------------------------------------------------------------------------------------ |
| `SemanticWorker.h`            | `std::thread` + latest-only slot + cpp-httplib. `submit({nodeId, rgb})` / 결과 콜백으로 mask 저장      |
| `SemanticMaskStore.h`         | `unordered_map<int nodeId, cv::Mat mask>` + `applyTo(map8S, poses, cameraModels, xMin, yMin, cellSize)` |
| `SemanticBackproject.h`       | 순수 함수: `backprojectPixel(...)`, `rayPlaneIntersect(...)`                                            |

**HTTP 클라이언트**: **cpp-httplib** (vcpkg `cpp-httplib`).
이유: single-header, libcurl/openssl 의존성 회피, 기존 winsock2 스타일과 호환.

### 3.3. GridPublisher 통합 (최소 수정)

기존 `GridPublisher.h` 의 두 hook 지점에 한 줄씩 추가:

**(a) 새 keyframe 감지 직후** — 기존 `localGrids_.add(...)` 다음에:

```cpp
if (semanticWorker_) {
    semanticWorker_->submit({lastId, signature.sensorData().imageRaw().clone()});
}
```

**(b) 글로벌 맵 추출 직후** — 기존 `cv::Mat map8S = grid_.getMap(xMin, yMin)` 다음에:

```cpp
if (semanticWorker_) {
    semanticMasks_.applyTo(map8S, stats.poses(),
                           getCameraModelsCache(),  // keyframe별 캐시 (아래 3.4)
                           xMin, yMin, cellSize);
}
```

**추가 멤버**:

```cpp
std::unique_ptr<SemanticWorker> semanticWorker_;
SemanticMaskStore semanticMasks_;
std::unordered_map<int, CameraModel> cameraModelCache_;  // keyframe별
```

**CLI 플래그** (`main.cpp`):

- `--semantic-url http://127.0.0.1:7788/detect` (활성)
- semantic 미지정 시 = 기본값, 기존 동작 보존

**CMakeLists.txt**:

```cmake
find_package(httplib CONFIG REQUIRED)
target_link_libraries(${TARGET} PRIVATE httplib::httplib)
```

### 3.4. 카메라 모델 캐시 — 왜 필요한가

`stats.poses()`는 매 publish마다 *모든* keyframe의 corrected pose를 줘서 라이브로 반영되지만, 그 시점에 그 keyframe의 SensorData는 이미 working memory에서 빠져나갔을 수 있음. 따라서 새 keyframe 등록 시 `cameraModels()[0]`을 `cameraModelCache_[nodeId]`에 따로 저장. 마스크 raster 시 lookup.

### 3.5. Unity 테스트 씬

기존 `D455_Virtual_Camera.prefab` + Unity TCP 송신 재사용.

| 요소   | 스펙                                                                   |
| ------ | ---------------------------------------------------------------------- |
| 바닥   | 20×10 m 쿼드, 단색 회색                                                |
| 주차선 | 자식 쿼드에 흰 직사각형 2.5×0.1 m × 6~8개 (표준 주차 베이)             |
| 장애물 | 큐브 "차량" 2~3개 (라인 태깅이 실제 장애물과 독립적으로 동작함을 검증) |
| 주행   | 통로 중앙 직선 1 m/s × 30s, **루프 클로저 검증용으로 마지막 5초간 출발점 회귀** |

---

## 4. Back-projection (publish 시점에 매번 재계산)

`SemanticMaskStore::applyTo`가 매 publish마다 keyframe별로 수행:

```
for each (nodeId, mask) in semanticMasks_:
    if poses.count(nodeId) == 0:  continue   // working memory에서 빠짐
    T_node2map = poses.at(nodeId)
    T_cam2map  = T_node2map * cameraModelCache_[nodeId].localTransform()
    K          = cameraModelCache_[nodeId]   // fx, fy, cx, cy

    for each pixel (u, v) where mask(u,v) > threshold:
        // (1) 깊이 유효 시 (보존된 keyframe depth 또는 캐시):
        d = depth(u, v)
        if d ∈ [0.3, 6.0]:
            X_cam = (u − cx) * d / fx
            Y_cam = (v − cy) * d / fy
            P_map = T_cam2map * (X, Y, d)
        // (2) 깊이 무효 시 → ray-plane fallback:
        else:
            r_cam = normalize((u−cx)/fx, (v−cy)/fy, 1)
            r_map = R_cam2map * r_cam
            o_map = t_cam2map
            z_floor = node_local_grid_view_point.z   // RTAB-Map gridViewPoint
            t = (z_floor − o_map.z) / r_map.z
            if t > 0:
                P_map = o_map + t * r_map

        // (3) 위생 필터:
        if |P_map.z − z_floor| > 0.15: continue

        // (4) 격자 셀 인덱스 → obstacle 강제 태깅:
        cx_idx = round((P_map.x - xMin) / cellSize)
        cy_idx = round((P_map.y - yMin) / cellSize)
        map8S(cy_idx, cx_idx) = 100   // override
```

**중요**: 변환은 **submit 시점이 아니라 publish 시점에** 계산. 따라서 그 사이에 루프 클로저로 `poses[nodeId]`가 갱신되면 자동으로 새 위치에 raster.

**Depth 캐시**: depth도 keyframe별 캐시 필요. `cameraModelCache_`와 동일하게 새 keyframe 시 `depthCache_[nodeId] = depthOrRightRaw().clone()`. 메모리 부담 시 압축 보관.

---

## 5. Files to Modify / Create

### 수정

| 파일                              | 변경 규모                                               |
| --------------------------------- | ------------------------------------------------------- |
| `rtabmap_pipeline/GridPublisher.h` | 멤버 3개 + hook 2개 (submit, applyTo) ─ 약 10줄         |
| `rtabmap_pipeline/main.cpp`        | CLI 플래그 파싱, GridPublisher 생성자에 worker 인자     |
| `rtabmap_pipeline/CMakeLists.txt`  | `find_package(httplib)`, target_link_libraries          |

### 신규

```
c:\dev\
├── semantic_service\           (Python 사이드카)
│   ├── app.py
│   ├── gdino_runner.py
│   ├── requirements.txt
│   └── run_semantic.bat
└── rtabmap_pipeline\           (기존 디렉토리에 헤더만 추가)
    ├── SemanticWorker.h        (★ 신규)
    ├── SemanticMaskStore.h     (★ 신규)
    └── SemanticBackproject.h   (★ 신규)
```

---

## 6. Verification (End-to-End Test)

1. **환경 준비**
   - Python 3.11 venv → `pip install -r semantic_service/requirements.txt`
   - vcpkg: `.\vcpkg install cpp-httplib:x64-windows`
   - `run_semantic.bat` 실행 → `curl http://127.0.0.1:7788/healthz` 200 확인

2. **Unity 데이터**: 주차선 씬, 마지막 5초 출발점 회귀로 루프 클로저 유발

3. **Mock 단계 (Worker만 하드코딩 박스 반환)**:

   ```
   rtabmap_pipeline.exe --source unity                 # semantic 미지정 = baseline
   rtabmap_pipeline.exe --source unity --semantic-url mock://hardcoded
   ```

   → 하드코딩 박스가 PGM에 검정 셀로 찍히는지, **루프 클로저 시 태그가 keyframe pose 갱신을 따라가는지** 확인.

4. **Live VLM 단계**:

   ```
   rtabmap_pipeline.exe --source unity --semantic-url http://127.0.0.1:7788/detect
   ```

5. **검증 기준**
   - 주차선 위치가 실제 차량 장애물과 **동일한 검정(100)** 으로 태깅 (육안)
   - `baseline` ↔ `semantic` PGM diff: **추가 태깅 영역이 주차선 부근에만**, 통로 false positive 없음
   - 루프 클로저 발생 시점 전후 PGM 캡처: **태그가 keyframe pose 보정과 함께 같은 변환량으로 이동**
   - (Optional) OpenCV 뷰에 semantic overlay 빨강으로 시각화

6. **TCP 연동**: Unity 클라이언트 / 탈출 경로 생성기는 **프로토콜 변경 없이** 기존 32B 헤더 packet 그대로 수신 (pose 이미 포함). 주차면이 벽으로 들어옴 확인.

---

## 7. Implementation Phases

> 위험 구간(back-projection 좌표 변환 + raster override)을 Python·VLM 의존성 없이 먼저 검증.

| #   | 단계                                                                       | 산출물                                          |
| --- | -------------------------------------------------------------------------- | ----------------------------------------------- |
| 1   | C++ SemanticMaskStore + SemanticBackproject (헤더 only, mock submit 경로) | 단위 테스트 가능한 헤더                         |
| 2   | GridPublisher 통합 + Worker mock 모드 (HTTP 없이 하드코딩 박스 반환)      | `--semantic-url mock://...` 동작                |
| 3   | Unity 주차선 씬 제작 + mock 검증 (루프 클로저 포함)                       | PGM diff 스크린샷, loop closure 시퀀스          |
| 4   | Python semantic_service 스켈레톤 (FastAPI, mock detection 응답)           | `app.py`, healthz/detect 동작                   |
| 5   | GroundingDINO 통합 + Worker를 cpp-httplib HTTP 호출로 전환                | 실제 VLM 추론 동작                              |
| 6   | End-to-end 검증                                                            | baseline vs semantic PGM diff                   |
| 7   | 문서화                                                                     | `plan.md` Step 5, 본 문서 최종 갱신            |

---

## 8. Risks & Out-of-Scope (Phase-1)

| 위험                                                  | Phase-1 대응                                                                            | Phase-2 이후                                                                     |
| ----------------------------------------------------- | --------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| **루프 클로저 재최적화**                              | **자동 해결됨** ─ keyframe-anchored mask가 매 publish마다 corrected pose로 raster        | 해당 없음                                                                        |
| **실제 지하주차장 라인 열화** (마모·유분·저조도)      | 범위 외 (Unity 이상적 환경)                                                             | 프롬프트 엔지니어링 + confidence 누적 + 실측 데이터 수집                         |
| **태그 확장** (출구, 일방통행 등)                     | 단일 클래스 마스크 (uint8)                                                              | 마스크를 `{class: enum, conf: uint8}` 다채널로 확장, TCP에 semantic 레이어 추가  |
| **VRAM 요구**                                         | GroundingDINO-T Swin-T ~1GB (6GB+ GPU 권장). CPU fallback은 수 초/프레임                | ONNX 변환 + TensorRT/DirectML 가속 검토                                          |
| **추론 지연**                                         | 1 Hz @ 800px 기준 300–700 ms/keyframe. SLAM 무영향, 마스크 도착이 keyframe 1–2개 늦음   | 정식 요구사항 정립 후 판단                                                       |
| **Working memory에서 빠진 keyframe**                  | `cameraModelCache_` / `depthCache_`로 별도 보관. 마스크 raster 시 둘 다 있는 키프레임만 처리 | RTAB-Map의 long-term memory 콜백과 연동해 캐시도 함께 관리                       |
| **(map, pose) 일관성** (live RealSense 한정)          | 1프레임 odom-correction lag (5cm 단위) — 실측 시 무시 가능                              | 필요 시 publish 시점에 `currentPose_ = mapCorrection × rawOdom`로 즉시 재계산    |

---

## 9. Critical Files (구현 시 첫 진입점)

- [rtabmap_pipeline/GridPublisher.h](rtabmap_pipeline/GridPublisher.h) — Phase-1 주 통합 지점 (line 187 keyframe hook, line 211 publish hook)
- [rtabmap_pipeline/main.cpp](rtabmap_pipeline/main.cpp) — CLI 진입점, `runUnity` (line 169)
- [plan.md](plan.md) — Step 5 섹션 추가 필요
