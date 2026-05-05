# Semantic Pipeline Rewrite — Status (2026-05-06)

## 완료된 작업

### Python 사이드카 (`semantic_service/`)

**삭제:**
- `florence_runner.py` — Florence-2 박스 제안기
- `gdino_runner.py` — GroundingDINO 박스 제안기
- `gemini_runner.py` — 2차 Gemini 분류기

**신규:**
- `llm_runner.py` — 단일 LLM 라운드트립. Gemini 3 Flash (기본) 또는 Claude Sonnet 4.6 (`LLM_PROVIDER=anthropic`). 라벨 카탈로그 + visual_angle 포함. Pydantic structured output 강제.

**재작성:**
- `app.py` — `/detect` 단일 엔드포인트 (LLM→SAM→마스크 응답). `/debug_image`, `/debug_json`, `/healthz`.
- `requirements.txt` — florence/einops/timm 제거; `google-genai`, `anthropic` 추가.
- `.env.bat.example` — Gemini / Anthropic 양쪽 키 설정 예시.
- `run_semantic.bat` — .venv 자동 생성, requirements.txt 설치, uvicorn 7788 실행.

### C++ (`rtabmap/guilib/include/rtabmap/gui/semantic/`)

**`SemanticBackproject.h`**
- `imageAngleToWorldDirBin(visualAngleDeg, robotYawRad) -> int` 추가.
  - image CW-from-up ≈ base CW-from-forward (카메라 roll/pitch 작음 가정).
  - worldCcwDeg = robotYaw×(180/π) − visualAngle → cwFromNorth → 45° 양자화 → 1..8 bin.
  - `M_PI` 의존 없이 `constexpr kRadToDeg` 사용 (MSVC 호환).

**`SemanticMaskStore.h`**
- `labelToGridCode()` → `resolveGridCode(lb, robotYawRad)` 교체.
  - `pillar`, `traffic_cone`, `parking_line`, `no_entry_sign`, `construction_sign` → `1` (벽)
  - `exit_area` → `10` (목적지)
  - `exit_sign`, `floor_arrow` + visualAngle ≥ 0 → `11..18` (10 + dirBin)
  - `one_way_marker` + visualAngle ≥ 0 → `21..28` (20 + dirBin)
  - `lane_divider`, `intersection`, `other`, angle-없는 방향 라벨 → `0` (skip)
- `priorityOf()` 갱신: wall(1)→100, one-way(21..28)→60, dest-dir(11..18)→55, exit-area(10)→50.
- `setLabeledBoxes()` 에서 `e.pendingPose.theta()` 추출 후 `resolveGridCode()` 호출.

**`SemanticWorker.h`**
- 2-stage 배칭 로직 (`PendingFrame`, `kBatchSize`, `flushBatch()`, `httpClassifyBatch()`) 완전 제거.
- `loop()` 단순화: 프레임 수신 → `httpDetect()` → 콜백.
- `httpDetect()` 가 label + confidence + visual_angle 포함 `LabeledBox` 직접 구성.
- read_timeout 30s (LLM 레이턴시 수용).

### 시각화 / 문서

**`grid_client.py`**
- `grid_to_bgr()` 에 11..18 (청록 BGR `220,200,60`) / 21..28 (자홍 BGR `200,60,200`) 색상 추가.
- `n_sem` 카운트 범위 확장.

**`SETUP_GUIDE.md`**
- §1 헤딩 → "1-stage LLM + SAM 사이드카".
- §6-1 파이프라인 다이어그램 갱신.
- §6-2 그리드 셀 표에 11..18 / 21..28 행 추가 + dirBin 규칙.
- §6-3 Gemini vs Anthropic API 키 설정.
- §6-4 SAM만 로컬 다운로드 (~360MB); LLM은 클라우드.
- §6-7 카탈로그 변경 절차 (llm_runner.py + SemanticMaskStore.h).
- §8 /detect 502/503 트러블슈팅, LLM 레이턴시 주의사항.
- §9 파일 구조 갱신.

---

## 남은 작업 (다음 단계)

### 필수 검증

1. **API 키 설정**
   ```bat
   copy semantic_service\.env.bat.example semantic_service\.env.bat
   :: GEMINI_API_KEY 또는 ANTHROPIC_API_KEY 입력
   ```

2. **사이드카 실행 확인**
   ```bat
   semantic_service\run_semantic.bat
   :: 로그에 "Uvicorn running on 127.0.0.1:7788" + SAM 로드 완료 확인
   ```

3. **C++ 재빌드**
   ```bat
   cmake --build rtabmap\build --config Release --target rtabmap_gui --parallel
   ```

4. **동작 검증 (Unity 씬)**
   - 주차선/라바콘 → red cell (grid=1)
   - 출구 영역 → green cell (grid=10)
   - 바닥 화살표 → 청록 cell (grid 11..18); 차량 yaw 90° 회전 후에도 같은 절대 bin 유지
   - `python grid_client.py` 창에서 색상 분포 확인

5. **curl 스모크 테스트**
   ```bat
   curl -X POST http://127.0.0.1:7788/detect ^
        -H "Content-Type: image/jpeg" ^
        --data-binary @test.jpg
   :: 응답 JSON에 detections[].visual_angle 필드 존재 확인
   ```

### 선택적 후속 작업

- **카메라 pitch/roll 정밀 보정**: 현재는 roll/pitch ≈ 0 가정. 정밀도가 필요하면 `imageAngleToWorldDirBin()`에 `opticalToBaseR` 회전 행렬 추가.
- **방향 화살표 오버레이**: `grid_client.py`에서 11..18 / 21..28 셀 위에 dirBin에 맞는 화살표 오버레이 렌더링.
- **loop-closure 후 방향 코드 재계산**: 현재 등록 시점 pose로 bake됨. 루프 클로저로 yaw 보정이 크면 재계산 필요 (45° 양자화로 보통 1-bin 오차 이내).
- **PNG export 컨벤션**: TCP int8 `-1`(unknown)을 prompt.md PNG 컨벤션 `255`로 변환하는 export 단계.
