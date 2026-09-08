# Visual SLAM 기반 의미론적 매핑 및 경로 생성 시스템

GPS 음영지역(실내 지하주차장)에서 **사전 지도 없이** 자율 출차 경로를 생성하는 시스템입니다.
RGB-D SLAM으로 주행 중 지도를 만들면서, 동시에 장면의 의미(주차면·기둥·차량·출구 표지)를 지도에 입히고, 그 위에서 D* Lite로 경로를 재계획합니다.

홍익대학교 기계·시스템디자인공학과 졸업논문 프로젝트 (2026-1학기, 과제번호 2026-DC-06) — 1/5 스케일 차량 플랫폼으로 실증했습니다.

---

## 왜 이 구조인가

지하주차장은 GNSS가 닿지 않고, 사전 지도를 받을 수도 없습니다. 그래서 다음 세 가지를 동시에 만족해야 합니다.

1. **주행하면서 지도를 만든다** — RTAB-Map 기반 RGB-D SLAM
2. **지도가 의미를 가진다** — 점유 격자만으로는 "출구 방향"을 알 수 없으므로 시맨틱 레이블을 3D 지도에 역투영
3. **지도가 늘어날 때마다 경로를 다시 만든다** — 매번 처음부터 푸는 A* 대신, 변화한 셀만 갱신하는 D* Lite

## 시스템 구성

```
   RealSense D455f ──RGB+Depth+IMU──┐
   (또는 Unity 가상 카메라 / TCP)    │
                                     ▼
                            ┌──────────────────┐
                            │  RTAB-Map (C++)  │  ← 포즈 추정 · 3D 맵 · 2D 점유격자
                            └────────┬─────────┘
                    키프레임 JPEG    │    마스크 + 박스
                            ┌────────▼─────────┐
                            │ semantic_service │  ← YOLO11-seg 비동기 사이드카 (FastAPI)
                            └────────┬─────────┘
                                     ▼
                     SemanticMaskStore → 3D 역투영 → 레이블드 격자
                                     ▼
                            ┌──────────────────┐
                            │  D* Lite (C++)   │  ← 다해상도 재계획
                            └────────┬─────────┘
                                     ▼
                        AutopilotPanel → TCP → 차량 컨트롤러
```

### 설계상의 선택

- **시맨틱 추론을 별도 프로세스로 분리** — YOLO 추론이 SLAM 트래킹 루프를 막지 않도록 FastAPI 사이드카로 떼어내고 키프레임만 비동기로 넘깁니다. 박스 좌표는 호출자가 보낸 이미지 해상도 기준으로 되돌려 주어 C++ 쪽에서 바로 래스터화할 수 있습니다.
- **다해상도 D\* Lite** — 저해상도 격자에서 개략 경로를 먼저 잡고, 차량 주변 윈도우에서만 고해상도로 정밀화해 재계획 비용을 낮췄습니다.
- **미탐색 공간 비용 항** — 아직 관측되지 않은 셀을 무작정 통과 가능으로 두면 경로가 벽을 향해 뻗습니다. 미탐색 영역에 클리어런스 비용을 주어 보수적으로 주행하도록 했습니다.
- **Unity 가상 D455** — 실차 주행 없이 파이프라인을 회귀 검증할 수 있도록 Unity에서 RGB+Depth+IMU를 생성해 동일한 TCP 인터페이스로 흘려보냅니다.

---

## 저장소 안내 — 직접 작성한 코드

본 저장소는 **RTAB-Map(BSD-3-Clause)을 포크**해 그 위에 기능을 얹은 것입니다. `rtabmap/` 하위 대부분은 원저작자(IntRoLab, Université de Sherbrooke)의 코드이며, 아래 파일이 본 프로젝트에서 새로 작성한 부분입니다.

| 경로 | 줄 수 | 내용 |
|------|-------|------|
| `rtabmap/guilib/src/planning/DStarLite.cpp` / `.h` | 760 | D* Lite 증분 경로계획기 C++ 구현 — 다해상도 탐색, 미탐색 공간 클리어런스 비용 |
| `rtabmap/guilib/include/rtabmap/gui/semantic/SemanticMaskStore.h` | 647 | 시맨틱 마스크 저장·3D 역투영·레이블드 격자 병합 |
| `rtabmap/guilib/include/rtabmap/gui/semantic/SemanticWorker.h` | 399 | 사이드카 통신 워커 스레드 (키프레임 큐잉, 논블로킹 추론 요청) |
| `rtabmap/guilib/include/rtabmap/gui/semantic/SemanticBackproject.h` · `SemanticLabeledBox.h` | 138 | 역투영 기하 · 레이블드 박스 자료구조 |
| `rtabmap/guilib/src/AutopilotPanel.cpp` / `.h` | 162 | 경로·포즈를 외부 컨트롤러로 스트리밍하는 GUI 패널 |
| `semantic_service/` | 576 | YOLO11-seg FastAPI 사이드카 — 추론 엔드포인트, 학습 스크립트, 디버그 뷰 |
| `path_planning/` | 743 | D* Lite Python 프로토타입 및 검증용 시뮬레이터 |
| `default_Controller.py` | 657 | 차량측 제어 클라이언트 (경로 추종, PID 튜닝 GUI) |
| `grid_client.py` · `semantic_viewer.py` | 331 | 점유격자 TCP 클라이언트, 시맨틱 결과 시각화 |
| `unityCar/` | — | Unity 가상 D455 카메라 및 차량 프리팹 (C#) |

빌드·실행 절차는 [`SETUP_GUIDE.md`](SETUP_GUIDE.md)를, 시맨틱 파이프라인의 단계별 상태는 [`semantic_pipeline_status.md`](semantic_pipeline_status.md)를 참고하세요.

## 환경

Windows 10/11 · Visual Studio 2022 + CMake · vcpkg · OpenCV · PCL · g2o · Ultralytics YOLO11 · PyTorch · FastAPI · Unity

## 라이선스

RTAB-Map 원본 코드는 BSD-3-Clause를 따릅니다 (`rtabmap/LICENSE`).
