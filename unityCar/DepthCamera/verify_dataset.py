"""
D455_Dataset 검증 스크립트
Unity RGBD_DataCollector가 저장한 데이터셋이 RTAB-Map 호환인지 확인합니다.
(RGB + Depth + 캘리브레이션 + IMU 검증)

사용법:
  pip install opencv-python numpy
  python verify_dataset.py C:/D455_Dataset
"""

import sys
import os
import glob
import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python이 필요합니다.  pip install opencv-python")
    sys.exit(1)


def verify(dataset_dir: str):
    print(f"\n{'='*60}")
    print(f"  D455 Dataset 검증: {dataset_dir}")
    print(f"{'='*60}\n")

    rgb_dir   = os.path.join(dataset_dir, "rgb_sync")
    depth_dir = os.path.join(dataset_dir, "depth_sync")
    calib     = os.path.join(dataset_dir, "d455_virtual.yaml")

    # ── 1. 폴더 존재 확인 ──
    ok = True
    for d in [rgb_dir, depth_dir]:
        exists = os.path.isdir(d)
        print(f"  [{'OK' if exists else 'FAIL'}] 폴더 존재: {d}")
        ok &= exists

    calib_exists = os.path.isfile(calib)
    print(f"  [{'OK' if calib_exists else 'FAIL'}] 캘리브레이션: {calib}")
    ok &= calib_exists

    if not ok:
        print("\n❌ 기본 구조 검증 실패. Unity에서 Play 후 다시 시도하세요.")
        return

    # ── 2. 파일 수 매칭 ──
    rgb_files   = sorted(glob.glob(os.path.join(rgb_dir, "*.png")))
    depth_files = sorted(glob.glob(os.path.join(depth_dir, "*.png")))

    print(f"\n  RGB 이미지:   {len(rgb_files)}장")
    print(f"  Depth 이미지: {len(depth_files)}장")

    if len(rgb_files) != len(depth_files):
        print("  [WARN] RGB와 Depth 이미지 수가 다릅니다!")
    elif len(rgb_files) == 0:
        print("  [FAIL] 이미지가 없습니다. Unity Play 모드를 실행했는지 확인하세요.")
        return
    else:
        print("  [OK] RGB-Depth 이미지 수 일치")

    # ── 3. RGB 이미지 검증 ──
    print(f"\n── RGB 이미지 검증 (첫 번째) ──")
    rgb_img = cv2.imread(rgb_files[0], cv2.IMREAD_UNCHANGED)
    if rgb_img is not None:
        print(f"  해상도: {rgb_img.shape[1]}×{rgb_img.shape[0]}")
        print(f"  채널:   {rgb_img.shape[2] if len(rgb_img.shape) == 3 else 1}")
        print(f"  dtype:  {rgb_img.dtype}")
        print(f"  [{'OK' if rgb_img.dtype == np.uint8 else 'WARN'}] 8-bit RGB")
    else:
        print("  [FAIL] RGB 이미지 로드 실패")

    # ── 4. Depth 이미지 검증 (핵심!) ──
    print(f"\n── Depth 이미지 검증 (첫 번째) ──")
    depth_img = cv2.imread(depth_files[0], cv2.IMREAD_UNCHANGED)
    if depth_img is not None:
        print(f"  해상도: {depth_img.shape[1]}×{depth_img.shape[0]}")
        print(f"  dtype:  {depth_img.dtype}")

        is_16bit = depth_img.dtype == np.uint16
        print(f"  [{'OK' if is_16bit else 'FAIL'}] 16-bit unsigned (CV_16UC1)")

        if is_16bit:
            valid = depth_img[depth_img > 0]
            if len(valid) > 0:
                min_mm = valid.min()
                max_mm = valid.max()
                mean_mm = valid.mean()
                zero_pct = 100.0 * (depth_img == 0).sum() / depth_img.size

                print(f"  최소 깊이: {min_mm} mm ({min_mm/1000:.2f} m)")
                print(f"  최대 깊이: {max_mm} mm ({max_mm/1000:.2f} m)")
                print(f"  평균 깊이: {mean_mm:.0f} mm ({mean_mm/1000:.2f} m)")
                print(f"  무효 픽셀(0): {zero_pct:.1f}%")

                # D455 스펙 범위 확인 (0.2m ~ 6m)
                in_range = (min_mm >= 100) and (max_mm <= 7000)
                print(f"  [{'OK' if in_range else 'WARN'}] D455 범위 (0.2~6m) 이내")
            else:
                print("  [WARN] 모든 픽셀이 0 (측정 불가). 씬에 3D 오브젝트가 있는지 확인하세요.")
    else:
        print("  [FAIL] Depth 이미지 로드 실패")

    # ── 5. 타임스탬프 파일명 확인 ──
    print(f"\n── 파일명 형식 확인 ──")
    sample = os.path.basename(rgb_files[0])
    try:
        ts = float(sample.replace(".png", ""))
        print(f"  파일명: {sample} → 타임스탬프: {ts:.6f}s")
        print(f"  [OK] RTAB-Map 타임스탬프 형식")
    except ValueError:
        print(f"  파일명: {sample}")
        print(f"  [WARN] 타임스탬프 형식이 아닙니다 (RTAB-Map이 알파벳순 매칭 사용)")

    # ── 6. IMU 데이터 검증 ──
    imu_file = os.path.join(dataset_dir, "imu.csv")
    has_imu = os.path.isfile(imu_file)
    print(f"\n── IMU 데이터 검증 ──")
    print(f"  [{'OK' if has_imu else 'WARN'}] imu.csv {'exists' if has_imu else 'not found (선택적)'}: {imu_file}")

    if has_imu:
        try:
            with open(imu_file, 'r') as f:
                lines = f.readlines()

            header = lines[0].strip()
            data_lines = [l.strip() for l in lines[1:] if l.strip()]

            print(f"  헤더:    {header}")
            print(f"  샘플 수: {len(data_lines)}행")

            expected_header = "timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z"
            if header == expected_header:
                print(f"  [OK] 헤더 형식 일치")
            else:
                print(f"  [WARN] 헤더 불일치! 기대: {expected_header}")

            if len(data_lines) >= 2:
                # 첫번째와 마지막 샘플 검증
                first = data_lines[0].split(',')
                last  = data_lines[-1].split(',')
                if len(first) == 7 and len(last) == 7:
                    t_first = float(first[0])
                    t_last  = float(last[0])
                    duration = t_last - t_first
                    avg_rate = len(data_lines) / max(duration, 0.001)
                    print(f"  시작 타임스탬프: {t_first:.6f}s")
                    print(f"  끝 타임스탬프:   {t_last:.6f}s")
                    print(f"  기간:           {duration:.2f}s")
                    print(f"  평균 샘플률:   {avg_rate:.1f}Hz")

                    # 정지 시 가속도 확인 (Y축 ≈ 9.81)
                    acc_y_first = float(first[5])  # accel_y (Unity Y = up)
                    if 8.0 < abs(acc_y_first) < 12.0:
                        print(f"  [OK] 정지 시 중력 참조 (accel_y={acc_y_first:.2f} ≈ 9.81)")
                    else:
                        print(f"  [WARN] 정지 시 accel_y={acc_y_first:.2f} (기대값 ≈9.81)")
                else:
                    print(f"  [WARN] CSV 열 수 비정상 (expected 7, got {len(first)})")
            elif len(data_lines) == 0:
                print(f"  [WARN] IMU 데이터 없음 (header only)")
        except Exception as e:
            print(f"  [WARN] IMU 검증 오류: {e}")

    # ── 7. 캘리브레이션 내용 확인 ──
    print(f"\n── 캘리브레이션 YAML ──")
    with open(calib, 'r') as f:
        content = f.read()
    print(content[:500])

    # ── 결과 요약 ──
    print(f"\n{'='*60}")
    if is_16bit and len(rgb_files) == len(depth_files) and len(rgb_files) > 0:
        print("  ✅ RTAB-Map 호환 데이터셋 검증 통과!")
        if has_imu:
            print("  ✅ IMU 데이터 포함")
        else:
            print("  ⚠️  IMU 데이터 없음 (--no-imu 옵션으로 실행 가능)")
        print(f"\n  실행 명령 (rtabmap_unity_bridge):")
        print(f"    rtabmap_unity_offline.exe \"{dataset_dir}\"")
        if not has_imu:
            print(f"    rtabmap_unity_offline.exe \"{dataset_dir}\" --no-imu")
    else:
        print("  ❌ 일부 검증 항목이 실패했습니다. 위의 로그를 확인하세요.")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python verify_dataset.py <dataset_directory>")
        print("Example: python verify_dataset.py C:/D455_Dataset")
        sys.exit(1)

    verify(sys.argv[1])
