using System;
using System.Globalization;
using System.IO;
using System.Threading;
using Unity.Collections;
using UnityEngine;

/// <summary>
/// Intel RealSense D455 스펙의 가상 RGBD 카메라에서
/// RGB 이미지 + 16-bit Depth 이미지를 캡처하여
/// RTAB-Map 호환 데이터셋 구조로 저장합니다.
///
/// ■ 출력 폴더 구조:
///   {saveDirectory}/
///   ├── rgb_sync/          ← 8-bit RGB PNG
///   ├── depth_sync/        ← 16-bit grayscale PNG (mm 단위, CV_16UC1)
///   ├── imu.csv            ← 가상 IMU 데이터 (FixedUpdate 주기, ~50Hz)
///   └── d455_virtual.yaml  ← OpenCV YAML 카메라 캘리브레이션
///
/// ■ RTAB-Map 사용법:
///   rtabmap-rgbd_dataset {saveDirectory}
///     --calibration {saveDirectory}/d455_virtual.yaml
///     --depth_scale_factor 1.0
///
/// ■ D455 가상 카메라 스펙:
///   해상도: 1280×720 | FOV: 87°(H)×58°(V) | 범위: 0.2~6m | FPS: 최대 30
/// </summary>
[RequireComponent(typeof(Camera))]
public class RGBD_DataCollector : MonoBehaviour
{
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Inspector 설정
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [Header("Depth Material (DepthGrayscale 셰이더)")]
    [Tooltip("DepthMaterial을 드래그하여 할당하세요")]
    public Material depthMaterial;

    [Header("저장 설정")]
    [Tooltip("데이터셋이 저장될 루트 폴더 경로")]
    public string saveDirectory = "C:/D455_Dataset";

    [Tooltip("초당 캡처 횟수 (D455 기본 30fps, 권장 10~15)")]
    [Range(1f, 10f)]
    public float captureFPS = 30f;

    [Header("해상도 (D455: 1280×720)")]
    public int imageWidth  = 1280;
    public int imageHeight = 720;

    [Header("캡처 제어")]
    [Tooltip("체크하면 Play 시 자동 캡처 시작")]
    public bool captureOnStart = true;

    [Tooltip("체크하면 RGB/Depth 이미지를 디스크에 저장합니다 (기본 OFF)")]
    public bool saveImages = false;

    [Header("Game View 표시")]
    [Tooltip("true → Depth 그레이스케일 | false → RGB 컬러")]
    public bool showDepthInGameView = false;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Private 필드
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private Camera cam;
    private float timer = 0f;
    private float interval;
    private bool isCapturing = false;
    private int frameCount = 0;
    private double startTimestamp;

    // 재사용 텍스처 (매 프레임 new 방지 → GC 부하 제거)
    private Texture2D rgbTex;
    private Texture2D depthFloatTex;
    private Texture2D depth16Tex;

    // 폴더 경로
    private string rgbDir;
    private string depthDir;

    // ── IMU 기록 ──
    private StreamWriter imuWriter;
    private Vector3 prevPosition;
    private Vector3 prevVelocity;
    private Quaternion prevRotation;
    private bool imuInitialized = false;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Public API
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>캡처 시작 (Inspector 버튼 또는 외부 스크립트에서 호출)</summary>
    public void StartCapture()
    {
        if (isCapturing) return;
        isCapturing    = true;
        frameCount     = 0;
        timer          = 0f;
        startTimestamp = (double)Time.realtimeSinceStartup;

        // IMU 기록 시작
        if (saveImages)
        {
            string imuPath = Path.Combine(saveDirectory, "imu.csv");
            imuWriter = new StreamWriter(imuPath, false, System.Text.Encoding.ASCII);
            imuWriter.WriteLine("timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z");
            imuWriter.AutoFlush = false;
        }
        prevPosition   = transform.position;
        prevVelocity   = Vector3.zero;
        prevRotation   = transform.rotation;
        imuInitialized = true;

        Debug.Log($"[RGBD] ▶ 캡처 시작 — {captureFPS}fps, {imageWidth}×{imageHeight}, IMU={saveImages}");
    }

    /// <summary>캡처 중지</summary>
    public void StopCapture()
    {
        if (!isCapturing) return;
        isCapturing = false;

        // IMU 기록 종료
        if (imuWriter != null)
        {
            imuWriter.Flush();
            imuWriter.Close();
            imuWriter = null;
            Debug.Log("[RGBD] IMU CSV 저장 완료");
        }
        imuInitialized = false;

        Debug.Log($"[RGBD] ■ 캡처 중지 — 총 {frameCount}프레임 저장됨 → {saveDirectory}");
    }

    /// <summary>현재 캡처 중인지 여부</summary>
    public bool IsCapturing => isCapturing;

    /// <summary>저장된 프레임 수</summary>
    public int FrameCount => frameCount;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Unity Lifecycle
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    void Start()
    {
        cam = GetComponent<Camera>();
        cam.depthTextureMode = DepthTextureMode.Depth;

        interval = 1f / captureFPS;

        // 폴더 경로 설정
        rgbDir   = Path.Combine(saveDirectory, "rgb_sync");
        depthDir = Path.Combine(saveDirectory, "depth_sync");

        // 기존 이미지 삭제 후 폴더 재생성
        ClearDirectory(rgbDir);
        ClearDirectory(depthDir);

        // 재사용 텍스처 생성
        rgbTex        = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        depthFloatTex = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        depth16Tex    = new Texture2D(imageWidth, imageHeight, TextureFormat.R16, false);

        // RTAB-Map용 캘리브레이션 YAML 생성
        SaveCalibrationYAML();

        Debug.Log($"[RGBD] 데이터셋 경로: {saveDirectory}");
        Debug.Log($"[RGBD] 카메라: FOV={cam.fieldOfView}°, Near={cam.nearClipPlane}m, Far={cam.farClipPlane}m");

        if (captureOnStart)
            StartCapture();
    }

    void Update()
    {
        if (!isCapturing) return;
        timer += Time.deltaTime;
    }

    /// <summary>
    /// FixedUpdate에서 가상 IMU 데이터를 기록합니다.
    /// Transform 변화량으로부터 각속도(gyro)와 가속도(accel)를 계산합니다.
    ///
    /// 좌표계: RTABMap base_link (X:앞, Y:왼쪽, Z:위) — 월드 좌표에서 직접 변환
    /// 가속도에는 중력(9.81 m/s²)이 포함됩니다 (실제 IMU와 동일).
    /// IMU local_transform = identity (0 0 0 0 0 0) 로 설정.
    /// </summary>
    void FixedUpdate()
    {
        if (!isCapturing || !saveImages || imuWriter == null || !imuInitialized) return;

        float dt = Time.fixedDeltaTime;
        if (dt <= 0f) return;

        double timestamp = (double)Time.realtimeSinceStartup - startTimestamp;

        // ── 각속도 (Angular Velocity) ──
        // 쿼터니언 차이로 회전 변화량 계산 → 각속도 (rad/s)
        Quaternion deltaRot = transform.rotation * Quaternion.Inverse(prevRotation);
        deltaRot.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        float angleRad = angleDeg * Mathf.Deg2Rad;

        Vector3 worldAngVel = Vector3.zero;
        if (Mathf.Abs(angleDeg) > 0.001f && !float.IsNaN(axis.x))
        {
            worldAngVel = axis.normalized * (angleRad / dt);
        }

        // ── 가속도 (Linear Acceleration) ──
        Vector3 currentVelocity = (transform.position - prevPosition) / dt;
        Vector3 worldAccel = (currentVelocity - prevVelocity) / dt;

        // Specific force = 물리 가속도 + 중력 (IMU 측정값, 정지 시 (0, 9.81, 0))
        Vector3 specificForceWorld = worldAccel + new Vector3(0f, 9.81f, 0f);

        // ── Unity 월드 (Y-up, 좌손) → RTABMap 월드 (Z-up, 우손) 변환 ──
        // RTABMap X = Unity Z (앞)
        // RTABMap Y = -Unity X (왼쪽)
        // RTABMap Z = Unity Y (위)
        float gx = worldAngVel.z;
        float gy = -worldAngVel.x;
        float gz = worldAngVel.y;
        float ax = specificForceWorld.z;
        float ay = -specificForceWorld.x;
        float az = specificForceWorld.y;

        // ── CSV 기록 ──
        imuWriter.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6}",
            timestamp, gx, gy, gz, ax, ay, az));

        // ── 이전 값 업데이트 ──
        prevPosition = transform.position;
        prevVelocity = currentVelocity;
        prevRotation = transform.rotation;
    }

    /// <summary>
    /// 카메라 렌더링 완료 후 호출.
    /// source = RGB 원본, destination = Game View 출력.
    /// 여기서 Depth RT를 생성하고 RGB/Depth를 동시에 캡처합니다.
    /// </summary>
    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // ── 1. Depth를 float precision RT에 렌더링 ──
        RenderTexture depthRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 0, RenderTextureFormat.RFloat);
        depthRT.filterMode = FilterMode.Point; // 깊이값 보간 방지

        if (depthMaterial != null)
            Graphics.Blit(source, depthRT, depthMaterial);

        // ── 2. Game View에 표시할 이미지 선택 ──
        if (showDepthInGameView && depthMaterial != null)
        {
            // Depth를 시각화용 ARGB32에 다시 Blit (RFloat는 직접 표시 안됨)
            RenderTexture visRT = RenderTexture.GetTemporary(
                source.width, source.height, 0, RenderTextureFormat.ARGB32);
            Graphics.Blit(source, visRT, depthMaterial);
            Graphics.Blit(visRT, destination);
            RenderTexture.ReleaseTemporary(visRT);
        }
        else
        {
            Graphics.Blit(source, destination);
        }

        // ── 3. 타이머 기반 캡처 ──
        if (isCapturing && timer >= interval)
        {
            timer -= interval;

            // 타임스탬프 기반 파일명 (RTAB-Map이 파일명에서 타임스탬프 파싱)
            double timestamp = (double)Time.realtimeSinceStartup - startTimestamp;
            string filename  = $"{timestamp:F6}.png";

            // 이미지 저장 (토글 ON일 때만)
            if (saveImages)
            {
                CaptureRGB(source, Path.Combine(rgbDir, filename));

                if (depthMaterial != null)
                    CaptureDepth16(depthRT, Path.Combine(depthDir, filename));
            }

            frameCount++;

            if (frameCount % 100 == 0)
                Debug.Log($"[RGBD] {frameCount}프레임 저장 완료");
        }

        RenderTexture.ReleaseTemporary(depthRT);
    }

    void OnDestroy()
    {
        if (isCapturing)
            StopCapture();

        // IMU writer 안전 종료
        if (imuWriter != null)
        {
            imuWriter.Flush();
            imuWriter.Close();
            imuWriter = null;
        }

        if (rgbTex != null)         Destroy(rgbTex);
        if (depthFloatTex != null)  Destroy(depthFloatTex);
        if (depth16Tex != null)     Destroy(depth16Tex);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  폴더 초기화
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>
    /// 지정 폴더 내 모든 파일을 삭제하고 폴더를 재생성합니다.
    /// </summary>
    private void ClearDirectory(string dirPath)
    {
        if (Directory.Exists(dirPath))
        {
            Directory.Delete(dirPath, true);
            Debug.Log($"[RGBD] 기존 데이터 삭제: {dirPath}");
        }
        Directory.CreateDirectory(dirPath);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  RGB 캡처
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private void CaptureRGB(RenderTexture source, string filePath)
    {
        // source 해상도가 imageWidth×Height과 다를 수 있으므로 리사이즈
        RenderTexture rgbRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 0, RenderTextureFormat.ARGB32);
        Graphics.Blit(source, rgbRT);

        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = rgbRT;

        rgbTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        rgbTex.Apply();

        RenderTexture.active = prev;
        RenderTexture.ReleaseTemporary(rgbRT);

        byte[] pngData = rgbTex.EncodeToPNG();
        SaveAsync(pngData, filePath);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Depth 캡처 (16-bit unsigned PNG, mm)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>
    /// depthRT (RFloat, Linear01Depth) → 16-bit unsigned PNG (mm 단위)
    ///
    /// 변환 과정:
    ///   셰이더 출력: Linear01Depth  (0.0 = 카메라, 1.0 = farClipPlane)
    ///   실제 거리(m): linearDepth × farClipPlane
    ///   mm 변환:      거리(m) × 1000
    ///
    /// 특수 처리:
    ///   farClip 근처(≥0.999) → 0 (스카이박스/무한원 = 측정 불가)
    ///   nearClip 이하         → 0 (D455 최소 측정거리 미달)
    /// </summary>
    private void CaptureDepth16(RenderTexture depthRT, string filePath)
    {
        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = depthRT;

        depthFloatTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        depthFloatTex.Apply();

        RenderTexture.active = prev;

        // Float → 16-bit mm 변환
        NativeArray<float>  floatData  = depthFloatTex.GetRawTextureData<float>();
        NativeArray<ushort> ushortData = depth16Tex.GetRawTextureData<ushort>();

        float farClip  = cam.farClipPlane;
        float nearClip = cam.nearClipPlane;
        float nearNorm = nearClip / farClip; // nearClip을 정규화 값으로

        for (int i = 0; i < floatData.Length; i++)
        {
            float linear01 = floatData[i];

            // 스카이박스/배경 (farClip 도달) → 측정 불가
            if (linear01 >= 0.999f)
            {
                ushortData[i] = 0;
                continue;
            }

            // nearClip 미만 → 측정 불가 (D455 최소거리 0.2m)
            if (linear01 <= nearNorm * 0.5f)
            {
                ushortData[i] = 0;
                continue;
            }

            // 실제 거리를 mm로 변환
            float depthMeters = linear01 * farClip;
            ushort depthMm = (ushort)Mathf.Clamp(depthMeters * 1000f, 0f, 65535f);
            ushortData[i] = depthMm;
        }

        // R16 Texture → 16-bit grayscale PNG
        byte[] pngData = depth16Tex.EncodeToPNG();
        SaveAsync(pngData, filePath);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  비동기 파일 저장
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>
    /// PNG 바이트 배열을 백그라운드 스레드에서 디스크에 저장합니다.
    /// 메인 스레드 블로킹을 방지하여 프레임 드롭을 최소화합니다.
    /// </summary>
    private void SaveAsync(byte[] data, string filePath)
    {
        ThreadPool.QueueUserWorkItem(_ =>
        {
            try
            {
                File.WriteAllBytes(filePath, data);
            }
            catch (Exception e)
            {
                Debug.LogError($"[RGBD] 파일 저장 실패: {filePath}\n{e.Message}");
            }
        });
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  카메라 캘리브레이션 YAML 생성
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>
    /// Unity 카메라 파라미터로부터 RTAB-Map/OpenCV 호환 캘리브레이션 YAML을 생성합니다.
    ///
    /// 계산:
    ///   fy = height / (2 × tan(vfov/2))
    ///   fx = fy  (정사각 픽셀)
    ///   cx = width/2,  cy = height/2  (주점 = 이미지 중앙)
    ///   왜곡 = [0,0,0,0,0]  (가상 카메라 = 렌즈 왜곡 없음)
    ///
    /// local_transform:
    ///   카메라 optical frame (Z-forward, X-right, Y-down) →
    ///   로봇 base frame (X-forward, Y-left, Z-up) 변환 행렬
    ///   카메라 높이(Unity Y좌표)를 base Z 오프셋으로 반영 (지면 = Z=0)
    /// </summary>
    private void SaveCalibrationYAML()
    {
        float vfovRad = cam.fieldOfView * Mathf.Deg2Rad;
        float fy = imageHeight / (2f * Mathf.Tan(vfovRad / 2f));
        float fx = fy;
        float cx = imageWidth  / 2f;
        float cy = imageHeight / 2f;

        // 수평 FOV 계산 (확인용)
        float hfov = 2f * Mathf.Atan2(imageWidth, 2f * fx) * Mathf.Rad2Deg;

        string calibPath = Path.Combine(saveDirectory, "d455_virtual.yaml");

        // OpenCV FileStorage YAML 포맷 (RTAB-Map CameraModel이 읽는 형식)
        string yaml =
$@"%YAML:1.0
---
camera_name: ""d455_virtual""
image_width: {imageWidth}
image_height: {imageHeight}
camera_matrix:
  rows: 3
  cols: 3
  data: [ {fx:F6}, 0., {cx:F6}, 0., {fy:F6}, {cy:F6}, 0., 0., 1. ]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [ 0., 0., 0., 0., 0. ]
distortion_model: plumb_bob
rectification_matrix:
  rows: 3
  cols: 3
  data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]
projection_matrix:
  rows: 3
  cols: 4
  data: [ {fx:F6}, 0., {cx:F6}, 0., 0., {fy:F6}, {cy:F6}, 0., 0., 0., 1., 0. ]
local_transform:
  rows: 3
  cols: 4
  data: [ 0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 0. ]
";

        File.WriteAllText(calibPath, yaml);
        Debug.Log($"[RGBD] 캘리브레이션 저장: {calibPath} (카메라 높이={transform.position.y:F2}m)");
        Debug.Log($"[RGBD] fx={fx:F2}, fy={fy:F2}, cx={cx:F2}, cy={cy:F2}, HFOV={hfov:F1}°, VFOV={cam.fieldOfView}°");
    }
}
