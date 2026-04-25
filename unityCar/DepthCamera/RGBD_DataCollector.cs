using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

/// <summary>
/// Intel RealSense D455 스펙의 가상 RGBD 카메라.
/// 매 프레임 RGB + 16-bit Depth(mm)를 캡처해 내부 큐에 적재합니다.
/// 디스크에는 저장하지 않으며, RTABMapStreamer가 큐에서 꺼내 TCP로 전송합니다.
///
/// ■ D455 가상 스펙: 1280×720 | FOV 87°(H)×58°(V) | 0.2~6m | ~30fps
///
/// ■ 출력 좌표
///   픽셀 데이터는 OpenCV/PNG 컨벤션(top-left origin)으로 정렬 후 큐에 적재.
///   캘리브레이션 local_transform은 OpenCV optical → base_link 변환.
/// </summary>
[RequireComponent(typeof(Camera))]
public class RGBD_DataCollector : MonoBehaviour
{
    public struct Frame
    {
        public double timestamp;   // seconds since epoch (set by SetEpoch)
        public int    width;
        public int    height;
        public byte[] rgb;         // length = width*height*3, RGB24, top-left origin
        public byte[] depth;       // length = width*height*2, uint16 mm little-endian, top-left origin
    }

    public struct Calibration
    {
        public int      width;
        public int      height;
        public double   fx, fy, cx, cy;
        public double[] localTransform;   // 12 doubles, row-major 3x4 (optical → base_link)
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Inspector
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [Header("Depth Material (DepthGrayscale 셰이더)")]
    public Material depthMaterial;

    [Header("해상도 (D455: 1280×720)")]
    public int imageWidth  = 1280;
    public int imageHeight = 720;

    [Tooltip("초당 캡처 횟수 (D455 기본 30fps, 권장 10~15)")]
    [Range(1f, 30f)]
    public float captureFPS = 15f;

    [Header("Game View 표시")]
    [Tooltip("true → Depth 그레이스케일 | false → RGB 컬러")]
    public bool showDepthInGameView = false;

    [Header("큐 동작")]
    [Tooltip("내부 큐가 꽉 차면 oldest frame 드롭")]
    public int maxQueueSize = 4;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Private
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private Camera cam;
    private float  timer = 0f;
    private float  interval;
    private double epochRealtime;

    // 재사용 텍스처 (GC 부하 제거)
    private Texture2D rgbTex;
    private Texture2D depthFloatTex;

    // 큐 (producer: main thread, consumer: streamer 외부 스레드)
    private readonly Queue<Frame> queue = new Queue<Frame>(8);
    private readonly object queueLock   = new object();

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Public API
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    public void SetEpoch(double realtimeSinceStartup)
    {
        epochRealtime = realtimeSinceStartup;
    }

    public bool TryDequeueFrame(out Frame f)
    {
        lock (queueLock)
        {
            if (queue.Count == 0) { f = default; return false; }
            f = queue.Dequeue();
            return true;
        }
    }

    public int PendingCount
    {
        get { lock (queueLock) { return queue.Count; } }
    }

    /// <summary>
    /// 현재 카메라 파라미터로 calibration 계산.
    /// local_transform: OpenCV optical (X-right, Y-down, Z-forward) → base_link (X-forward, Y-left, Z-up)
    /// </summary>
    public Calibration GetCalibration()
    {
        if (cam == null) cam = GetComponent<Camera>();

        float vfovRad = cam.fieldOfView * Mathf.Deg2Rad;
        float fy = imageHeight / (2f * Mathf.Tan(vfovRad / 2f));
        float fx = fy;
        float cx = imageWidth  / 2f;
        float cy = imageHeight / 2f;

        return new Calibration {
            width  = imageWidth,
            height = imageHeight,
            fx = fx, fy = fy, cx = cx, cy = cy,
            localTransform = new double[] {
                0,  0, 1, 0,
               -1,  0, 0, 0,
                0, -1, 0, 0,
            },
        };
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Unity Lifecycle
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    void Start()
    {
        cam = GetComponent<Camera>();
        cam.depthTextureMode = DepthTextureMode.Depth;
        interval = 1f / captureFPS;
        epochRealtime = Time.realtimeSinceStartupAsDouble;

        rgbTex        = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        depthFloatTex = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);

        Debug.Log($"[RGBD] {imageWidth}×{imageHeight} @ {captureFPS}fps, FOV={cam.fieldOfView}°");
    }

    void Update()
    {
        timer += Time.deltaTime;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // ── Depth를 float precision RT에 렌더링 ──
        RenderTexture depthRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 0, RenderTextureFormat.RFloat);
        depthRT.filterMode = FilterMode.Point;

        if (depthMaterial != null)
            Graphics.Blit(source, depthRT, depthMaterial);

        // ── Game View 출력 ──
        if (showDepthInGameView && depthMaterial != null)
        {
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

        // ── 타이머 기반 캡처 ──
        if (timer >= interval && depthMaterial != null)
        {
            timer -= interval;

            byte[] rgbBytes   = CaptureRGB(source);
            byte[] depthBytes = CaptureDepth16(depthRT);

            Frame f = new Frame {
                timestamp = Time.realtimeSinceStartupAsDouble - epochRealtime,
                width     = imageWidth,
                height    = imageHeight,
                rgb       = rgbBytes,
                depth     = depthBytes,
            };

            lock (queueLock)
            {
                while (queue.Count >= maxQueueSize)
                    queue.Dequeue();   // drop oldest
                queue.Enqueue(f);
            }
        }

        RenderTexture.ReleaseTemporary(depthRT);
    }

    void OnDestroy()
    {
        if (rgbTex != null)        Destroy(rgbTex);
        if (depthFloatTex != null) Destroy(depthFloatTex);
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  RGB → byte[] (top-left origin, RGB24)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private byte[] CaptureRGB(RenderTexture source)
    {
        RenderTexture rgbRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 0, RenderTextureFormat.ARGB32);
        Graphics.Blit(source, rgbRT);

        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = rgbRT;
        rgbTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        rgbTex.Apply();
        RenderTexture.active = prev;
        RenderTexture.ReleaseTemporary(rgbRT);

        // Unity raw data: bottom-left origin → flip to top-left (OpenCV/PNG)
        byte[] src = rgbTex.GetRawTextureData();
        int stride = imageWidth * 3;
        byte[] dst = new byte[src.Length];
        for (int y = 0; y < imageHeight; y++)
        {
            System.Buffer.BlockCopy(src, (imageHeight - 1 - y) * stride, dst, y * stride, stride);
        }
        return dst;
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Depth → byte[] (top-left, uint16 mm, little-endian)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private byte[] CaptureDepth16(RenderTexture depthRT)
    {
        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = depthRT;
        depthFloatTex.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        depthFloatTex.Apply();
        RenderTexture.active = prev;

        NativeArray<float> floatData = depthFloatTex.GetRawTextureData<float>();

        float farClip  = cam.farClipPlane;
        float nearClip = cam.nearClipPlane;
        float nearNorm = nearClip / farClip;

        byte[] outBytes = new byte[imageWidth * imageHeight * 2];

        // Convert + vertical flip (Unity bottom-left → OpenCV top-left)
        for (int y = 0; y < imageHeight; y++)
        {
            int srcRow = (imageHeight - 1 - y) * imageWidth;
            int dstRow = y * imageWidth * 2;

            for (int x = 0; x < imageWidth; x++)
            {
                float linear01 = floatData[srcRow + x];

                ushort mm;
                if (linear01 >= 0.999f || linear01 <= nearNorm * 0.5f)
                    mm = 0;   // skybox / under-near = invalid
                else
                    mm = (ushort)Mathf.Clamp(linear01 * farClip * 1000f, 0f, 65535f);

                int o = dstRow + x * 2;
                outBytes[o]     = (byte)(mm & 0xFF);
                outBytes[o + 1] = (byte)((mm >> 8) & 0xFF);
            }
        }
        return outBytes;
    }
}
