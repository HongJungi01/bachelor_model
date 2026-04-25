using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Unity → rtabmap_pipeline TCP 클라이언트.
///
/// 같은 GameObject (또는 부모/자식)에 부착된 RGBD_DataCollector / IMUSensor 에서
/// 프레임/IMU 샘플을 꺼내 background 스레드로 서버에 전송합니다.
///
/// ■ 프로토콜 (little-endian)
///   공통 헤더 (5B): [type:u8][payloadSize:u32]
///
///   Type=1 Calib (연결 직후 1회):
///     [width:u32][height:u32]
///     [fx:f64][fy:f64][cx:f64][cy:f64]
///     [localTransform[12]:f64]                  → 96 bytes
///     총 payload = 8 + 32 + 96 = 136 bytes
///
///   Type=2 IMU:
///     [stamp:f64][gx,gy,gz:f64][ax,ay,az:f64]   → 56 bytes
///
///   Type=3 RGBD:
///     [stamp:f64][width:u32][height:u32]
///     [rgb: width*height*3 bytes]
///     [depth: width*height*2 bytes]
/// </summary>
public class RTABMapStreamer : MonoBehaviour
{
    private const byte PKT_CALIB = 1;
    private const byte PKT_IMU   = 2;
    private const byte PKT_RGBD  = 3;

    [Header("rtabmap_pipeline 서버")]
    public string serverHost = "127.0.0.1";
    public int    serverPort = 7778;

    [Header("센서 컴포넌트 (비워두면 자동 검색)")]
    public RGBD_DataCollector rgbd;
    public IMUSensor          imu;

    [Header("연결 옵션")]
    [Tooltip("연결 실패 시 재시도 간격(초)")]
    public float reconnectInterval = 2f;
    [Tooltip("연결 끊겨도 자동 재연결")]
    public bool  autoReconnect = true;

    // ── 내부 상태 ──
    private TcpClient     client;
    private NetworkStream stream;
    private Thread        sendThread;
    private volatile bool running;
    private volatile bool calibSent;

    private readonly Queue<byte[]> sendQueue   = new Queue<byte[]>(64);
    private readonly object        sendLock    = new object();
    private readonly AutoResetEvent sendSignal = new AutoResetEvent(false);
    private const int MAX_SEND_QUEUE = 32;

    private readonly List<IMUSensor.Sample> imuBuf = new List<IMUSensor.Sample>(64);

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Unity Lifecycle
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    void Start()
    {
        if (rgbd == null) rgbd = GetComponent<RGBD_DataCollector>();
        if (rgbd == null) rgbd = GetComponentInChildren<RGBD_DataCollector>();
        if (rgbd == null) rgbd = GetComponentInParent<RGBD_DataCollector>();
        if (imu  == null) imu  = GetComponent<IMUSensor>();
        if (imu  == null) imu  = GetComponentInChildren<IMUSensor>();
        if (imu  == null) imu  = GetComponentInParent<IMUSensor>();

        if (rgbd == null) { Debug.LogError("[Streamer] RGBD_DataCollector not found"); enabled = false; return; }
        if (imu  == null) Debug.LogWarning("[Streamer] IMUSensor not found — streaming RGBD only");

        // epoch 동기화 (모든 센서의 timestamp 0를 같은 시각으로)
        double epoch = Time.realtimeSinceStartupAsDouble;
        rgbd.SetEpoch(epoch);
        if (imu != null) imu.SetEpoch(epoch);

        running   = true;
        sendThread = new Thread(SendLoop) { IsBackground = true, Name = "RTABMapStreamer.Send" };
        sendThread.Start();

        Debug.Log($"[Streamer] target = {serverHost}:{serverPort}");
    }

    void Update()
    {
        if (!running) return;

        // RGBD 프레임 큐에서 꺼내 패킷화 → 송신 큐
        while (rgbd.TryDequeueFrame(out RGBD_DataCollector.Frame f))
        {
            EnqueueSend(BuildRGBDPacket(f));
        }

        // IMU 샘플 큐에서 꺼내 패킷화
        if (imu != null)
        {
            imuBuf.Clear();
            imu.DrainSamples(imuBuf);
            for (int i = 0; i < imuBuf.Count; i++)
                EnqueueSend(BuildIMUPacket(imuBuf[i]));
        }
    }

    void OnDisable() { Shutdown(); }
    void OnDestroy() { Shutdown(); }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Send Thread
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private void Shutdown()
    {
        running = false;
        sendSignal.Set();
        try { sendThread?.Join(500); } catch { }
        CloseConnection();
    }

    private void EnqueueSend(byte[] packet)
    {
        lock (sendLock)
        {
            // overflow → drop oldest (latest sensor data more important than backlog)
            while (sendQueue.Count >= MAX_SEND_QUEUE)
                sendQueue.Dequeue();
            sendQueue.Enqueue(packet);
        }
        sendSignal.Set();
    }

    private void SendLoop()
    {
        while (running)
        {
            // ── 연결 ──
            if (client == null || !client.Connected)
            {
                if (!TryConnect())
                {
                    Thread.Sleep((int)(reconnectInterval * 1000));
                    if (!autoReconnect) break;
                    continue;
                }
            }

            // ── 캘리브레이션 (1회) ──
            if (!calibSent)
            {
                try
                {
                    byte[] calib = BuildCalibPacket(rgbd.GetCalibration());
                    stream.Write(calib, 0, calib.Length);
                    calibSent = true;
                    Debug.Log("[Streamer] calibration sent");
                }
                catch (Exception e) { HandleSendFailure(e); continue; }
            }

            // ── 큐에서 꺼내 전송 ──
            byte[] pkt = null;
            lock (sendLock)
            {
                if (sendQueue.Count > 0) pkt = sendQueue.Dequeue();
            }

            if (pkt == null)
            {
                sendSignal.WaitOne(100);
                continue;
            }

            try
            {
                stream.Write(pkt, 0, pkt.Length);
            }
            catch (Exception e) { HandleSendFailure(e); }
        }
    }

    private bool TryConnect()
    {
        try
        {
            client = new TcpClient();
            client.NoDelay = true;
            client.SendBufferSize = 8 * 1024 * 1024;   // 8 MB (RGBD frame ~4.6 MB)
            client.Connect(serverHost, serverPort);
            stream = client.GetStream();
            calibSent = false;
            Debug.Log($"[Streamer] connected → {serverHost}:{serverPort}");
            return true;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[Streamer] connect failed: {e.Message}");
            CloseConnection();
            return false;
        }
    }

    private void HandleSendFailure(Exception e)
    {
        Debug.LogWarning($"[Streamer] send failed: {e.Message} — reconnecting");
        CloseConnection();
        Thread.Sleep((int)(reconnectInterval * 1000));
    }

    private void CloseConnection()
    {
        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }
        stream = null;
        client = null;
        calibSent = false;
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Packet Builders
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    private static byte[] BuildCalibPacket(RGBD_DataCollector.Calibration c)
    {
        const int payloadSize = 4 + 4 + 8 * 4 + 8 * 12;   // 136
        using var ms = new MemoryStream(5 + payloadSize);
        using var bw = new BinaryWriter(ms);
        bw.Write(PKT_CALIB);
        bw.Write((uint)payloadSize);
        bw.Write((uint)c.width);
        bw.Write((uint)c.height);
        bw.Write(c.fx); bw.Write(c.fy); bw.Write(c.cx); bw.Write(c.cy);
        for (int i = 0; i < 12; i++) bw.Write(c.localTransform[i]);
        return ms.ToArray();
    }

    private static byte[] BuildIMUPacket(IMUSensor.Sample s)
    {
        const int payloadSize = 8 + 8 * 6;   // 56
        using var ms = new MemoryStream(5 + payloadSize);
        using var bw = new BinaryWriter(ms);
        bw.Write(PKT_IMU);
        bw.Write((uint)payloadSize);
        bw.Write(s.timestamp);
        bw.Write((double)s.gyro.x);
        bw.Write((double)s.gyro.y);
        bw.Write((double)s.gyro.z);
        bw.Write((double)s.accel.x);
        bw.Write((double)s.accel.y);
        bw.Write((double)s.accel.z);
        return ms.ToArray();
    }

    private static byte[] BuildRGBDPacket(RGBD_DataCollector.Frame f)
    {
        int payloadSize = 8 + 4 + 4 + f.rgb.Length + f.depth.Length;
        using var ms = new MemoryStream(5 + payloadSize);
        using var bw = new BinaryWriter(ms);
        bw.Write(PKT_RGBD);
        bw.Write((uint)payloadSize);
        bw.Write(f.timestamp);
        bw.Write((uint)f.width);
        bw.Write((uint)f.height);
        bw.Write(f.rgb,   0, f.rgb.Length);
        bw.Write(f.depth, 0, f.depth.Length);
        return ms.ToArray();
    }
}
