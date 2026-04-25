using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Unity 네이티브 가상 IMU.
///
/// ■ 측정 원리
///   - Rigidbody가 있으면: PhysX의 linearVelocity / angularVelocity 사용
///   - 없으면: transform 변화량으로 직접 미분 (fallback)
///   - 가속도와 각속도 모두 IMU **body frame** 으로 변환 (실제 IMU와 동일)
///   - 중력은 body frame에 투영하여 specific force에 가산 (정지 시 (0, g, 0) — Unity Y-up)
///
/// ■ 좌표계
///   Unity 카메라/IMU body frame: X-오른쪽, Y-위, Z-앞
///   rtabmap_pipeline의 baseToImu 변환이 이 frame을 base_link로 매핑.
///
/// ■ 사용
///   1. 차량 또는 카메라 GameObject에 부착
///   2. (선택) Rigidbody 컴포넌트 부착 — 더 정확한 측정값
///   3. 외부에서 DrainSamples() 호출하여 큐에서 샘플 꺼내기
/// </summary>
public class IMUSensor : MonoBehaviour
{
    public struct Sample
    {
        public double timestamp;   // seconds since session start
        public Vector3 gyro;       // rad/s, body frame
        public Vector3 accel;      // m/s², body frame, includes gravity (specific force)
    }

    [Header("노이즈 (실제 IMU 모사)")]
    [Tooltip("자이로 가우시안 노이즈 표준편차 (rad/s). 0 = 노이즈 없음")]
    public float gyroNoiseStd = 0.0f;

    [Tooltip("가속도계 가우시안 노이즈 표준편차 (m/s²). 0 = 노이즈 없음")]
    public float accelNoiseStd = 0.0f;

    [Header("타임스탬프 기준")]
    [Tooltip("이 시각을 IMU stamp 0으로 사용. 보통 외부에서 SetEpoch()로 설정")]
    public double epochRealtime = 0.0;

    // ── 내부 상태 ──
    private Rigidbody rb;
    private Vector3 prevPosition;
    private Vector3 prevVelocity;
    private Quaternion prevRotation;
    private bool initialized = false;

    private readonly Queue<Sample> queue = new Queue<Sample>(256);
    private readonly object queueLock = new object();
    private const int MAX_QUEUE = 1024;   // overflow 시 oldest drop

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Public API
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    /// <summary>외부에서 timestamp 0의 기준 시각 설정 (RGBD와 맞추기 위함)</summary>
    public void SetEpoch(double realtimeSinceStartup)
    {
        epochRealtime = realtimeSinceStartup;
    }

    /// <summary>큐에 쌓인 모든 샘플을 꺼냄 (오래된 순). 호출 후 큐는 비워짐.</summary>
    public int DrainSamples(List<Sample> dst)
    {
        lock (queueLock)
        {
            int n = queue.Count;
            while (queue.Count > 0)
                dst.Add(queue.Dequeue());
            return n;
        }
    }

    /// <summary>현재 큐에 쌓인 샘플 수 (디버그용)</summary>
    public int PendingCount
    {
        get { lock (queueLock) { return queue.Count; } }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  Unity Lifecycle
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        epochRealtime = Time.realtimeSinceStartupAsDouble;
    }

    void OnEnable()
    {
        prevPosition = transform.position;
        prevRotation = transform.rotation;
        prevVelocity = GetRigidbodyLinearVelocity();
        initialized = false;
    }

    // Unity 6 renamed Rigidbody.velocity → Rigidbody.linearVelocity. Wrap to support both.
    private Vector3 GetRigidbodyLinearVelocity()
    {
        if (rb == null) return Vector3.zero;
#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity;
#else
        return rb.velocity;
#endif
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0f) return;

        // 첫 프레임은 이전값 캐싱만 (delta 계산 불가)
        if (!initialized)
        {
            prevPosition = transform.position;
            prevRotation = transform.rotation;
            prevVelocity = GetRigidbodyLinearVelocity();
            initialized = true;
            return;
        }

        // ── 월드 프레임 각속도 ──
        Vector3 worldAngVel;
        if (rb != null && !rb.isKinematic)
        {
            worldAngVel = rb.angularVelocity;
        }
        else
        {
            // Kinematic Rigidbody 또는 Rigidbody 없음 → quaternion delta로 미분
            Quaternion deltaRot = transform.rotation * Quaternion.Inverse(prevRotation);
            deltaRot.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (angleDeg > 180f) angleDeg -= 360f;
            float angleRad = angleDeg * Mathf.Deg2Rad;
            worldAngVel = (Mathf.Abs(angleDeg) > 1e-4f && !float.IsNaN(axis.x))
                ? axis.normalized * (angleRad / dt)
                : Vector3.zero;
        }

        // ── 월드 프레임 선가속도 ──
        Vector3 worldVelocity;
        if (rb != null && !rb.isKinematic)
        {
            worldVelocity = GetRigidbodyLinearVelocity();
        }
        else
        {
            worldVelocity = (transform.position - prevPosition) / dt;
        }
        Vector3 worldAccel = (worldVelocity - prevVelocity) / dt;

        // Specific force = 물리 가속도 - 중력 (IMU 기준)
        // Unity 중력은 (0, -9.81, 0). 정지 시 IMU는 (0, +9.81, 0)을 측정 (위로 받치는 힘).
        Vector3 specificForceWorld = worldAccel - Physics.gravity;

        // ── 월드 → body frame 변환 (실제 IMU 측정 좌표) ──
        Vector3 gyroBody  = transform.InverseTransformDirection(worldAngVel);
        Vector3 accelBody = transform.InverseTransformDirection(specificForceWorld);

        // ── 노이즈 (선택) ──
        if (gyroNoiseStd > 0f)
        {
            gyroBody.x += GaussianNoise(gyroNoiseStd);
            gyroBody.y += GaussianNoise(gyroNoiseStd);
            gyroBody.z += GaussianNoise(gyroNoiseStd);
        }
        if (accelNoiseStd > 0f)
        {
            accelBody.x += GaussianNoise(accelNoiseStd);
            accelBody.y += GaussianNoise(accelNoiseStd);
            accelBody.z += GaussianNoise(accelNoiseStd);
        }

        // ── 큐에 적재 ──
        Sample s = new Sample {
            timestamp = Time.realtimeSinceStartupAsDouble - epochRealtime,
            gyro      = gyroBody,
            accel     = accelBody,
        };

        lock (queueLock)
        {
            if (queue.Count >= MAX_QUEUE)
                queue.Dequeue();   // drop oldest
            queue.Enqueue(s);
        }

        // ── 이전값 갱신 ──
        prevPosition = transform.position;
        prevRotation = transform.rotation;
        prevVelocity = worldVelocity;
    }

    // Box-Muller transform
    private float GaussianNoise(float std)
    {
        float u1 = Mathf.Max(1e-6f, Random.value);
        float u2 = Random.value;
        return std * Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }
}
