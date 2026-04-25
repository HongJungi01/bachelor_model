using UnityEngine;

/// <summary>
/// 자전거 모델 기반 차량 컨트롤러.
/// Kinematic Rigidbody 사용 → Unity 물리엔진이 속도/각속도를 계산하므로
/// IMUSensor가 정확한 측정값을 얻을 수 있습니다.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    [Header("주행 설정")]
    [Tooltip("전/후진 속도 (m/s)")]
    public float speed = 5f;

    [Header("조향 설정")]
    [Tooltip("최대 조향각 (도)")]
    public float maxSteerAngle = 35f;

    [Tooltip("A/D 키로 조향각이 변하는 속도 (도/초)")]
    public float steerSpeed = 90f;

    [Tooltip("키를 떼면 핸들이 중앙으로 돌아오는 속도 (도/초)")]
    public float steerReturnSpeed = 120f;

    [Header("차량 제원")]
    [Tooltip("앞바퀴~뒷바퀴 거리 (m). 회전 반경에 영향")]
    public float wheelBase = 2.5f;

    private Rigidbody rb;
    private float currentSteerAngle = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true;          // 물리 충돌 영향 없이 transform 직접 제어
        rb.useGravity  = false;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // ── 입력 (FixedUpdate에서도 GetAxis는 동작) ──
        float moveInput  = Input.GetAxis("Vertical");
        float steerInput = Input.GetAxis("Horizontal");
        float linearSpeed = moveInput * speed;

        // ── 조향각 갱신 ──
        if (Mathf.Abs(steerInput) > 0.01f)
        {
            currentSteerAngle = Mathf.Clamp(
                currentSteerAngle + steerInput * steerSpeed * dt,
                -maxSteerAngle, maxSteerAngle);
        }
        else if (Mathf.Abs(currentSteerAngle) > 0.1f)
        {
            currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, 0f, steerReturnSpeed * dt);
        }
        else
        {
            currentSteerAngle = 0f;
        }

        // ── 자전거 모델 ──
        if (Mathf.Abs(linearSpeed) <= 0.01f) return;

        // 회전: ω = v / R, R = wheelBase / tan(steerAngle)
        Quaternion newRotation = rb.rotation;
        if (Mathf.Abs(currentSteerAngle) > 0.1f)
        {
            float steerRad = currentSteerAngle * Mathf.Deg2Rad;
            float angularVelDeg = (linearSpeed * Mathf.Tan(steerRad) / wheelBase) * Mathf.Rad2Deg;
            Quaternion deltaRot = Quaternion.Euler(0f, angularVelDeg * dt, 0f);
            newRotation = rb.rotation * deltaRot;
            rb.MoveRotation(newRotation);
        }

        // 전방 이동 (새 회전을 반영한 forward)
        Vector3 forward    = newRotation * Vector3.forward;
        Vector3 newPosition = rb.position + forward * (linearSpeed * dt);
        rb.MovePosition(newPosition);
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 300, 25), $"조향각: {currentSteerAngle:F1}°");
        GUI.Label(new Rect(10, 35, 300, 25), $"속도: {Input.GetAxis("Vertical") * speed:F1} m/s");
    }
}
