using UnityEngine;

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

    // 현재 조향각 (도). 좌(-)  우(+)
    private float currentSteerAngle = 0f;

    void Update()
    {
        float dt = Time.deltaTime;

        // ── 1. 전후진 입력 (W/S) ──
        float moveInput = Input.GetAxis("Vertical");   // -1 ~ +1
        float linearSpeed = moveInput * speed;          // m/s

        // ── 2. 조향 입력 (A/D) ──
        float steerInput = Input.GetAxis("Horizontal"); // -1(좌) ~ +1(우)

        if (Mathf.Abs(steerInput) > 0.01f)
        {
            // A/D 누르는 중 → 조향각 증가
            currentSteerAngle += steerInput * steerSpeed * dt;
            currentSteerAngle = Mathf.Clamp(currentSteerAngle, -maxSteerAngle, maxSteerAngle);
        }
        else
        {
            // 키를 뗐으면 → 핸들 자동 복귀 (센터로)
            if (Mathf.Abs(currentSteerAngle) > 0.1f)
                currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, 0f, steerReturnSpeed * dt);
            else
                currentSteerAngle = 0f;
        }

        // ── 3. 자전거 모델 기반 이동 ──
        // 이동 중일 때만 회전 (정지 상태에서 핸들 꺾어도 제자리 회전 안 함)
        if (Mathf.Abs(linearSpeed) > 0.01f)
        {
            if (Mathf.Abs(currentSteerAngle) > 0.1f)
            {
                // 회전 반경 R = wheelBase / tan(steerAngle)
                float steerRad = currentSteerAngle * Mathf.Deg2Rad;
                float turnRadius = wheelBase / Mathf.Tan(steerRad);

                // 각속도 ω = v / R  (도/초)
                float angularVelocity = (linearSpeed / turnRadius) * Mathf.Rad2Deg;

                transform.Rotate(0f, angularVelocity * dt, 0f);
            }

            // 차량 전방으로 이동
            transform.Translate(0f, 0f, linearSpeed * dt);
        }
    }

    // Inspector / 디버그용: 현재 조향각 표시
    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 300, 25), $"조향각: {currentSteerAngle:F1}°");
        GUI.Label(new Rect(10, 35, 300, 25), $"속도: {Input.GetAxis("Vertical") * speed:F1} m/s");
    }
}