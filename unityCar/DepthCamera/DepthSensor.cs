using UnityEngine;

[RequireComponent(typeof(Camera))]
public class DepthSensor : MonoBehaviour
{
    [Header("방금 만든 DepthMaterial을 여기에 넣으세요")]
    public Material depthMaterial;

    void Start()
    {
        // 카메라에게 깊이(Depth) 데이터를 수집하라고 명령합니다.
        GetComponent<Camera>().depthTextureMode = DepthTextureMode.Depth;
    }

    // 카메라가 화면을 그릴 때 가로채서 필터를 씌우는 함수
    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if (depthMaterial != null)
        {
            // 뎁스 매터리얼(흑백 화면)을 출력합니다.
            Graphics.Blit(source, destination, depthMaterial);
        }
        else
        {
            // 매터리얼이 없으면 원래 컬러 화면(RGB)을 출력합니다.
            Graphics.Blit(source, destination);
        }
    }
}