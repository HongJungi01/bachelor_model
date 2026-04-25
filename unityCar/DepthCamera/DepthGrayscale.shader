Shader "Custom/DepthGrayscale"
{
    SubShader
    {
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata {
                float4 vertex : POSITION;
            };

            struct v2f {
                float4 pos : SV_POSITION;
                float4 scrPos : TEXCOORD0;
            };

            v2f vert (appdata v) {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.scrPos = ComputeScreenPos(o.pos);
                return o;
            }

            // 카메라가 생성한 깊이 데이터를 가져옵니다
            sampler2D _CameraDepthTexture;

            // float4 사용 — fixed4는 11-bit 정밀도라 16-bit depth 변환 시 손실 발생
            float4 frag (v2f i) : SV_Target {
                // 깊이 값을 0(카메라) ~ 1(farClipPlane) 범위의 선형 값으로 변환
                float depth = Linear01Depth(SAMPLE_DEPTH_TEXTURE_PROJ(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos)));
                return float4(depth, depth, depth, 1.0);
            }
            ENDCG
        }
    }
}