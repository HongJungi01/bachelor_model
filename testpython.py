import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# ── 물성치 (막온도 Tf = 70°C, 1 atm) ──────────────────────────────────────
nu  = 19.95e-6   # 동점도 [m²/s]
k   = 0.02881    # 열전도율 [W/m·K]
Pr  = 0.7177     # 프란틀 수
V   = 7.0        # 유속 [m/s]
T_inf = 120.0    # 자유류 온도 [°C]
T_s   = 20.0     # 표면 온도 [°C]
Re_cr = 5e5      # 임계 레이놀즈 수

# ── 임계 거리 ──────────────────────────────────────────────────────────────
x_cr = Re_cr * nu / V
print(f"임계 거리 x_cr = {x_cr:.4f} m")

# ── x 배열 (0.2 ~ 3 m) ────────────────────────────────────────────────────
x = np.linspace(0.2, 3.0, 2000)
Re_x = V * x / nu

# ── 국소 누셀트 수 & 열전달 계수 ──────────────────────────────────────────
h = np.where(
    Re_x < Re_cr,
    # 층류: Nu_x = 0.332 * Re_x^0.5 * Pr^(1/3)
    (0.332 * Re_x**0.5 * Pr**(1/3)) * k / x,
    # 난류: Nu_x = 0.0296 * Re_x^0.8 * Pr^(1/3)
    (0.0296 * Re_x**0.8 * Pr**(1/3)) * k / x
)

# ── 분리된 층류 / 난류 구간 ────────────────────────────────────────────────
mask_lam = Re_x < Re_cr
mask_tur = Re_x >= Re_cr

# ── 그래프 ────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 6))

# 배경 색 채우기
ax.axvspan(0.2,  x_cr, alpha=0.10, color='royalblue', label='_nolegend_')
ax.axvspan(x_cr, 3.0,  alpha=0.10, color='tomato',    label='_nolegend_')

# 임계 거리 수직선
ax.axvline(x=x_cr, color='gray', linestyle='--', linewidth=1.5,
           label=f'Critical distance $x_{{cr}}$ = {x_cr:.3f} m')

# Laminar / Turbulent curves
ax.plot(x[mask_lam], h[mask_lam], color='royalblue', linewidth=2.5, label='Laminar  ($Nu_x = 0.332\,Re_x^{1/2}\,Pr^{1/3}$)')
ax.plot(x[mask_tur], h[mask_tur], color='tomato',    linewidth=2.5, label='Turbulent  ($Nu_x = 0.0296\,Re_x^{4/5}\,Pr^{1/3}$)')

# Transition point
h_cr = (0.332 * Re_cr**0.5 * Pr**(1/3)) * k / x_cr
ax.plot(x_cr, h_cr, 'ko', markersize=7, zorder=5)
ax.annotate(f'Transition point\n({x_cr:.2f} m,  {h_cr:.2f} W/m²K)',
            xy=(x_cr, h_cr), xytext=(x_cr + 0.15, h_cr + 1.5),
            fontsize=10, arrowprops=dict(arrowstyle='->', color='black'),
            bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray'))

# Axes
ax.set_xlabel('Distance from leading edge  $x$  [m]', fontsize=13)
ax.set_ylabel('Local heat transfer coefficient  $h_x$  [W/m²·K]', fontsize=13)
ax.set_title('Problem 7-39: Local convection heat transfer coefficient along a flat plate\n'
             r'($V_\infty=7$ m/s,  $T_\infty=120°C$,  $T_s=20°C$,  1 atm)',
             fontsize=13, pad=12)
ax.set_xlim(0.2, 3.0)
ax.set_ylim(0, None)
ax.grid(True, linestyle='--', alpha=0.5)
ax.legend(fontsize=10)

# Region labels
ymax = ax.get_ylim()[1]
ax.text(0.75,  ymax * 0.88, 'Laminar',   color='royalblue', fontsize=12, ha='center', fontweight='bold')
ax.text(2.30,  ymax * 0.30, 'Turbulent', color='tomato',    fontsize=12, ha='center', fontweight='bold')

plt.tight_layout()
plt.savefig('/mnt/user-data/outputs/heat_transfer_7_39.png', dpi=150, bbox_inches='tight')
plt.show()
print("그래프 저장 완료!")