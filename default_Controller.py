#!/usr/bin/env python3
"""
디버깅용 PID 속도/조향 제어 GUI
PID로 구동/조향 모터를 제어하고
엔코더/가변저항 피드백을 실시간 모니터링

Arduino 프로토콜:
  송신: V<speed>,A<angle>\n   (speed: -15~15, angle: -20~20)
  수신: E<delta>,P<pot>,C<cumulative>\n
"""

import math
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports


class DebugController:
    BAUD_RATE = 115200
    SEND_INTERVAL_MS = 20    # 20ms
    SPEED_ACCEL_MS = 1000    # W/S를 누르면 이 시간에 걸쳐 목표 속도까지 가속
    SPEED_DECAY_MS = 1000    # W/S를 떼면 이 시간에 걸쳐 0으로 감속

    def __init__(self):
        self.ser = None
        self.connected = False
        self.running = True

        # 제어값
        self.speed = 0          # -15 ~ 15 (감속 중에는 float)
        self.angle = 0          # -20 ~ 20
        self.target_speed = 0   # 키/슬라이더로 지정된 목표 속도 (여기로 램프)

        # 키 입력 고정값 (인터페이스에서 변경 가능)
        self.hold_speed = 5     # W/S를 누르면 도달할 속도
        self.hold_angle = 15    # A/D를 누르면 도달할 조향각

        # 오토모드 (rtabgui 경로 추종, pure-pursuit)
        self.auto_mode = False
        self.auto_speed = 3          # 오토모드 구동 속도
        self.look_ahead = 0.7        # pure-pursuit 룩어헤드 거리 (m)
        self.FULL_LOCK_DEG = 30.0    # heading error -> ±20 매핑(이 각도면 풀락)
        self.PLAN_STALE_SEC = 1.5    # 데이터가 이보다 오래되면 정지 (안전 워치독)

        # rtabgui 경로+pose TCP 스트림 수신 (PlanTcpStreamer)
        self.plan_host = "127.0.0.1"
        self.plan_port = 5006
        self.plan_sock = None
        self.plan_connected = False
        self.latest_path = []        # [(x, y), ...] 월드 m
        self.latest_pose = None      # (x, y, yaw)
        self.last_plan_time = 0.0
        self.plan_lock = threading.Lock()

        # 피드백값
        self.enc_delta = 0
        self.pot_value = 0
        self.enc_cumulative = 0

        # 키보드
        self.keys_pressed = set()

        self._build_gui()
        self._bind_keys()

    # ================================================================
    # 시리얼 I/O
    # ================================================================
    def get_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self):
        port = self.port_var.get()
        if not port:
            return
        try:
            self.ser = serial.Serial(port, self.BAUD_RATE, timeout=0.1)
            self.connected = True
            self.status_label.config(text="● 연결됨", foreground="green")
            self.connect_btn.config(text="Disconnect")
            threading.Thread(target=self._read_loop, daemon=True).start()
        except serial.SerialException as ex:
            self.status_label.config(text=f"✗ {ex}", foreground="red")

    def disconnect(self):
        # 모터 정지 명령 전송
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"V0,A0\n")
            except (serial.SerialException, OSError):
                pass
        self.connected = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.status_label.config(text="○ 미연결", foreground="gray")
        self.connect_btn.config(text="Connect")

    def toggle_connect(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def _read_loop(self):
        """백그라운드 스레드: 아두이노 피드백 수신"""
        buf = ""
        while self.connected and self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    raw = self.ser.read(self.ser.in_waiting)
                    buf += raw.decode('ascii', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        self._parse_feedback(line)
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError):
                break

    def _parse_feedback(self, line):
        """E<delta>,P<pot>,C<cumulative> 형식 파싱"""
        try:
            if not line.startswith('E'):
                # 일반 메시지는 로그에 표시
                self._append_log(line)
                return

            parts = line.split(',')
            for part in parts:
                if part.startswith('E'):
                    self.enc_delta = int(part[1:])
                elif part.startswith('P'):
                    self.pot_value = int(part[1:])
                elif part.startswith('C'):
                    self.enc_cumulative = int(part[1:])
        except (ValueError, IndexError):
            pass

    def _tick(self):
        """상시 루프: 속도 램프 처리 + 주기적 명령 전송 + 화면 갱신"""
        if self.auto_mode:
            self._apply_auto_control()
        self._ramp_speed()

        if self.connected and self.ser and self.ser.is_open:
            try:
                cmd = f"V{int(round(self.speed))},A{self.angle}\n"
                self.ser.write(cmd.encode('ascii'))
            except (serial.SerialException, OSError):
                self.disconnect()

        self._update_controls_display()
        self._update_display()
        self.root.after(self.SEND_INTERVAL_MS, self._tick)

    def _ramp_speed(self):
        """현재 속도를 목표 속도(target_speed)로 서서히 이동.
        0에서 멀어지는 구간은 가속, 0으로 다가가는 구간은 감속 레이트 사용."""
        target = self.target_speed
        if self.speed == target:
            return
        # 0 쪽으로 가는 중이면(목표 크기가 더 작거나 부호가 반대) 감속, 아니면 가속
        if abs(target) < abs(self.speed) or target * self.speed < 0:
            ramp_ms = self.SPEED_DECAY_MS
        else:
            ramp_ms = self.SPEED_ACCEL_MS
        # 한 틱당 변화량: 최대 속도 기준으로 ramp_ms가 걸리도록 (램프 속도 일정)
        step = max(self.hold_speed, 1) * self.SEND_INTERVAL_MS / ramp_ms
        if abs(target - self.speed) <= step:
            self.speed = target
        elif target > self.speed:
            self.speed += step
        else:
            self.speed -= step

    # ================================================================
    # GUI
    # ================================================================
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("디버깅 — PID 속도/조향 제어")
        self.root.resizable(False, False)
        self.root.configure(padx=8, pady=8)

        style = ttk.Style()
        style.configure('TLabelframe.Label', font=('Arial', 10, 'bold'))

        # ---- 시리얼 연결 ----
        conn = ttk.LabelFrame(self.root, text="시리얼 연결", padding=8)
        conn.pack(fill='x', pady=(0, 5))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var,
                                       width=15, state='readonly')
        self.port_combo.pack(side='left', padx=(0, 5))

        ttk.Button(conn, text="Refresh", width=8,
                   command=self._refresh_ports).pack(side='left', padx=(0, 5))

        self.connect_btn = ttk.Button(conn, text="Connect", width=12,
                                      command=self.toggle_connect)
        self.connect_btn.pack(side='left', padx=(0, 10))

        self.status_label = ttk.Label(conn, text="○ 미연결", foreground="gray")
        self.status_label.pack(side='left')

        # ---- PID 속도/조향 ----
        ctrl = ttk.LabelFrame(self.root, text="PID 속도/조향 제어", padding=8)
        ctrl.pack(fill='x', pady=5)

        # Speed
        df = ttk.Frame(ctrl)
        df.pack(fill='x', pady=2)
        ttk.Label(df, text="Speed:", width=7).pack(side='left')
        self.speed_var = tk.IntVar(value=0)
        self.speed_scale = ttk.Scale(df, from_=-15, to=15, orient='horizontal',
                                     variable=self.speed_var,
                                     command=self._on_speed_slider)
        self.speed_scale.pack(side='left', fill='x', expand=True, padx=5)
        self.speed_lbl = ttk.Label(df, text="0", width=5, anchor='e')
        self.speed_lbl.pack(side='left')
        ttk.Button(df, text="Stop", width=5,
                   command=self._reset_speed).pack(side='left', padx=(5, 0))

        # Angle
        sf = ttk.Frame(ctrl)
        sf.pack(fill='x', pady=2)
        ttk.Label(sf, text="Angle:", width=7).pack(side='left')
        self.angle_var = tk.IntVar(value=0)
        self.angle_scale = ttk.Scale(sf, from_=-20, to=20, orient='horizontal',
                                     variable=self.angle_var,
                                     command=self._on_angle_slider)
        self.angle_scale.pack(side='left', fill='x', expand=True, padx=5)
        self.angle_lbl = ttk.Label(sf, text="0", width=5, anchor='e')
        self.angle_lbl.pack(side='left')
        ttk.Button(sf, text="Stop", width=5,
                   command=self._reset_angle).pack(side='left', padx=(5, 0))

        # ---- 키 입력 고정값 설정 ----
        setf = ttk.LabelFrame(self.root, text="키 입력 고정값 (W/S·A/D)", padding=8)
        setf.pack(fill='x', pady=5)

        ttk.Label(setf, text="구동 속도:").pack(side='left')
        self.hold_speed_var = tk.IntVar(value=self.hold_speed)
        sp = ttk.Spinbox(setf, from_=0, to=15, width=5,
                         textvariable=self.hold_speed_var,
                         command=self._sync_hold_values)
        sp.pack(side='left', padx=(3, 15))

        ttk.Label(setf, text="조향 각도:").pack(side='left')
        self.hold_angle_var = tk.IntVar(value=self.hold_angle)
        sa = ttk.Spinbox(setf, from_=0, to=20, width=5,
                         textvariable=self.hold_angle_var,
                         command=self._sync_hold_values)
        sa.pack(side='left', padx=(3, 0))

        # 직접 입력(타이핑) 후 Enter/포커스 아웃 시에도 반영
        for w in (sp, sa):
            w.bind('<Return>', lambda e: self._sync_hold_values())
            w.bind('<FocusOut>', lambda e: self._sync_hold_values())

        # ---- 오토모드 (rtabgui 경로 추종) ----
        af = ttk.LabelFrame(self.root, text="오토모드 (rtabgui 경로 추종)", padding=8)
        af.pack(fill='x', pady=5)

        row1 = ttk.Frame(af)
        row1.pack(fill='x')
        self.auto_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row1, text="Auto", variable=self.auto_var,
                        command=self._toggle_auto).pack(side='left')
        ttk.Label(row1, text="  속도:").pack(side='left')
        self.auto_speed_var = tk.IntVar(value=self.auto_speed)
        ttk.Spinbox(row1, from_=0, to=15, width=4, textvariable=self.auto_speed_var,
                    command=self._sync_auto_values).pack(side='left', padx=(2, 8))
        ttk.Label(row1, text="LAD(m):").pack(side='left')
        self.lad_var = tk.DoubleVar(value=self.look_ahead)
        ttk.Spinbox(row1, from_=0.1, to=5.0, increment=0.1, width=5,
                    textvariable=self.lad_var,
                    command=self._sync_auto_values).pack(side='left', padx=(2, 8))
        self.plan_status_label = ttk.Label(row1, text="○ 스트림 끊김", foreground="gray")
        self.plan_status_label.pack(side='left', padx=(8, 0))

        row2 = ttk.Frame(af)
        row2.pack(fill='x', pady=(4, 0))
        ttk.Label(row2, text="rtabgui:").pack(side='left')
        self.host_var = tk.StringVar(value=self.plan_host)
        ttk.Entry(row2, textvariable=self.host_var, width=12).pack(side='left', padx=(2, 4))
        ttk.Label(row2, text="port:").pack(side='left')
        self.plan_port_var = tk.IntVar(value=self.plan_port)
        ttk.Entry(row2, textvariable=self.plan_port_var, width=6).pack(side='left', padx=(2, 0))
        ttk.Button(row2, text="적용", width=5,
                   command=self._apply_plan_addr).pack(side='left', padx=(6, 0))

        # ---- ESTOP ----
        self.estop_btn = tk.Button(
            self.root, text="ALL STOP", font=('Arial', 14, 'bold'),
            bg='#ee3333', fg='white', activebackground='#cc0000',
            activeforeground='white', relief='raised', bd=3, height=2,
            command=self._all_stop)
        self.estop_btn.pack(fill='x', pady=5)

        # ---- 피드백 ----
        fb = ttk.LabelFrame(self.root, text="피드백 (Arduino → PC)", padding=8)
        fb.pack(fill='x', pady=5)

        grid = ttk.Frame(fb)
        grid.pack(fill='x')
        grid.columnconfigure((0, 1, 2), weight=1)

        self.enc_delta_lbl = ttk.Label(grid, text="Enc Delta: ---", anchor='w',
                                       font=('Consolas', 11))
        self.enc_delta_lbl.grid(row=0, column=0, sticky='w', padx=5)

        self.pot_lbl = ttk.Label(grid, text="Pot (A0): ---", anchor='w',
                                 font=('Consolas', 11))
        self.pot_lbl.grid(row=0, column=1, sticky='w', padx=5)

        self.enc_cum_lbl = ttk.Label(grid, text="Enc Total: ---", anchor='w',
                                     font=('Consolas', 11))
        self.enc_cum_lbl.grid(row=0, column=2, sticky='w', padx=5)

        # ---- 송신 미리보기 ----
        pv = ttk.LabelFrame(self.root, text="송신 명령", padding=8)
        pv.pack(fill='x', pady=5)
        self.cmd_lbl = ttk.Label(pv, text="---", font=('Consolas', 10))
        self.cmd_lbl.pack(anchor='w')

        # ---- 로그 ----
        lf = ttk.LabelFrame(self.root, text="로그", padding=5)
        lf.pack(fill='x', pady=5)
        self.log_text = tk.Text(lf, height=4, width=50, font=('Consolas', 9),
                                state='disabled')
        self.log_text.pack(fill='x')

        # ---- 키보드 도움말 ----
        hf = ttk.LabelFrame(self.root, text="키보드", padding=5)
        hf.pack(fill='x', pady=(5, 0))
        ttk.Label(hf, foreground="gray",
                  text="W/S 속도(1초 가속/감속) | A/D 조향 -15/+15 | "
                       "Space 즉시정지").pack()

        self._refresh_ports()

    def _append_log(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert('end', msg + '\n')
        self.log_text.see('end')
        # 최대 100줄 유지
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 100:
            self.log_text.delete('1.0', f'{lines - 100}.0')
        self.log_text.config(state='disabled')

    # ---- GUI 콜백 ----
    def _refresh_ports(self):
        ports = self.get_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def _on_speed_slider(self, v):
        # 슬라이더로 직접 지정한 값은 감속 대상이 아니므로 목표값도 함께 설정
        self.speed = int(float(v))
        self.target_speed = self.speed
        self.speed_lbl.config(text=str(self.speed))

    def _on_angle_slider(self, v):
        self.angle = int(float(v))
        self.angle_lbl.config(text=str(self.angle))

    def _reset_speed(self):
        # 즉시 정지 (감속 없이) — E-stop/Stop 버튼용
        self.speed = 0
        self.target_speed = 0
        self.speed_var.set(0)
        self.speed_lbl.config(text="0")

    def _reset_angle(self):
        self.angle = 0
        self.angle_var.set(0)
        self.angle_lbl.config(text="0")

    def _all_stop(self):
        # 비상정지/Space는 오토모드도 해제 (수동 인계)
        if self.auto_var.get():
            self.auto_var.set(False)
            self._toggle_auto()
        self._reset_speed()
        self._reset_angle()

    def _update_display(self):
        self.enc_delta_lbl.config(text=f"Enc Delta: {self.enc_delta}")
        self.pot_lbl.config(text=f"Pot (A0): {self.pot_value}")
        self.enc_cum_lbl.config(text=f"Enc Total: {self.enc_cumulative}")
        self.cmd_lbl.config(text=f"V{int(round(self.speed))},A{self.angle}")

    # ================================================================
    # 키보드 제어
    # ================================================================
    def _bind_keys(self):
        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self._all_stop()
            return
        if key in self.keys_pressed:
            return
        self.keys_pressed.add(key)
        self._apply_held_keys()

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.keys_pressed.discard(key)
        self._apply_held_keys()

    def _sync_hold_values(self):
        """Spinbox 입력값을 고정값(hold_speed/hold_angle)에 반영."""
        try:
            self.hold_speed = max(0, min(15, int(self.hold_speed_var.get())))
        except (tk.TclError, ValueError):
            pass
        try:
            self.hold_angle = max(0, min(20, int(self.hold_angle_var.get())))
        except (tk.TclError, ValueError):
            pass
        # 키를 누른 채로 값을 바꾸면 즉시 새 목표값 반영
        self._apply_held_keys()

    def _apply_held_keys(self):
        """현재 눌려 있는 키로부터 목표 속도/조향을 재계산."""
        if self.auto_mode:   # 오토모드 중에는 수동 W/S/A/D 무시
            return
        keys = self.keys_pressed

        # 속도 목표: W=+, S=- (둘 다 누르면 0). 실제 속도는 _ramp_speed가 서서히 이동
        fwd = bool(keys & {'w', 'up'})
        back = bool(keys & {'s', 'down'})
        if fwd == back:
            self.target_speed = 0
        elif fwd:
            self.target_speed = self.hold_speed
        else:
            self.target_speed = -self.hold_speed

        # 조향: A=-15, D=+15 (둘 다 누르면 0) — 즉시 적용
        left = bool(keys & {'a', 'left'})
        right = bool(keys & {'d', 'right'})
        if left == right:
            self.angle = 0
        elif left:
            self.angle = -self.hold_angle
        else:
            self.angle = self.hold_angle

        self._update_controls_display()

    def _update_controls_display(self):
        """속도/조향 슬라이더·라벨을 현재 값으로 갱신."""
        spd = int(round(self.speed))
        self.speed_var.set(spd)
        self.speed_lbl.config(text=str(spd))
        self.angle_var.set(self.angle)
        self.angle_lbl.config(text=str(self.angle))

    # ================================================================
    # 오토모드 (rtabgui 경로 추종, pure-pursuit)
    # ================================================================
    def _toggle_auto(self):
        self.auto_mode = self.auto_var.get()
        if self.auto_mode:
            self._sync_auto_values()
            self.keys_pressed.clear()
            self._append_log("오토모드 ON (경로 추종)")
        else:
            self.target_speed = 0   # 수동 복귀, 정지
            self.angle = 0
            self._update_controls_display()
            self._append_log("오토모드 OFF")

    def _sync_auto_values(self):
        try:
            self.auto_speed = max(0, min(15, int(self.auto_speed_var.get())))
        except (tk.TclError, ValueError):
            pass
        try:
            self.look_ahead = float(self.lad_var.get())
        except (tk.TclError, ValueError):
            pass

    def _apply_auto_control(self):
        """매 틱 호출: 최신 경로+pose로 조향 재계산 (새 데이터 오기 전엔 직전 값 유지)."""
        steer = self._compute_auto_steering()
        if steer is None:
            # 유효 경로/pose 없음 또는 오래됨(GUI 멈춤) -> 직진 + 정지 (안전)
            self.angle = 0
            self.target_speed = 0
        else:
            self.angle = steer
            self.target_speed = self.auto_speed
        self._update_controls_display()

    def _compute_auto_steering(self):
        """pure-pursuit: LAD 앞 경로점으로의 heading error를 -20..20 조향으로 매핑.
        반환 None = 유효한 명령 없음(정지)."""
        with self.plan_lock:
            pose = self.latest_pose
            path = list(self.latest_path)
            age = time.time() - self.last_plan_time
        if pose is None or len(path) < 2:
            return None
        if age > self.PLAN_STALE_SEC:    # 워치독: 스트림 끊김 -> 정지
            return None

        px, py, yaw = pose
        lad = max(0.05, self.look_ahead)
        # 1) 로봇과 가장 가까운 경로점(현재 진행 위치)을 먼저 찾는다.
        nearest, best_d = 0, float('inf')
        for i, (wx, wy) in enumerate(path):
            d = math.hypot(wx - px, wy - py)
            if d < best_d:
                best_d, nearest = d, i
        # 2) 거기서부터 앞으로 LAD 이상 떨어진 첫 점을 lookahead로.
        #    (인덱스 0부터 스캔하면 전진 후 뒤쪽 점에 latch되어 ±180°->포화 조향이 됨)
        idx = -1
        for i in range(nearest, len(path)):
            wx, wy = path[i]
            if math.hypot(wx - px, wy - py) >= lad:
                idx = i
                break
        if idx < 0:
            idx = len(path) - 1          # 남은 경로가 LAD 안 -> 목표를 겨냥

        tx, ty = path[idx]
        alpha = math.atan2(ty - py, tx - px) - yaw
        while alpha > math.pi:
            alpha -= 2.0 * math.pi
        while alpha < -math.pi:
            alpha += 2.0 * math.pi
        # 좌(alpha>0, CCW) -> 음수(수동 A=좌와 일치). LAD로 부드러움 조절.
        cmd = -(math.degrees(alpha) / self.FULL_LOCK_DEG) * 20.0
        return max(-20, min(20, int(round(cmd))))

    # ---- rtabgui 경로+pose 스트림 클라이언트 ----
    def _apply_plan_addr(self):
        try:
            self.plan_host = self.host_var.get().strip() or "127.0.0.1"
            self.plan_port = int(self.plan_port_var.get())
        except (tk.TclError, ValueError):
            return
        self.plan_connected = False      # 새 주소로 강제 재연결
        try:
            if self.plan_sock:
                self.plan_sock.close()
        except OSError:
            pass

    def _start_plan_client(self):
        threading.Thread(target=self._plan_loop, daemon=True).start()

    def _plan_loop(self):
        buf = ""
        while self.running:
            if not self.plan_connected:
                try:
                    s = socket.create_connection((self.plan_host, self.plan_port), timeout=2.0)
                    s.settimeout(1.0)
                    self.plan_sock = s
                    self.plan_connected = True
                    buf = ""
                    self._set_plan_status(True)
                except OSError:
                    time.sleep(1.0)
                    continue
            try:
                data = self.plan_sock.recv(8192)
                if not data:
                    raise OSError("closed")
                buf += data.decode('ascii', errors='ignore')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    self._parse_plan_line(line.strip())
            except socket.timeout:
                continue
            except OSError:
                self.plan_connected = False
                try:
                    if self.plan_sock:
                        self.plan_sock.close()
                except OSError:
                    pass
                self.plan_sock = None
                self._set_plan_status(False)
                time.sleep(1.0)

    def _parse_plan_line(self, line):
        if not line:
            return
        parts = line.split()
        try:
            if parts[0] == 'P' and len(parts) >= 4:
                x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
                with self.plan_lock:
                    self.latest_pose = (x, y, yaw)
                    self.last_plan_time = time.time()
            elif parts[0] == 'W' and len(parts) >= 3:
                nums = [float(v) for v in parts[1:]]
                pts = [(nums[i], nums[i + 1]) for i in range(0, len(nums) - 1, 2)]
                with self.plan_lock:
                    self.latest_path = pts
                    self.last_plan_time = time.time()
        except (ValueError, IndexError):
            pass

    def _set_plan_status(self, connected):
        def upd():
            if connected:
                self.plan_status_label.config(text="● 스트림 연결됨", foreground="green")
            else:
                self.plan_status_label.config(text="○ 스트림 끊김", foreground="gray")
        try:
            self.root.after(0, upd)
        except RuntimeError:
            pass

    # ================================================================
    # 실행
    # ================================================================
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._start_plan_client()
        self.root.after(self.SEND_INTERVAL_MS, self._tick)
        self.root.mainloop()

    def _on_close(self):
        self.running = False
        self.disconnect()
        try:
            if self.plan_sock:
                self.plan_sock.close()
        except OSError:
            pass
        self.root.destroy()


if __name__ == '__main__':
    app = DebugController()
    app.run()
