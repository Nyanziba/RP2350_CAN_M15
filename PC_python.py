import argparse
import json
import threading
import time
from dataclasses import dataclass

import serial
from cmath import sin, cos
from serial.tools import list_ports
from pynput import keyboard


@dataclass
class MotorFeedback:
    motor_id: int
    angle: float
    velocity: float
    current: float
    fault: int
    mode: str
    pid_error: float
    pid_output: float

@dataclass
class CommandConfig:
    port: str
    baud: int = 115200
    tick_hz: float = 50.0          # 送信更新周期（Hz）
    step_rpm: float = 10.0          # 1tickあたりのRPM変化量
    max_rpm: float = 200.0         # RPMの上限（M15 -210~210なので余裕を持って200）
    deadman_s: float = 100        # この秒数キー入力がなければ0に戻す
    decay_step_rpm: float = 0.3    # デッドマン時の0戻し速度（step_rpm倍率）
    keepalive_hz: float = 10.0     # 変更がなくても送る周期（Hz）

class CalcWheel:
    @staticmethod
    def rpm_to_wheel_velocity(rpm: float, wheel_radius_m: float) -> float:
        # RPMをホイール周速度(m/s)に変換
        return (rpm * 2 * 3.141592653589793 * wheel_radius_m) / 60.0

    @staticmethod
    def wheel_velocity_to_rpm(velocity_m_s: float, wheel_radius_m: float) -> float:
        # ホイール周速度(m/s)をRPMに変換
        return (velocity_m_s * 60.0) / (2 * 3.141592653589793 * wheel_radius_m)

class CalcDifferentialDrive:
    @staticmethod
    def robot_velocity_to_wheel_velocities(v_m_s: float, omega_rad_s: float, wheel_base_m: float) -> tuple[float, float]:
        # ロボットの前進速度v(m/s)と角速度ω(rad/s)から左右ホイールの周速度(m/s)を計算
        v_r = v_m_s + (omega_rad_s * wheel_base_m / 2.0)
        v_l = v_m_s - (omega_rad_s * wheel_base_m / 2.0)
        return v_l, v_r

    @staticmethod
    def wheel_velocities_to_robot_velocity(v_l_m_s: float, v_r_m_s: float, wheel_base_m: float) -> tuple[float, float]:
        # 左右ホイールの周速度(m/s)からロボットの前進速度v(m/s)と角速度ω(rad/s)を計算
        v_m_s = (v_r_m_s + v_l_m_s) / 2.0
        omega_rad_s = (v_r_m_s - v_l_m_s) / wheel_base_m
        return v_m_s, omega_rad_s

class RobotOdometry:
    def __init__(self):
        #以下の書き方はPython 3.10以降で有効なユニオン型の記法
        #受け入れ可能なデータ型を|で区切って指定できる。
        self.last_motor1_feedback: MotorFeedback | None = None
        self.last_motor2_feedback: MotorFeedback | None = None
        self.motor1_feedback: MotorFeedback | None = None #MOtorFeedback型かNoneを受け入れる
        self.motor2_feedback: MotorFeedback | None = None
        self.position_x: float = 0.0  # ロボットのX座標(m)
        self.position_y: float = 0.0  # ロボットのY座標(m)
        self.orientation_theta: float = 0.0  # ロボットの向き(rad)
        self.linear_velocity: float = 0.0  # ロボットの前進速度(m/s)
        self.angular_velocity: float = 0.0  # ロボットの角速度
        self.last_update_time: float | None = None

    def update(self, motor1_fb: MotorFeedback, motor2_fb: MotorFeedback, wheel_radius_m: float, wheel_base_m: float):
        current_time = time.monotonic()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return

        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        v_l = CalcWheel.rpm_to_wheel_velocity(motor1_fb.velocity, wheel_radius_m)
        v_r = CalcWheel.rpm_to_wheel_velocity(motor2_fb.velocity, wheel_radius_m)

        v_m_s, omega_rad_s = CalcDifferentialDrive.wheel_velocities_to_robot_velocity(v_l, v_r, wheel_base_m)

        self.linear_velocity = v_m_s
        self.angular_velocity = omega_rad_s

        # オドメトリの更新
        if abs(omega_rad_s) < 1e-6:
            # 直進運動
            dx = v_m_s * dt * cos(self.orientation_theta)
            dy = v_m_s * dt * sin(self.orientation_theta)
            dtheta = 0.0
        else:
            # 回転運動を考慮した位置更新
            R = v_m_s / omega_rad_s
            dtheta = omega_rad_s * dt
            dx = R * (sin(self.orientation_theta + dtheta) - sin(self.orientation_theta))
            dy = -R * (cos(self.orientation_theta + dtheta) - cos(self.orientation_theta))

        self.position_x += dx
        self.position_y += dy
        self.orientation_theta = (self.orientation_theta + dtheta) % (2 * 3.141592653589793)

class KeyState:
    def __init__(self):
        self._lock = threading.Lock()
        self._pressed = set()
        self._quit = False
        self._emergency = False
        self._zero = False

    def set_pressed(self, k, down: bool):
        with self._lock:
            if down:
                self._pressed.add(k)
            else:
                self._pressed.discard(k)

    def snapshot(self):
        with self._lock:
            return set(self._pressed), self._quit, self._emergency, self._zero

    def request_quit(self):
        with self._lock:
            self._quit = True

    def pulse_emergency(self):
        with self._lock:
            self._emergency = True

    def consume_emergency(self) -> bool:
        with self._lock:
            v = self._emergency
            self._emergency = False
            return v

    def pulse_zero(self):
        with self._lock:
            self._zero = True

    def consume_zero(self) -> bool:
        with self._lock:
            v = self._zero
            self._zero = False
            return v


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def list_serial_ports():
    pts = list(list_ports.comports())
    if not pts:
        print("シリアルポートが見つからない。デバイス接続を確認しろ。")
        return
    print("利用可能なポート:")
    for p in pts:
        desc = f"{p.device}  ({p.description})"
        if p.hwid:
            desc += f"  [{p.hwid}]"
        print("  -", desc)


def serial_reader(ser: serial.Serial, stop_event: threading.Event):
    # デバイス→PC（フィードバックJSON行）を垂れ流し表示
    ser.timeout = 0.1
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if not line:
                continue
            s = line.decode("utf-8", errors="replace").strip()
            if not s:
                continue

            # フィードバックJSONを読めたら整形表示
            try:
                fb = json.loads(s)
                motor_id = fb.get("motor_id")
                angle = fb.get("angle")
                velocity = fb.get("velocity")
                current = fb.get("current")
                fault = fb.get("fault")
                mode = fb.get("mode")
                pid_error = fb.get("pid_error", 0)
                pid_output = fb.get("pid_output", 0.0)
                print(
                    f"pid_err={pid_error}rpm pid_out={pid_output:.2f}A"
                )
            except Exception:
                # JSONでなければそのまま
                print(f"[DEV] {s}")
        except Exception as e:
            print("[DEV] read error:", e)
            break

def make_current_cmd(i1: float, i2: float) -> str:
    # MicroPython側 parse_json_command が mode/value1/value2 を期待している :contentReference[oaicite:2]{index=2}
    return json.dumps({"mode": "current_loop", "value1": i1, "value2": i2}, separators=(",", ":")) + "\n"
def make_rpm_pid_cmd(rpm1: float, rpm2: float) -> str:
    # MicroPython側 parse_json_command が mode/value1/value2 を期待している
    return json.dumps({"mode": "rpm_pid", "value1": rpm1, "value2": rpm2}, separators=(",", ":")) + "\n"


def make_disable_cmd() -> str:
    return json.dumps({"mode": "disable"}, separators=(",", ":")) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="例: COM5, /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tick-hz", type=float, default=5.0)
    ap.add_argument("--step-rpm", type=float, default=10.0)
    ap.add_argument("--max-rpm", type=float, default=200.0)
    ap.add_argument("--deadman-s", type=float, default=100.0)
    ap.add_argument("--keepalive-hz", type=float, default=10.0)
    args = ap.parse_args()

    if not args.port:
        list_serial_ports()
        print("\n--port を指定して起動してください。例: python wasd_current_loop.py --port COM5")
        return

    cfg = CommandConfig(
        port=args.port,
        baud=args.baud,
        tick_hz=args.tick_hz,
        step_rpm=args.step_rpm,
        max_rpm=args.max_rpm,
        deadman_s=args.deadman_s,
        keepalive_hz=args.keepalive_hz,
        decay_step_rpm=0.3,
    )
    print(f"接続: {cfg.port} @ {cfg.baud}")
    print("操作: W/S=前後, A/D=旋回, X=ゼロ, Space=disable, ESC=終了")
    print("注意: このウィンドウにフォーカスがないとキーが取れない/OS権限が必要な場合がある。")

    ser = serial.Serial(cfg.port, cfg.baud)
    stop_event = threading.Event()
    t = threading.Thread(target=serial_reader, args=(ser, stop_event), daemon=True)
    t.start()
    # キー入力状態管理
    #キー入出力に関してはは2つのスレッドで、管理している。KeyStateクラスで状態を管理する
    ks = KeyState()
    # キー入力状態監視関数
    def on_press(key):
        try:
            if key == keyboard.Key.esc:
                ks.request_quit()
                return False
            if key == keyboard.Key.space:
                ks.pulse_emergency()
                return
            if hasattr(key, "char") and key.char:
                c = key.char.lower()
                if c in ("w", "a", "s", "d"):
                    ks.set_pressed(c, True)
                elif c == "x":
                    ks.pulse_zero()
        except Exception:
            pass

    def on_release(key):
        try:
            if hasattr(key, "char") and key.char:
                c = key.char.lower()
                if c in ("w", "a", "s", "d"):
                    ks.set_pressed(c, False)
        except Exception:
            pass

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()


    r1 = 0.0  # motor1 target RPM
    r2 = 0.0  # motor2 target RPM

    last_input_t = time.monotonic()
    last_sent = None
    keepalive_dt = 1.0 / max(cfg.keepalive_hz, 1e-6)
    last_keepalive_t = 0.0

    dt = 1.0 / cfg.tick_hz
    try:
        while True:
            pressed, want_quit, _, _ = ks.snapshot()
            if want_quit:
                break

            # 非常停止（disable）
            if ks.consume_emergency():
                ser.write(make_disable_cmd().encode("utf-8"))
                ser.flush()
                print("[PC] disable sent")
                r1, r2 = 0.0, 0.0
                last_sent = None
                last_input_t = time.monotonic()
                time.sleep(dt)
                continue

            # ゼロ
            if ks.consume_zero():
                r1, r2 = 0.0, 0.0
                last_sent = None
                last_input_t = time.monotonic()

            # 入力があればランプ更新
            any_key = bool(pressed)
            if any_key:
                last_input_t = time.monotonic()

                # W/S: 両輪同方向
                if "w" in pressed:
                    r1 += cfg.step_rpm
                    r2 += cfg.step_rpm
                if "s" in pressed:
                    r1 -= cfg.step_rpm
                    r2 -= cfg.step_rpm

                # A/D: 差動（旋回）
                if "a" in pressed:
                    r1 -= cfg.step_rpm
                    r2 += cfg.step_rpm
                if "d" in pressed:
                    r1 += cfg.step_rpm
                    r2 -= cfg.step_rpm

            # デッドマン：一定時間入力なしなら0へ戻す（暴走防止）
            if (time.monotonic() - last_input_t) > cfg.deadman_s:
                decay = cfg.step_rpm * cfg.decay_step_rpm
                if abs(r1) < decay:
                    r1 = 0.0
                else:
                    r1 -= decay if r1 > 0 else -decay
                if abs(r2) < decay:
                    r2 = 0.0
                else:
                    r2 -= decay if r2 > 0 else -decay

            r1 = clamp(r1, -cfg.max_rpm, cfg.max_rpm)
            r2 = clamp(r2, -cfg.max_rpm, cfg.max_rpm)

            # 変更があったら送る。変更がなくてもkeepaliveで送る（デバイス側の状態維持用）
            now = time.monotonic()
            cmd = make_rpm_pid_cmd(r1, -r2)  # RPM PIDコマンドを送信
            changed = (cmd != last_sent)
            keepalive_due = (now - last_keepalive_t) >= keepalive_dt

            if changed or keepalive_due:
                ser.write(cmd.encode("utf-8"))
                ser.flush()
                last_sent = cmd
                last_keepalive_t = now
                # print(f"[PC] sent r1={r1:.1f}rpm r2={r2:.1f}rpm")

            time.sleep(dt)

    finally:
        # 終了時はdisable（安全側）
        try:
            ser.write(make_disable_cmd().encode("utf-8"))
            ser.flush()
        except Exception:
            pass
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        print("終了。")


if __name__ == "__main__":
    main()
