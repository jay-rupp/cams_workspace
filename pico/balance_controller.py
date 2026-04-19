"""
CAMS Bot — cascaded PID balance controller
Pico MicroPython  |  MPU-6050 (I2C)  |  L298N dual H-bridge
Encoders: FIT0186 hall quadrature, 2797 ticks/rev

Wiring
------
MPU-6050
  SDA -> GP0   SCL -> GP1   VCC -> 3V3   GND -> GND

L298N  (Motor A = left, Motor B = right)
  ENA -> GP2  (PWM)   IN1 -> GP3   IN2 -> GP4
  ENB -> GP5  (PWM)   IN3 -> GP6   IN4 -> GP7

Encoders
  Left:   A -> GP12  B -> GP13   (inverted)
  Right:  A -> GP10  B -> GP11

UART1 (commands from Pi 4, 115200 baud)
  TX -> GP8   RX -> GP9

Serial commands
  D<float>,T<float>,B<float>\n  — drive, turn, boost (joystick ±1.0, trigger 0–1)
  S\n                           — stop
  K\n                           — kill switch: stop motors and zero state
  R\n                           — resume from kill

Drive model
  Joystick D value:
    1. Offsets SETPOINT by ±MAX_DRIVE_ANGLE degrees (balance loop leans)
    2. Sets VEL_SETPOINT to ±VEL_DRIVE_SCALE ticks/sec (velocity loop drives)
  Joystick T value: differential PWM between left/right motors for turning.
  Right trigger B value (0–1): multiplies balance output for extra speed.
"""

from machine import Pin, PWM, I2C, UART
import struct, time, math

# ── Inner loop (balance) tuning ───────────────────────────────────────────────
SETPOINT       =  0.7    # base tilt target (degrees)
KP             =  8700.0
KI             =  0.0
KD             =  2000.0
LOOP_HZ        =  100
DEADBAND       =  0.12   # degrees
MAX_OUTPUT     =  65535
MIN_PWM        =  1000.0
ALPHA          =  0.98   # complementary filter
D_FILTER_ALPHA =  0.3   # derivative low-pass
OUTPUT_FILTER = 0.3

DT = 1.0 / LOOP_HZ

INTEGRAL_ZONE  = 1.5
INTEGRAL_CLAMP = 8.0

# ── Drive scaling ─────────────────────────────────────────────────────────────
MAX_DRIVE_ANGLE = 1.7      # degrees of lean at full stick
VEL_DRIVE_SCALE = 3700.0   # full stick → ±3500 ticks/sec velocity target
TURN_SCALE      = 15000.0  # ±0.3 max turn → ±4500 PWM differential
TRIGGER_BOOST   = 2.5      # full trigger = 2.5x balance output (1.0 + 1.5)

# ── Outer loop (velocity) tuning ─────────────────────────────────────────────
VEL_KP             =  0.004
VEL_KI             =  0.002
VEL_KD             =  0.0
VEL_LOOP_DIV       =  10       # run every N inner loops (10 Hz)
VEL_SETPOINT       =  0.0     # target velocity in ticks/sec — set by D command
VEL_INTEGRAL_CLAMP =  3.0
VEL_MAX_OFFSET     =  8.0
VEL_D_FILTER       =  0.08

# ── Encoder config ────────────────────────────────────────────────────────────
TICKS_PER_REV = 2797

# ── Encoder class ─────────────────────────────────────────────────────────────
class Encoder:
    def __init__(self, pin_a, pin_b, invert=False):
        self.ticks = 0
        self.sign = -1 if invert else 1
        self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)

    def _cb(self, pin):
        if self.a.value() == self.b.value():
            self.ticks += self.sign
        else:
            self.ticks -= self.sign

    def reset(self):
        self.ticks = 0

# ── Onboard LED ───────────────────────────────────────────────────────────────
led = Pin(25, Pin.OUT)
_led_ctr = 0

def heartbeat():
    global _led_ctr
    _led_ctr += 1
    if _led_ctr >= LOOP_HZ // 2:
        led.toggle()
        _led_ctr = 0

# ── Console print ─────────────────────────────────────────────────────────────
print_enabled = True
_print_ctr    = 0
PRINT_EVERY   = 10  # ~10 Hz

def maybe_print(angle, gyro_rate, output, vel, vel_offset, drive_offset, boost, turn):
    global _print_ctr
    if not print_enabled:
        return
    _print_ctr += 1
    if _print_ctr >= PRINT_EVERY:
        _print_ctr = 0
        print(f"tilt={angle:+6.2f}  sp={SETPOINT+vel_offset+drive_offset:+5.2f}  "
              f"gyro={gyro_rate:+7.2f}  out={output:+6.0f}  "
              f"vel={vel:+7.1f}  voff={vel_offset:+5.2f}  "
              f"doff={drive_offset:+5.2f}  boost={boost:.2f}  turn={turn:+5.2f}")

# ── MPU-6050 ──────────────────────────────────────────────────────────────────
MPU_ADDR = 0x68
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

def mpu_init():
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')  # wake
    i2c.writeto_mem(MPU_ADDR, 0x1C, b'\x00')  # accel ±2 g
    i2c.writeto_mem(MPU_ADDR, 0x1B, b'\x08')  # gyro ±500 °/s

IMU_OFF = {'gx': -109, 'gy': 337, 'gz': 25}

def read_raw():
    d = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
    ax, ay, az, _, gx, gy, gz = struct.unpack('>hhhhhhh', d)
    return (ax, ay, az,
            gx - IMU_OFF['gx'],
            gy - IMU_OFF['gy'],
            gz - IMU_OFF['gz'])

# ── Complementary filter ──────────────────────────────────────────────────────
tilt_angle = 0.0
gyro_rate  = 0.0

def update_angle(ax, ay, az, gy):
    global tilt_angle, gyro_rate
    accel_angle = math.atan2(ay, az) * 57.2958
    gyro_rate   = gy / 65.5
    tilt_angle  = ALPHA * (tilt_angle + gyro_rate * DT) + (1 - ALPHA) * accel_angle
    return tilt_angle

# ── L298N ─────────────────────────────────────────────────────────────────────
def make_motor(pwm_pin, in1_pin, in2_pin):
    pwm = PWM(Pin(pwm_pin)); pwm.freq(20_000)
    in1 = Pin(in1_pin, Pin.OUT)
    in2 = Pin(in2_pin, Pin.OUT)
    return pwm, in1, in2

def drive(motor, speed):
    pwm, in1, in2 = motor
    spd = max(-MAX_OUTPUT, min(MAX_OUTPUT, int(speed)))
    if abs(spd) < MIN_PWM:
        in1.off(); in2.off(); pwm.duty_u16(0)
    elif spd > 0:
        in1.on();  in2.off(); pwm.duty_u16(spd)
    else:
        in1.off(); in2.on();  pwm.duty_u16(-spd)

def stop_motors():
    drive(motor_l, 0)
    drive(motor_r, 0)

motor_l = make_motor(2, 3, 4)
motor_r = make_motor(5, 6, 7)

# ── Encoders ──────────────────────────────────────────────────────────────────
enc_left  = Encoder(12, 13, invert=True)
enc_right = Encoder(10, 11)

# ── UART1 (commands from Pi 4) ────────────────────────────────────────────────
_uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
_serial_buf = b''

# ── PID state ─────────────────────────────────────────────────────────────────
_d_buf = [0.0] * 5  # last 5 samples
_d_idx = 0
integral    = 0.0
prev_angle  = 0.0

def pid_step(angle, setpoint):
    global integral, prev_angle, _d_idx
    error = setpoint - angle

    if abs(error) < DEADBAND:
        prev_angle = angle
        return 0

    integral *= 0.91
    if abs(error) < INTEGRAL_ZONE:
        integral += error * DT
        integral  = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))
    else:
        integral = 0.0

    raw_d = (prev_angle - angle) / DT
    prev_angle = angle

    # Moving average filter
    _d_buf[_d_idx] = raw_d
    _d_idx = (_d_idx + 1) % len(_d_buf)
    filtered_d = sum(_d_buf) / len(_d_buf)

    output = KP * error + KI * integral + KD * filtered_d
    return max(-MAX_OUTPUT, min(MAX_OUTPUT, output))

# ── Outer PID (velocity) ─────────────────────────────────────────────────────
vel_integral   = 0.0
vel_prev_error = 0.0
vel_filtered_d = 0.0
vel_offset     = 0.0      # angle offset fed to inner loop
prev_ticks_l   = 0
prev_ticks_r   = 0
avg_velocity   = 0.0      # ticks/sec, for telemetry

def velocity_step():
    global vel_integral, vel_prev_error, vel_filtered_d, vel_offset
    global prev_ticks_l, prev_ticks_r, avg_velocity

    tl = enc_left.ticks
    tr = enc_right.ticks
    dt_vel = VEL_LOOP_DIV * DT

    vel_l = (tl - prev_ticks_l) / dt_vel
    vel_r = (tr - prev_ticks_r) / dt_vel
    avg_velocity = (vel_l + vel_r) / 2.0

    prev_ticks_l = tl
    prev_ticks_r = tr

    error = VEL_SETPOINT - avg_velocity

    vel_integral += error * dt_vel
    vel_integral  = max(-VEL_INTEGRAL_CLAMP / VEL_KI if VEL_KI > 0 else 0,
                        min( VEL_INTEGRAL_CLAMP / VEL_KI if VEL_KI > 0 else 0,
                             vel_integral))

    raw_d          = (error - vel_prev_error) / dt_vel
    vel_filtered_d = VEL_D_FILTER * raw_d + (1.0 - VEL_D_FILTER) * vel_filtered_d
    vel_prev_error = error

    offset = VEL_KP * error + VEL_KI * vel_integral + VEL_KD * vel_filtered_d
    vel_offset = max(-VEL_MAX_OFFSET, min(VEL_MAX_OFFSET, offset))

# ── Serial command parser ─────────────────────────────────────────────────────
killed       = False
drive_offset = 0.0   # setpoint offset from joystick (degrees)
boost_val    = 0.0   # trigger boost 0.0–1.0
turn_cmd     = 0.0   # turn differential from joystick

def parse_serial():
    global killed, _serial_buf, integral, prev_angle
    global vel_integral, vel_prev_error, vel_filtered_d, vel_offset
    global drive_offset, boost_val, turn_cmd, VEL_SETPOINT

    if not _uart.any():
        return

    _serial_buf += _uart.read(_uart.any())

    while b'\n' in _serial_buf:
        line, _serial_buf = _serial_buf.split(b'\n', 1)
        line = line.strip().decode('utf-8', 'ignore')

        if line == 'K':
            killed         = True
            drive_offset   = 0.0
            boost_val      = 0.0
            turn_cmd       = 0.0
            VEL_SETPOINT   = 0.0
            integral       = 0.0
            vel_integral   = 0.0
            vel_offset     = 0.0
            vel_filtered_d = 0.0
            stop_motors()
            print("CMD: KILL")

        elif line == 'R':
            killed         = False
            drive_offset   = 0.0
            boost_val      = 0.0
            turn_cmd       = 0.0
            VEL_SETPOINT   = 0.0
            integral       = 0.0
            prev_angle     = tilt_angle
            vel_integral   = 0.0
            vel_prev_error = 0.0
            vel_filtered_d = 0.0
            vel_offset     = 0.0
            enc_left.reset()
            enc_right.reset()
            print("CMD: RESUME")

        elif line == 'S':
            drive_offset   = 0.0
            boost_val      = 0.0
            turn_cmd       = 0.0
            VEL_SETPOINT   = 0.0
            vel_integral   = 0.0
            vel_prev_error = 0.0
            vel_filtered_d = 0.0
            vel_offset     = 0.0
            print("CMD: STOP")

        elif line.startswith('D'):
            try:
                # Format: D<float>,T<float>,B<float>
                rest = line[1:]
                d_str, rest2 = rest.split(',T')
                t_str, b_str = rest2.split(',B')
                joy_val      = float(d_str)
                drive_offset = joy_val * MAX_DRIVE_ANGLE
                VEL_SETPOINT = joy_val * VEL_DRIVE_SCALE
                turn_cmd     = float(t_str)
                boost_val    = float(b_str)
                print(f"CMD: doff={drive_offset:+.2f}°  vsp={VEL_SETPOINT:+.0f}  "
                      f"turn={turn_cmd:+.3f}  boost={boost_val:.2f}")
            except Exception:
                print(f"CMD: parse error — '{line}'")

# ── Startup ───────────────────────────────────────────────────────────────────
time.sleep_ms(2000)
mpu_init()
time.sleep_ms(200)

print("Settling IMU...")
for _ in range(50):
    read_raw()
    time.sleep_ms(5)

prev_ticks_l = enc_left.ticks
prev_ticks_r = enc_right.ticks

print("CAMS Bot — angle+vel+boost+turn — send K to kill, R to resume")

# ── Main loop ─────────────────────────────────────────────────────────────────
next_tick = time.ticks_us()
interval  = int(DT * 1_000_000)
loop_ctr  = 0
_prev_output = 0.0
while True:
    now = time.ticks_us()
    if time.ticks_diff(now, next_tick) >= 0:
        next_tick = time.ticks_add(next_tick, interval)

        parse_serial()

        ax, ay, az, gx, gy, gz = read_raw()
        angle = update_angle(ax, ay, az, gy)

        if killed:
            stop_motors()
        else:
            # Outer velocity loop at 10 Hz
            loop_ctr += 1
            if loop_ctr >= VEL_LOOP_DIV:
                loop_ctr = 0
                velocity_step()

            # Inner balance loop at 100 Hz
            effective_setpoint = SETPOINT + vel_offset + drive_offset
            output = pid_step(angle, effective_setpoint)
            _prev_output = OUTPUT_FILTER * output + (1.0 - OUTPUT_FILTER) * _prev_output

            # Apply trigger boost — multiplies balance output
            final = _prev_output * (1.0 + boost_val * TRIGGER_BOOST)
            final = max(-MAX_OUTPUT, min(MAX_OUTPUT, final))

            # Apply turn differential
            turn_pwm = turn_cmd * TURN_SCALE
            drive(motor_l, final - turn_pwm)
            drive(motor_r, final + turn_pwm)

        maybe_print(angle, gyro_rate,
                    output if not killed else 0,
                    avg_velocity, vel_offset,
                    drive_offset, boost_val, turn_cmd)
        heartbeat()