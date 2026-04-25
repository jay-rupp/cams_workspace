
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
  S\n                           — stop (smooth ramp-down)
  K\n                           — kill switch: stop motors and zero state
  R\n                           — resume from kill

Drive model
  Joystick D value: lean offset for balance setpoint
  Joystick T value: differential PWM for turning
  Right trigger B value: forward throttle (sets velocity setpoint)

Motor deadband compensation (smooth)
------------------------------------
Measured: L298N + FIT0186 motors don't start spinning until ~30000 PWM
under load. Without compensation, the PID issues commands the motors
ignore until output gets large, then slams.

This file uses a SMOOTH ramp instead of an abrupt step at MIN_CMD:
- output near zero          → near-zero motor PWM (no jolt)
- output up to TRANSITION   → offset ramps linearly from 0 → MOTOR_OFFSET
- output above TRANSITION   → full MOTOR_OFFSET added

This avoids the "instant 30k step at zero crossing" that causes 10Hz
limit-cycle oscillation around the setpoint.
"""

from machine import Pin, PWM, I2C, UART
import struct, time, math

# ── Inner loop (balance) tuning ───────────────────────────────────────────────
SETPOINT       =  -1.0    # base tilt target (degrees)
KP             =  1000.0
KI             =  0.0
KD             =  700.0
LOOP_HZ        =  100
DEADBAND       =  0.3    # degrees
MAX_OUTPUT     =  65535
ALPHA          =  0.98    # complementary filter
D_FILTER_ALPHA =  0.3     # derivative low-pass
OUTPUT_FILTER  =  0.3

DT = 1.0 / LOOP_HZ

INTEGRAL_ZONE  = 2.0
INTEGRAL_CLAMP = 8.0

# ── Motor deadband feedforward (smooth) ──────────────────────────────────────
# MIN_CMD: below this absolute output value, motor is fully off (true zero)
# MOTOR_OFFSET: full offset added at and above TRANSITION
# TRANSITION: PID output value at which the offset becomes fully active.
#   Below TRANSITION, offset scales linearly from 0 → MOTOR_OFFSET.
#   This avoids an abrupt PWM step at zero-crossing that causes oscillation.
MIN_CMD        = 100
MOTOR_OFFSET   = 30000
TRANSITION     = 2800

# ── Drive scaling ─────────────────────────────────────────────────────────────
MAX_DRIVE_ANGLE = 1.5       # degrees of lean at full stick
VEL_DRIVE_SCALE = 5500.0    # full trigger → ±3700 ticks/sec velocity target
TURN_SCALE      = 10000.0   # ±0.3 max turn → ±4500 PWM differential

# Ramp rates — how fast the "current" value chases the "target" value
VEL_RAMP_RATE   = 4000.0
DRIVE_RAMP_RATE = 8.0

# Kick — overcomes stiction when starting from rest
KICK_PWM      = 12000.0
KICK_DECAY    = 0.88
KICK_MIN      = 200.0
TRIGGER_EDGE  = 0.2

# ── Outer loop (velocity) tuning ─────────────────────────────────────────────
VEL_KP = 0.002
VEL_KI = 0.0006
VEL_KD = 0.0
VEL_LOOP_DIV        =  10
VEL_SETPOINT        =  0.0
VEL_SETPOINT_TARGET =  0.0
VEL_INTEGRAL_CLAMP  =  1.5
VEL_MAX_OFFSET      =  2.0
VEL_D_FILTER        =  0.08

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
        print(f"tilt={angle:+6.2f}  out={output:+6.0f}  "
              f"vel={vel:+7.1f}  vsp={VEL_SETPOINT:+6.0f}  voff={vel_offset:+5.2f}  "
              f"doff={drive_offset:+5.2f}  boost={boost:.2f}  "
              f"L#={enc_left.ticks:+8d}  R#={enc_right.ticks:+8d}  "
              f"kick={kick_pwm:+6.0f}")

# ── MPU-6050 ──────────────────────────────────────────────────────────────────
MPU_ADDR = 0x68
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

def mpu_init():
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')
    i2c.writeto_mem(MPU_ADDR, 0x1C, b'\x00')
    i2c.writeto_mem(MPU_ADDR, 0x1B, b'\x08')

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

# ── L298N with smooth deadband feedforward ───────────────────────────────────
def make_motor(pwm_pin, in1_pin, in2_pin):
    pwm = PWM(Pin(pwm_pin)); pwm.freq(20_000)
    in1 = Pin(in1_pin, Pin.OUT)
    in2 = Pin(in2_pin, Pin.OUT)
    return pwm, in1, in2

def drive(motor, speed):
    """Drive with smooth deadband compensation.

    The offset ramps in linearly between MIN_CMD and TRANSITION, so the
    motor PWM doesn't have an abrupt step at zero-crossing. Above
    TRANSITION the full MOTOR_OFFSET is applied.
    """
    pwm, in1, in2 = motor
    spd = int(speed)
    abs_spd = abs(spd)

    if abs_spd < MIN_CMD:
        in1.off(); in2.off(); pwm.duty_u16(0)
        return

    # Smooth ramp of the offset
    if abs_spd < TRANSITION:
        offset = MOTOR_OFFSET * (abs_spd / TRANSITION)
    else:
        offset = MOTOR_OFFSET

    actual = min(MAX_OUTPUT, abs_spd + int(offset))

    if spd > 0:
        in1.on();  in2.off(); pwm.duty_u16(actual)
    else:
        in1.off(); in2.on();  pwm.duty_u16(actual)

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
_d_buf = [0.0] * 5
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

    _d_buf[_d_idx] = raw_d
    _d_idx = (_d_idx + 1) % len(_d_buf)
    filtered_d = sum(_d_buf) / len(_d_buf)

    output = KP * error + KI * integral + KD * filtered_d
    return max(-MAX_OUTPUT, min(MAX_OUTPUT, output))

# ── Outer PID (velocity) ─────────────────────────────────────────────────────
vel_integral   = 0.0
vel_prev_error = 0.0
vel_filtered_d = 0.0
vel_offset     = 0.0
prev_ticks_l   = 0
prev_ticks_r   = 0
avg_velocity   = 0.0

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

# ── Ramp helper ───────────────────────────────────────────────────────────────
def ramp_toward(current, target, rate, dt):
    max_step = rate * dt
    if current < target:
        return min(current + max_step, target)
    elif current > target:
        return max(current - max_step, target)
    return current

# ── Serial command parser ─────────────────────────────────────────────────────
killed              = False
drive_offset        = 0.0
drive_offset_target = 0.0
boost_val           = 0.0
turn_cmd            = 0.0
kick_pwm            = 0.0
prev_trigger_armed  = False

def parse_serial():
    global killed, _serial_buf, integral, prev_angle
    global vel_integral, vel_prev_error, vel_filtered_d, vel_offset
    global drive_offset, drive_offset_target, boost_val, turn_cmd
    global VEL_SETPOINT, VEL_SETPOINT_TARGET
    global kick_pwm, prev_trigger_armed

    if not _uart.any():
        return

    _serial_buf += _uart.read(_uart.any())

    while b'\n' in _serial_buf:
        line, _serial_buf = _serial_buf.split(b'\n', 1)
        line = line.strip().decode('utf-8', 'ignore')

        if line == 'K':
            killed              = True
            drive_offset        = 0.0
            drive_offset_target = 0.0
            boost_val           = 0.0
            turn_cmd            = 0.0
            VEL_SETPOINT        = 0.0
            VEL_SETPOINT_TARGET = 0.0
            integral            = 0.0
            vel_integral        = 0.0
            vel_offset          = 0.0
            vel_filtered_d      = 0.0
            kick_pwm            = 0.0
            prev_trigger_armed  = False
            stop_motors()
            print("CMD: KILL")

        elif line == 'R':
            killed              = False
            drive_offset        = 0.0
            drive_offset_target = 0.0
            boost_val           = 0.0
            turn_cmd            = 0.0
            VEL_SETPOINT        = 0.0
            VEL_SETPOINT_TARGET = 0.0
            integral            = 0.0
            prev_angle          = tilt_angle
            vel_integral        = 0.0
            vel_prev_error      = 0.0
            vel_filtered_d      = 0.0
            vel_offset          = 0.0
            kick_pwm            = 0.0
            prev_trigger_armed  = False
            enc_left.reset()
            enc_right.reset()
            print("CMD: RESUME")

        elif line == 'S':
            drive_offset_target = 0.0
            boost_val           = 0.0
            turn_cmd            = 0.0
            VEL_SETPOINT_TARGET = 0.0
            prev_trigger_armed  = False
            print("CMD: STOP")

        elif line.startswith('D'):
            try:
                rest = line[1:]
                d_str, rest2 = rest.split(',T')
                t_str, b_str = rest2.split(',B')
                joy_val      = float(d_str)
                boost_val    = float(b_str)
                turn_cmd     = float(t_str)

                drive_offset_target = joy_val  * MAX_DRIVE_ANGLE
                VEL_SETPOINT_TARGET = boost_val * VEL_DRIVE_SCALE

                trigger_armed = boost_val > TRIGGER_EDGE
                if trigger_armed and not prev_trigger_armed:
                    kick_pwm = KICK_PWM
                prev_trigger_armed = trigger_armed
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

print(f"CAMS Bot — smooth deadband (offset={MOTOR_OFFSET}, transition={TRANSITION})")
print(f"  KP={KP}  KD={KD}  SETPOINT={SETPOINT}°")
print("Send K to kill, R to resume")

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

        VEL_SETPOINT = ramp_toward(VEL_SETPOINT, VEL_SETPOINT_TARGET, VEL_RAMP_RATE,   DT)
        drive_offset = ramp_toward(drive_offset, drive_offset_target, DRIVE_RAMP_RATE, DT)

        kick_pwm *= KICK_DECAY
        if abs(kick_pwm) < KICK_MIN:
            kick_pwm = 0.0

        ax, ay, az, gx, gy, gz = read_raw()
        angle = update_angle(ax, ay, az, gy)

        if killed:
            stop_motors()
        else:
            loop_ctr += 1
            if loop_ctr >= VEL_LOOP_DIV:
                loop_ctr = 0
                velocity_step()

            effective_setpoint = SETPOINT + vel_offset + drive_offset
            output = pid_step(angle, effective_setpoint)
            _prev_output = OUTPUT_FILTER * output + (1.0 - OUTPUT_FILTER) * _prev_output

            turn_pwm = turn_cmd * TURN_SCALE
            drive(motor_l, _prev_output - turn_pwm + kick_pwm)
            drive(motor_r, _prev_output + turn_pwm + kick_pwm)

        maybe_print(angle, gyro_rate,
                    output if not killed else 0,
                    avg_velocity, vel_offset,
                    drive_offset, boost_val, turn_cmd)
        heartbeat()