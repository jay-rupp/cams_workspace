"""
CAMS Bot — OPEN-LOOP MOTOR TEST
Pico MicroPython  |  MPU-6050 (I2C)  |  L298N dual H-bridge

D commands drive motors directly via PWM — both PID loops bypassed.
Use this to verify motor response before closed-loop tuning.

Serial commands (from Pi 4 over USB, 115200 baud)
  D<float>,T<float>\n  — direct PWM to motors (scaled by DRIVE_SCALE)
  S\n                  — stop motors
  K\n                  — kill switch
  R\n                  — resume
"""

import sys
import uselect
from machine import Pin, PWM, I2C
import struct, time, math

# ── Serial I/O ────────────────────────────────────────────────────────────────
_poll = uselect.poll()
_poll.register(sys.stdin, uselect.POLLIN)

# ── Motor config ──────────────────────────────────────────────────────────────
MAX_OUTPUT  = 65535
MIN_PWM     = 500.0
DRIVE_SCALE = 42000.0
TURN_MAX    = 0.3
turn_offset = 0.0

# ── Motor driver ──────────────────────────────────────────────────────────────
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

# ── Onboard LED ───────────────────────────────────────────────────────────────
led = Pin(25, Pin.OUT)
_led_ctr = 0
LOOP_HZ  = 100

def heartbeat():
    global _led_ctr
    _led_ctr += 1
    if _led_ctr >= LOOP_HZ // 2:
        led.toggle()
        _led_ctr = 0

# ── Serial command parser ─────────────────────────────────────────────────────
killed      = False
_serial_buf = '' 

def parse_serial():
    global killed, _serial_buf, turn_offset

    if not _poll.poll(0):
        return

    chunk = sys.stdin.readline()   # was sys.stdin.read(256)
    if chunk:
        _serial_buf += chunk       # no .encode()

    while '\n' in _serial_buf:     # was b'\n'
        line, _serial_buf = _serial_buf.split('\n', 1)   # was b'\n'
        line = line.strip()    

        if line == 'K':
            killed = True
            stop_motors()
            sys.stdout.write("KILLED\n")

        elif line == 'R':
            killed = False
            stop_motors()
            sys.stdout.write("RESUMED\n")

        elif line == 'S':
            stop_motors()
            turn_offset = 0.0
            sys.stdout.write("STOP\n")

        elif line.startswith('D') and ',' in line:
            try:
                parts = line[1:].split(',')
                d_val = float(parts[0])
                t_val = float(parts[1].lstrip('T'))
                turn_offset = max(-TURN_MAX, min(TURN_MAX, t_val))
                pwm_out = d_val * DRIVE_SCALE
                if not killed:
                    drive(motor_l, pwm_out * (1.0 - turn_offset))
                    drive(motor_r, pwm_out * (1.0 + turn_offset))
                sys.stdout.write(f"D_OK pwm={pwm_out:.0f}\n")
            except Exception as e:
                sys.stdout.write(f"D_ERR {e}\n")

# ── Startup ───────────────────────────────────────────────────────────────────
stop_motors()
sys.stdout.write("CAMS Bot open-loop test — send D<drive>,T<turn> or K/S/R\n")

# ── Main loop ─────────────────────────────────────────────────────────────────
DT        = 1.0 / LOOP_HZ
next_tick = time.ticks_us()
interval  = int(DT * 1_000_000)

while True:
    now = time.ticks_us()
    if time.ticks_diff(now, next_tick) >= 0:
        next_tick = time.ticks_add(next_tick, interval)
        parse_serial()
        heartbeat()