"""
Motor threshold test
====================
Sweeps PWM from low to high in steps, runs each level for 2 seconds,
prints the encoder tick rate so you can see at what PWM the wheels
actually start spinning.

Hold the robot in the air. Wheels should be free.

Watch the printed dL/dR per second values:
  - 0 means wheel didn't move during that step
  - small positive means wheel just barely turning
  - large positive means wheel spinning freely

Both motors driven forward together so direction is consistent.
"""

from machine import Pin, PWM
import time

# ── Hardware setup (same as main controller) ─────────────────────────────────
def make_motor(pwm_pin, in1_pin, in2_pin):
    pwm = PWM(Pin(pwm_pin)); pwm.freq(20_000)
    in1 = Pin(in1_pin, Pin.OUT)
    in2 = Pin(in2_pin, Pin.OUT)
    return pwm, in1, in2

def drive(motor, speed):
    pwm, in1, in2 = motor
    spd = max(-65535, min(65535, int(speed)))
    if spd == 0:
        in1.off(); in2.off(); pwm.duty_u16(0)
    elif spd > 0:
        in1.on();  in2.off(); pwm.duty_u16(spd)
    else:
        in1.off(); in2.on();  pwm.duty_u16(-spd)

motor_l = make_motor(2, 3, 4)
motor_r = make_motor(5, 6, 7)

# ── Encoders ──────────────────────────────────────────────────────────────────
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

enc_left  = Encoder(12, 13, invert=True)
enc_right = Encoder(10, 11)

# ── Test sequence ─────────────────────────────────────────────────────────────
PWM_LEVELS  = [3000, 5000, 7000, 10000, 15000, 20000, 30000, 45000]
STEP_SEC    = 2.0

print()
print("=" * 60)
print("MOTOR THRESHOLD TEST — hold robot in air, wheels free")
print("=" * 60)
print(f"Will run each PWM level for {STEP_SEC}s and report tick counts")
print("Starting in 3 seconds...")
time.sleep(3)

results = []

try:
    for pwm in PWM_LEVELS:
        # Reset tick counters
        prev_l = enc_left.ticks
        prev_r = enc_right.ticks

        # Drive both motors at this PWM
        drive(motor_l, pwm)
        drive(motor_r, pwm)

        # Wait
        time.sleep(STEP_SEC)

        # Measure
        dl = enc_left.ticks  - prev_l
        dr = enc_right.ticks - prev_r
        rate_l = dl / STEP_SEC
        rate_r = dr / STEP_SEC

        moving = "MOVING" if (abs(dl) > 10 or abs(dr) > 10) else "stalled"
        print(f"PWM={pwm:>6d}  ({pwm*100/65535:>4.1f}% duty)  "
              f"L: {dl:>+6d} ticks ({rate_l:>+7.1f} t/s)  "
              f"R: {dr:>+6d} ticks ({rate_r:>+7.1f} t/s)  [{moving}]")

        results.append((pwm, dl, dr))

        # Brief pause to settle before next step
        drive(motor_l, 0)
        drive(motor_r, 0)
        time.sleep(0.3)

finally:
    drive(motor_l, 0)
    drive(motor_r, 0)

print()
print("=" * 60)
print("TEST COMPLETE")
print("=" * 60)
print()
print("Find the lowest PWM in the table above where ticks went from 0 to nonzero.")
print("That's the motor's true start threshold under no load.")
print()
print("If wheels barely move until 15000+, that's the problem we've been chasing.")
print("If wheels move at 3000-5000, motors are fine and the issue is elsewhere.")
