from machine import Pin, PWM
pwm = PWM(Pin(2)); pwm.freq(20_000)
in1 = Pin(3, Pin.OUT)
in2 = Pin(4, Pin.OUT)

# Forward
in1.on(); in2.off(); pwm.duty_u16(30000)
# wait a few seconds, then:
# Reverse
in1.off(); in2.on(); pwm.duty_u16(30000)
