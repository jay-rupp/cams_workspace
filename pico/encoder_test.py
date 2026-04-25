# encoder_test.py — check raw encoder signals by spinning wheels by hand
from machine import Pin
import time

print()
print("=" * 50)
print("ENCODER TEST — spin wheels by hand")
print("A/B values should toggle between 0 and 1")
print("=" * 50)

a12 = Pin(12, Pin.IN, Pin.PULL_UP)  # Left A
b13 = Pin(13, Pin.IN, Pin.PULL_UP)  # Left B
a10 = Pin(10, Pin.IN, Pin.PULL_UP)  # Right A
b11 = Pin(11, Pin.IN, Pin.PULL_UP)  # Right B

# Also count transitions via IRQ — so you can tell if IRQs would fire
left_ticks  = 0
right_ticks = 0

def left_cb(pin):
    global left_ticks
    left_ticks += 1

def right_cb(pin):
    global right_ticks
    right_ticks += 1

a12.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_cb)
a10.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_cb)

try:
    while True:
        print(f"LEFT  A={a12.value()} B={b13.value()}  ticks={left_ticks:6d}    "
              f"RIGHT A={a10.value()} B={b11.value()}  ticks={right_ticks:6d}")
        time.sleep_ms(100)
except Exception as e:
    print("ERROR:", e)