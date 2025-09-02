import machine
import rp2
import time
from machine import Pin

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink():
    wrap_target()
    set(pins, 1) [31]   # set pin high, delay 32 cycles
    set(pins, 0) [31]   # set pin low, delay 32 cycles
    wrap()

# Choose pin 25 (onboard LED)
STEP_PIN = 11
STEP_PIN_1 = 6
STEP_PIN_2 = 19
STEP_PIN_3 = 14

end_stop = Pin(4, Pin.IN, Pin.PULL_UP)

DIR_PIN = Pin(10, Pin.OUT)
DIR_PIN_1 = Pin(5, Pin.OUT)
DIR_PIN_2 = Pin(28, Pin.OUT)
DIR_PIN_3 = Pin(13, Pin.OUT)


EN_PIN = Pin(12, Pin.OUT)
EN_PIN_1 = Pin(7, Pin.OUT)
EN_PIN_2 = Pin(2, Pin.OUT)
EN_PIN_3 = Pin(15, Pin.OUT)

# Create state machine
sm = rp2.StateMachine(0, blink, freq=100_000, set_base=machine.Pin(STEP_PIN))
sm1 = rp2.StateMachine(1, blink, freq=100_000, set_base=machine.Pin(STEP_PIN_1))
sm2 = rp2.StateMachine(2, blink, freq=100_000, set_base=machine.Pin(STEP_PIN_2))
sm3 = rp2.StateMachine(3, blink, freq=100_000, set_base=machine.Pin(STEP_PIN_3))


DIR_PIN.value(1)
DIR_PIN_1.value(1)
DIR_PIN_2.value(1)
DIR_PIN_3.value(1)

EN_PIN.value(0)
EN_PIN_1.value(0)
EN_PIN_2.value(0)
EN_PIN_3.value(0)


print("Led started")
# Start it

sm.active(1)
sm1.active(1)
sm2.active(1)
sm3.active(1)

#time.sleep(5)
print("Led Finished")

while True:
    print('end_stop', end_stop.value())
    if end_stop.value() == 0:
        sm.active(0)
        sm1.active(0)
        sm2.active(0)
        sm3.active(0)
    time.sleep_ms(10)
    
    