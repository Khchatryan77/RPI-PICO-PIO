import rp2
import time
from machine import Pin, UART
from register import REGISTORS


uart1 = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))

uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) 

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)


def blink(): 
    pull()            # Get number of steps from CPU
    mov(x, osr)       # Move to X register
    label("loop")
    jmp(x_dec, "pulse")
    jmp("end")
    
    label("pulse")
    set(pins, 1) [31]  # Pulse HIGH
    set(pins, 0) [31]  # Pulse LOW
    jmp("loop")
    
    label("end")
    nop()              # End program
    
    '''
    wrap_target()
    set(pins, 1) [31]   # set pin high, delay 32 cycles
    set(pins, 0) [31]   # set pin low, delay 32 cycles
    wrap()
    '''
'''
sm = rp2.StateMachine(0, blink, freq=400_000, set_base=machine.Pin(STEP_PIN))
sm1 = rp2.StateMachine(1, blink, freq=400_000, set_base=machine.Pin(STEP_PIN_1))
sm2 = rp2.StateMachine(2, blink, freq=400_000, set_base=machine.Pin(STEP_PIN_2))
sm3 = rp2.StateMachine(3, blink, freq=400_000, set_base=machine.Pin(STEP_PIN_3))
sm4 = rp2.StateMachine(4, blink, freq=100_000, in_base=machine.Pin(end_stop))
'''

SLAVE_ADDR_1 = 0x00
SLAVE_ADDR_2 = 0x01
SLAVE_ADDR_3 = 0x02
SLAVE_ADDR_4 = 0x03

Running = True

Motors = {
    "Motor_X": {
        "step_pin": Pin(11, Pin.OUT),
        "dir_pin": Pin(10, Pin.OUT),
        "en_pin": Pin(12, Pin.OUT),
        "End_stop_X": Pin(4, Pin.OUT)
    },
    "Motor_Y": {
        "step_pin": Pin(6, Pin.OUT),
        "dir_pin": Pin(5, Pin.OUT),
        "en_pin": Pin(7, Pin.OUT),
        "End_stop_Y": Pin(3, Pin.OUT)
    },
    "Motor_Z": {
        "step_pin": Pin(19, Pin.OUT),
        "dir_pin": Pin(28, Pin.OUT),
        "en_pin": Pin(2, Pin.OUT),
        "End_stop_Z": Pin(14, Pin.OUT)
    },
    "Motor_E": {
        "step_pin": Pin(14, Pin.OUT),
        "dir_pin": Pin(13, Pin.OUT),
        "en_pin": Pin(15, Pin.OUT),
        "End_stop_E": Pin(4, Pin.OUT)
    }
}

Motors["Motor_X"]["en_pin"].on()
Motors["Motor_Y"]["en_pin"].on()
Motors["Motor_Z"]["en_pin"].on()

def end_stop_handler(pin):
    global Running
    Running= False
    
def uart0_handler():
    if uart0.any():  # check if there is data
        data = uart0.read().decode().strip()  # read all available data
        buffer.append(data)
        print("Received:", data)
        return data

    
# Choose pin 25 (onboard LED)
STEP_PIN = 11
STEP_PIN_1 = 6
STEP_PIN_2 = 19
STEP_PIN_3 = 14

end_stop = Pin(4, Pin.IN, Pin.PULL_UP)

print("Led started")

# Start it
def move_motor(motor_id, steps, dir_val, freq = 400_000):
    
    step_pin = motor_id["step_pin"]
    dir_pin = motor_id["dir_pin"]
    en_pin = motor_id["en_pin"]
    endstop = motor_id["End_stop_X"]
    
    dir_pin.value(dir_val)
    en_pin.value(0)
    sm = rp2.StateMachine(0, blink, freq=freq, set_base=machine.Pin(step_pin))
    sm.active(1)
    sm.put(steps)
    en_pin.value(1)
'''    
sm.active(1)
sm1.active(1)
sm2.active(1)
sm3.active(1)

sm.put(15000)
sm1.put(15000)
sm2.put(15000)
sm3.put(15000)
'''
#uart0.irq(handler=uart0_handler, trigger=uart0.RX_ANY)

move_motor(Motors["Motor_X"], 150, 0, freq = 400_000)

while True:
    print('end_stop', end_stop.value())
    if end_stop.value() == 0:
        sm.active(0)
        sm1.active(0)
        sm2.active(0)
        sm3.active(0)
    time.sleep_ms(10)
    
