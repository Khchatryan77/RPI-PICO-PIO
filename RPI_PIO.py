import rp2
import utime
from machine import Pin, UART
from register import REGISTORS


uart1 = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))

uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) 


@rp2.asm_pio()
def blink():
    wrap_target()
    set(pins, 1) [31]   # set pin high, delay 32 cycles
    set(pins, 0) [31]   # set pin low, delay 32 cycles
    wrap()


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():

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



SLAVE_ADDR_1 = 0x00
SLAVE_ADDR_2 = 0x01
SLAVE_ADDR_3 = 0x02
SLAVE_ADDR_4 = 0x03

Running = True

FAN1 = Pin(17, Pin.OUT)

sm ={}

buffer = []
    
Motors = {
    "Motor_X": {
        "step_pin": Pin(11, Pin.OUT),
        "dir_pin": Pin(10, Pin.OUT),
        "en_pin": Pin(12, Pin.OUT),
        "End_stop_X": Pin(4, Pin.IN, Pin.PULL_UP),
        "sm_id": 0
    },
    "Motor_Y": {
        "step_pin": Pin(6, Pin.OUT),
        "dir_pin": Pin(5, Pin.OUT),
        "en_pin": Pin(7, Pin.OUT),
        "End_stop_Y": Pin(3, Pin.IN, Pin.PULL_UP),
        "sm_id": 1
    },
    "Motor_Z": {
        "step_pin": Pin(19, Pin.OUT),
        "dir_pin": Pin(28, Pin.OUT),
        "en_pin": Pin(2, Pin.OUT),
        "End_stop_Z": Pin(14, Pin.IN, Pin.PULL_UP),
        "sm_id": 2
    },
    "Motor_E": {
        "step_pin": Pin(14, Pin.OUT),
        "dir_pin": Pin(13, Pin.OUT),
        "en_pin": Pin(15, Pin.OUT),
        "End_stop_E": Pin(4, Pin.IN, Pin.PULL_UP),
        "sm_id": 3
    }
}

Motors["Motor_X"]["en_pin"].on()
Motors["Motor_Y"]["en_pin"].on()
Motors["Motor_Z"]["en_pin"].on()
Motors["Motor_E"]["en_pin"].on()

sm4 = rp2.StateMachine(4, blink, freq=100_000, in_base=Motors["Motor_X"]["End_stop_X"])
sm5 = rp2.StateMachine(5, blink, freq=100_000, in_base=Motors["Motor_Y"]["End_stop_Y"])

def end_stop_handler(pin):
    for sm_id in sm:
        sm[sm_id].active(0)
        
    Motors["Motor_X"]["en_pin"].value(1)
    Motors["Motor_Y"]["en_pin"].value(1)
    Motors["Motor_Z"]["en_pin"].value(1)
    Motors["Motor_E"]["en_pin"].value(1)
    
    global Running
    Running= False
    
    
def uart0_handler(uart0):
    if uart0.any():  # check if there is data
        data = uart0.read().decode().strip()  # read all available data
        buffer.append(data)
        print("Received:", data)
        return data


# Start it
def move_motor(motor_id, steps, dir_val, freq = 400_000):
    
    step_pin = motor_id["step_pin"]
    dir_pin = motor_id["dir_pin"]
    en_pin = motor_id["en_pin"]
    sm_id = motor_id["sm_id"]
    
    dir_pin.value(dir_val)
    en_pin.value(0)
    sm[sm_id] = rp2.StateMachine(sm_id, stepper, freq=freq, set_base=step_pin)
    sm[sm_id].active(1)
    sm[sm_id].put(steps)
    
    
def crc8(datagram, initial_value=0):
    crc = initial_value
    for byte in datagram:
        for _ in range(0, 8):
            if (crc >> 7) ^ (byte & 0x01):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            # Shift to next bit
            byte = byte >> 1
    return crc


def response_uart():
    if uart1.any():
        for i in range(10):
            response = uart1.read(4)
            if response is not None and len(response) >= 2:
                if response[1] == 255:
                    response2 = uart1.read(4)
                    if response2 is not None:
                        print('response', response + response2)
                        if response[2] == REGISTORS['IFCNT']:
                            print('IFCNT', response2[2])
                    else:
                        pass
                        # print('response2 is None')
                else:
                    pass
                    # print('Its Echo')
            else:
                pass
                # print('response is None or too short')
    else:
        print("Invalid read response:")
    utime.sleep_us(50)


def read_register(slave, reg):
    sync = 0x05
    addr = slave
    packet = [sync, addr, reg]
    crc = crc8(packet)
    uart1.write(bytearray(packet + [crc]))
    time.sleep_us(140)  # 140


def write_register(slave, reg, value):
    sync = 0x05
    addr = slave
    reg_write = reg | 0x80

    data = [(value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF]

    packet = [sync, addr, reg_write] + data
    print('packet', packet)
    crc = crc8(packet)

    uart1.write(bytearray(packet + [crc]))
    utime.sleep_us(500)

    response_uart()

#uart0.irq(handler=uart0_handler, trigger=uart0.RX_ANY)
uart0.irq(trigger = UART.IRQ_RXIDLE, handler = uart0_handler)

Motors["Motor_X"]["End_stop_X"].irq(trigger=Pin.IRQ_FALLING, handler=end_stop_handler)

Motors["Motor_Y"]["End_stop_Y"].irq(trigger=Pin.IRQ_FALLING, handler=end_stop_handler)

write_register(SLAVE_ADDR_1, REGISTORS['GCONF'], 0b11001000)
write_register(SLAVE_ADDR_2, REGISTORS['GCONF'], 0b11001000)
write_register(SLAVE_ADDR_3, REGISTORS['GCONF'], 0b11001000)
write_register(SLAVE_ADDR_4, REGISTORS['GCONF'], 0b11001000)

write_register(SLAVE_ADDR_1, REGISTORS['CHOPCONF'], 0x15000053)
write_register(SLAVE_ADDR_2, REGISTORS['CHOPCONF'], 0x15000053)
write_register(SLAVE_ADDR_3, REGISTORS['CHOPCONF'], 0x15000053)
write_register(SLAVE_ADDR_4, REGISTORS['CHOPCONF'], 0x15000053)

FAN1.value(1)


while True:
    print('buffer', buffer)
    if buffer and buffer[0] == 'move_motors':
        
        move_motor(Motors["Motor_X"], 15000, 0, freq=400000)

        move_motor(Motors["Motor_Y"], 15000, 0, freq=400000)

        move_motor(Motors["Motor_Z"], 15000, 0, freq=400000)

        move_motor(Motors["Motor_E"], 15000, 0, freq=400000)
        
        buffer.pop(0)
    elif len(buffer) > 0:
        
        buffer.pop(0)
        
    else:
        pass

    utime.sleep_ms(10)
