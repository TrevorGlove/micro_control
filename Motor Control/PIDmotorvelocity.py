"""
@TrevorGlove https://github.com/TrevorGlove/motor_control
Developed for Raspberry Pi Pico and ESP32
"""

from machine import Pin, ADC, Timer
from L298N import L298N

# -----Pin Declarations----

# L298N and encoder
Pin_L298N = {"IN2": 6, "IN1": 7, "ENA": 8}
Pin_Encoder = {"A": 4, "B": 5}

# Potentiometer
PinPOT = 26

# -----Variable Declarations----

counter = 0  # Encoder pulses
ppr = 486  # Total pulses per revolution

vA1 = 0  # vA(k-1)
vB1 = 0  # vB(k-1)

motor_dir = False  # F for backward direction and T for forward direction
rpm = 0  # Revolutions per minute

# PID
sp = 0
c = [0.0, 0.0, 0.0]  # [c, c(k-1), c(k-2)]
e = [0.0, 0.0, 0.0]  # [e, e(k-1), e(k-2)]
K = [8.05, 4.71, 0.001]  # [Kp, Ki, Kd]
Ts = 0.1

q = [K[0] + K[2] / Ts, -K[0] + K[1] * Ts - 2 * K[2] / Ts, K[2] / Ts]

# Auxiliary values for PID transformation from A/D
# q =  [1.458, 0.5521, -0.9039]

# -----Interrupt Function Definitions----


def interrupt(pin):
    global counter, vA, vA1, vA2, vB, motor_dir
    vA = A.value()  # Encoder Pin A value
    if not vA1 and vA:
        vB = B.value()  # Encoder Pin B value
        if vB == (False and motor_dir):
            motor_dir = False
        elif vB == (True and (not motor_dir)):
            motor_dir = True
    vA1 = vA

    counter += 1 if not motor_dir else -1


def PID(timer):
    global rpm, counter, pot, sp, c, e, ppr, Ts, q
    
    rpm = int(counter * 60 / (ppr * Ts))
    counter = 0

    sp = int(pot * 90 / 65535 - 45)
    e[0] = sp - rpm

    # -----PID Control----

    c[0] = c[1] + q[0] * e[0] + q[1] * e[1] + q[2] * e[2]

    c[1] = c[0]
    c[2] = c[1]
    e[1] = e[0]
    e[2] = e[1]
    
    print(("Sp", sp, "Rpm", rpm))
    
# -----Pin Configuration----

# ADC Reading
adc = ADC(PinPOT)

# Interrupt on Pin A
A = Pin(Pin_Encoder["A"], Pin.IN)
B = Pin(Pin_Encoder["B"], Pin.IN)
A.irq(trigger=Pin.IRQ_RISING, handler=interrupt)

# Motor Control
motor1 = L298N(Pin_L298N["IN1"], Pin_L298N["IN2"], Pin_L298N["ENA"])

# Timer Interrupt
tim = Timer()
tim.init(period=int(Ts * 1000), mode=Timer.PERIODIC, callback=PID)

while True:
    
    pot = adc.read_u16()
    c = [min(max(value, -500), 500) for value in c]
    motor1.speed(int(c[0] * 65535 / 500))
