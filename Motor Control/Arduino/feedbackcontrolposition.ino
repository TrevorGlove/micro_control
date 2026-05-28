"""
@TrevorGlove https://github.com/TrevorGlove/motor_control
Developed with Raspberry Pi Pico
"""

from machine import Pin, ADC, Timer
from L298N import L298N
from FuzzyLib import Fuzzy, Cut, Proyect, Defuzzy

# -----Pin Declarations----

# L298N and encoder
Pin_L298N = {'IN2': 6, 'IN1': 7, 'ENA': 8}
Pin_Encoder = {'A': 4, 'B': 5}

# Serial ADC Pin
PinSerial = 28

# -----Variable Declarations----

counter = 0     # Encoder pulses (for speed)
counter_p = 0   # Encoder pulses (for position)
ppr = 486       # Total pulses per revolution

vA1 = 0        # vA(k-1)
vB1 = 0        # vB(k-1)

motor_dir = False  # F for backward direction and T for forward direction
deg = 0         # Motor rotation degrees

# Fuzzy

x = (0, 0)  # (error, speed)

e_u = (-500, 500)        # Error universe (degrees)
v_u = (-500, 500)        # Speed universe (degrees/s)
dc_u = (-65535, 65535)   # Duty cycle universe (bits)

# Input membership functions

FNL = (-500, -500, -300, -15)  # Far Negative Error
CNE = (-300, -50, 0)           # Close Negative Error
ZE = (-1, 0, 1)                # Zero Error
CPE = (0, 50, 300)             # Close Positive Error
FPE = (15, 300, 500, 500)      # Far Positive Error

E = [FNL, CNE, ZE, CPE, FPE]

NS = (-500, -500, -100, -1)  # Negative Speed
ZS = (-2, 0, 2)              # Zero Speed
PS = (1, 100, 500, 500)      # Positive Speed
V = [NS, ZS, PS]


# Output membership functions

HNDC = (-65535, -65535, -43690, -500)   # High Negative Duty Cycle
LNDC = (-10000, -500, 1000)             # Low Negative Duty Cycle
ZDC = (-500, 0, 500)                    # Zero Duty Cycle
LPDC = (100, 500, 10000)                # Low Positive Duty Cycle
HPDC = (500, 43690, 65535, 65535)       # High Positive Duty Cycle

DC = [HNDC, LNDC, ZDC, LPDC, HPDC]

# Fuzzy Rules

R = [(FNL, NS, HNDC),    # r1
     (FNL, ZS, HNDC),    # r2
     (FNL, PS, HNDC),    # r3
     (CNE, NS, LNDC),    # r4
     (CNE, ZS, LNDC),    # r5
     (CNE, PS, LNDC),    # r6
     ( ZE, NS, LPDC),    # r7
     ( ZE, ZS,  ZDC),    # r8
     ( ZE, PS, LNDC),    # r9
     (CPE, NS, LPDC),    # r10
     (CPE, ZS, LPDC),    # r11
     (CPE, PS, LPDC),    # r12
     (FPE, NS, HPDC),    # r13
     (FPE, ZS, HPDC),    # r14
     (FPE, PS, HPDC)]    # r15

# Control Variables

sp = 0     # Setpoint (degrees)
c = 0.0    # Control Variable (bits)
e = 0.0    # Error (degrees)
Ts = 0.1

# -----Interrupt Function Definitions----

def interrupt(pin):
    global counter, vA, vA1, vA2, vB, motor_dir
    vA = A.value()        # Encoder Pin A value
    if not vA1 and vA:
        vB = B.value()    # Encoder Pin B value
        if vB == (False and motor_dir):
            motor_dir = False
        elif vB == (True and (not motor_dir)):
            motor_dir = True
    vA1 = vA

    counter += 1 if not motor_dir else -1
    
def Fuzzyficator(timer):
    global deg, counter, pot, sp, c, e, ppr, Ts, x, dc_u

    # -----Values for Input Vector------
    deg += counter*360/ppr           # Position
    vl = counter*360/(ppr*Ts)        # Speed
    counter = 0
    sp = pot*540/65535 - 270         # Setpoint
    e = sp - deg                     # Error

    # -----Fuzzy Control------
    
    print(("Sp", int(sp), "Deg", int(deg)))    
    
    x = (e, vl)                        # Input Vector
    Val = Fuzzy(R, x, (E, V))          # Fuzzification
    Lines_cut = Proyect(Val)           # Projection of values in Speed Universe
    Trapezoids = Cut(Lines_cut, Val)   # Clipped membership functions
    c = Defuzzy(Trapezoids, dc_u, 40)  # Defuzzification


# -----Pin Configuration----

# ADC Reading
adc = ADC(PinSerial)

# Interrupt on Pin A
A = Pin(Pin_Encoder['A'], Pin.IN)
B = Pin(Pin_Encoder['B'], Pin.IN)
A.irq(trigger=Pin.IRQ_RISING, handler=interrupt)

# Motor Control
motor1 = L298N(Pin_L298N['IN1'], Pin_L298N['IN2'], Pin_L298N['ENA'])

# Timer Interrupt
tim = Timer()
tim.init(period=int(Ts * 1000), mode=Timer.PERIODIC, callback=Fuzzyficator)


while True:
    pot = adc.read_u16()
    motor1.speed(c)
