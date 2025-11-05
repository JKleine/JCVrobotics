import machine # type: ignore
import time
import sys

# --- Config ---
SERVO_PIN_1 = 2 #Track Left
SERVO_PIN_2 = 3 #Track Right
SERVO_PIN_3 = 4 #Head X axis (pan)
SERVO_PIN_4 = 5 #Head Y axis (tilt)
PWM_FREQ = 50  # 50 Hz = 20ms period
PERIOD_US = 20000
# UART
UART_ID = 0
BAUD_RATE = 115200
uart = machine.UART(UART_ID, baudrate=BAUD_RATE, tx=machine.Pin(0), rx=machine.Pin(1))

# LED for visual feedback
led = machine.Pin("LED", machine.Pin.OUT)

# Startup blink - proves script is running
for i in range(3):
    led.on()
    time.sleep(0.3)
    led.off()
    time.sleep(0.3)

# Blinks each time a command is registered    
def blink_led():
    """Quick blink to show command received"""
    led.on()
    time.sleep(0.1)
    led.off()

def us_to_duty(us):
    return int(us * 65535 / PERIOD_US)

def degrees_to_us(degrees):
    # Convert 0-300 degrees to 1000-2000μs range
    # Adjust these values after calibration
    return int(1000 + (degrees / 300.0) * 1000)

# Init PWM
servo1 = machine.PWM(machine.Pin(SERVO_PIN_1)) #Track Left
servo2 = machine.PWM(machine.Pin(SERVO_PIN_2)) #Track Right
servo3 = machine.PWM(machine.Pin(SERVO_PIN_3)) #Head X axis (pan)
servo4 = machine.PWM(machine.Pin(SERVO_PIN_4)) #Head Y axis (tilt)
servo1.freq(PWM_FREQ)
servo2.freq(PWM_FREQ)
servo3.freq(PWM_FREQ)
servo4.freq(PWM_FREQ)

# Predefined values
STOP = 1500
FWD = 1800
REV = 1200

# Head servo limits (set after calibration)
HEAD_X_MIN = 1300  # Update after testing
HEAD_X_MAX = 1900
HEAD_Y_MIN = 1400
HEAD_Y_MAX = 1869

print("Commands: f=forward, r=reverse, lt=left, rt=right, s=stop")
print("Head commands: hx [degrees], hy [degrees]")

while True:
    cmd = input("Enter command: ").strip().lower()
    parts = cmd.split()  # Split command into parts

    if cmd == "f":
        # Both tracks forward (one servo reversed in software)
        blink_led()
        servo1.duty_u16(us_to_duty(FWD))    # Left track forward
        servo2.duty_u16(us_to_duty(REV))    # Right track forward (reversed)
        print("Forward...")

    elif cmd == "r":
        # Both tracks reverse
        blink_led()
        servo1.duty_u16(us_to_duty(REV))    # Left track reverse
        servo2.duty_u16(us_to_duty(FWD))    # Right track reverse (reversed)
        print("Reverse...")

    elif cmd == "lt":
        # Left turn: left track reverse, right track forward
        blink_led()
        servo1.duty_u16(us_to_duty(REV))    # Left track reverse
        servo2.duty_u16(us_to_duty(REV))    # Right track forward (reversed)
        print("Left turn...")

    elif cmd == "rt":
        # Right turn: left track forward, right track reverse
        blink_led()
        servo1.duty_u16(us_to_duty(FWD))    # Left track forward
        servo2.duty_u16(us_to_duty(FWD))    # Right track reverse (reversed)
        print("Right turn...")

    elif cmd == "s":
        # Stop both tracks
        blink_led()
        servo1.duty_u16(us_to_duty(STOP))
        servo2.duty_u16(us_to_duty(STOP))
        print("Stop...")

    elif len(parts) >= 2 and parts[0] == "hx":
        # Head X axis position (looking straight forward should be 180)
        try:
            degrees = float(parts[1])
            if 90 <= degrees <= 270:
                us_value = degrees_to_us(degrees)
                # Apply safety limits
                us_value = max(HEAD_X_MIN, min(HEAD_X_MAX, us_value))
                servo3.duty_u16(us_to_duty(us_value))
                blink_led()
                print(f"Head X to {degrees}° ({us_value}μs)")
            else:
                print("Head X degrees must be 90-270")
        except ValueError:
            print("Invalid head X position - use number")

    elif len(parts) >= 2 and parts[0] == "hy":
        # Head Y axis position (top of head parallel with ground should be 180)
        try:
            degrees = float(parts[1])
            if 120 <= degrees <= 260:
                us_value = degrees_to_us(degrees)
                # Apply safety limits
                us_value = max(HEAD_Y_MIN, min(HEAD_Y_MAX, us_value))
                servo4.duty_u16(us_to_duty(us_value))
                blink_led()
                print(f"Head Y to {degrees}° ({us_value}μs)")
            else:
                print("Head Y degrees must be 120-260")
        except ValueError:
            print("Invalid head Y position - use number")

    else:
        print("Invalid command")
        print("Available: f, r, lt, rt, s, hx [degrees], hy [degrees]")
