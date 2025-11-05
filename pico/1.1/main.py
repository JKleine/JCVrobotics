import machine # type: ignore
import time
import sys

# --- Config ---
SERVO_PIN_1 = 2 #Track Left
SERVO_PIN_2 = 3 #Track Right
SERVO_PIN_3 = 4 #Head X axis (pan)
SERVO_PIN_4 = 5 #Head Y axis (tilt)
LED_PIN = 6     #GoBilda LED Headlight (PWM control)

PWM_FREQ = 50  # 50 Hz = 20ms period
PERIOD_US = 20000
# UART
UART_ID = 0
BAUD_RATE = 115200
uart = machine.UART(UART_ID, baudrate=BAUD_RATE, tx=machine.Pin(0), rx=machine.Pin(1))

# LED for visual feedback
led = machine.Pin("LED", machine.Pin.OUT)

# Store the current position in microseconds (us) for the servos
current_head_x_us = 1500
current_head_y_us = 1500 

# Startup blinks 3 times
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
    return int(1000 + (degrees / 300.0) * 1000)

def led_percent_to_us(percent):
    """Converts 0-100% brightness to 1000-2000us pulse width."""
    # Clamp the value between 0 and 100
    clamped_percent = max(0.0, min(100.0, float(percent)))
    # Linear interpolation: 1000us (at 0%) to 2000us (at 100%)
    return int(LED_OFF + (clamped_percent / 100.0) * (LED_ON - LED_OFF))

# Init PWM
servo1 = machine.PWM(machine.Pin(SERVO_PIN_1)) #Track Left
servo2 = machine.PWM(machine.Pin(SERVO_PIN_2)) #Track Right
servo3 = machine.PWM(machine.Pin(SERVO_PIN_3)) #Head X axis (pan)
servo4 = machine.PWM(machine.Pin(SERVO_PIN_4)) #Head Y axis (tilt)
led_headlight = machine.PWM(machine.Pin(LED_PIN)) #LED Headlight
servo1.freq(PWM_FREQ)
servo2.freq(PWM_FREQ)
servo3.freq(PWM_FREQ)
servo4.freq(PWM_FREQ)
led_headlight.freq(PWM_FREQ)

# Predefined values
STOP = 1500
FWD = 1800
REV = 1200

# LED Headlight values (in microseconds)
LED_OFF = 1000
LED_ON = 2000
LED_MID = 1500

# Head servo limits (set after calibration)
HEAD_X_MIN = 1300
HEAD_X_MAX = 1900
HEAD_Y_MIN = 1400
HEAD_Y_MAX = 1869

# Set initial stop/center position
servo1.duty_u16(us_to_duty(STOP))
servo2.duty_u16(us_to_duty(STOP))
servo3.duty_u16(us_to_duty(current_head_x_us))
servo4.duty_u16(us_to_duty(current_head_y_us))
led_headlight.duty_u16(us_to_duty(LED_OFF))

# Send initial readiness message back to the Pi 5
uart.write("Pico Ready\n")

while True:
    # Check if data is available on the UART
    if uart.any():
        try:
            # .readline() waits for the newline character ('\n') sent by the Pi 5.
            raw_cmd = uart.readline()
            if raw_cmd is None:
                continue

            cmd = raw_cmd.decode('utf-8').strip().lower()

        except Exception as e:
            # Use uart.write() instead of print() to send error message back
            uart.write(f"ERROR: Decoding failed - {e}\n") 
            continue

        parts = cmd.split()
        
        if not cmd:
            continue

        if cmd == "f":
            blink_led()
            servo1.duty_u16(us_to_duty(REV))
            servo2.duty_u16(us_to_duty(FWD))
            uart.write("OK: Forward\n")

        elif cmd == "r":
            blink_led()
            servo1.duty_u16(us_to_duty(FWD))
            servo2.duty_u16(us_to_duty(REV))
            uart.write("OK: Reverse\n")

        elif cmd == "lt":
            blink_led()
            servo1.duty_u16(us_to_duty(REV))
            servo2.duty_u16(us_to_duty(REV))
            uart.write("OK: Left turn\n")

        elif cmd == "rt":
            blink_led()
            servo1.duty_u16(us_to_duty(FWD))
            servo2.duty_u16(us_to_duty(FWD))
            uart.write("OK: Right turn\n")

        elif cmd == "s":
            blink_led()
            servo1.duty_u16(us_to_duty(STOP))
            servo2.duty_u16(us_to_duty(STOP))
            uart.write("OK: Stop\n")
        # LED Headlight brightness commandS

        elif cmd == "lon": # LED On (Full)
            blink_led()
            led_headlight.duty_u16(us_to_duty(LED_ON))
            uart.write("OK: Headlight ON\n")

        elif cmd == "loff": # LED Off
            blink_led()
            led_headlight.duty_u16(us_to_duty(LED_OFF))
            uart.write("OK: Headlight OFF\n")
            
        elif len(parts) >= 2 and parts[0] == "lset":
            # Sets brightness to a percentage (0-100)
            try:
                percent_val = float(parts[1])
                if 0 <= percent_val <= 100:
                    us_value = led_percent_to_us(percent_val)
                    led_headlight.duty_u16(us_to_duty(us_value))
                    blink_led()
                    uart.write(f"OK: Headlight set to {percent_val}%\n")
                else:
                    uart.write("ERROR: LED percent must be 0-100\n")
            except ValueError:
                uart.write("ERROR: Invalid LED percent - use number 0-100\n")

        # --- Direct coordinate commands (hx/hy) ---
        elif len(parts) >= 2 and parts[0] == "hx":
            try:
                degrees = float(parts[1])
                # Note: Pi 5 is sending a degree value (e.g., 190)
                if 90 <= degrees <= 270: 
                    us_value = degrees_to_us(degrees)
                    current_head_x_us = max(HEAD_X_MIN, min(HEAD_X_MAX, us_value))
                    servo3.duty_u16(us_to_duty(current_head_x_us))
                    blink_led()
                    uart.write(f"OK: Head X set to {degrees}°\n") 
                else:
                    uart.write("ERROR: Head X degrees must be 90-270\n") 
            except ValueError:
                uart.write("ERROR: Invalid head X position - use number\n") 

        elif len(parts) >= 2 and parts[0] == "hy":
            try:
                degrees = float(parts[1])
                if 120 <= degrees <= 260:
                    us_value = degrees_to_us(degrees)
                    current_head_y_us = max(HEAD_Y_MIN, min(HEAD_Y_MAX, us_value))
                    servo4.duty_u16(us_to_duty(current_head_y_us))
                    blink_led()
                    uart.write(f"OK: Head Y set to {degrees}°\n") 
                else:
                    uart.write("ERROR: Head Y degrees must be 120-260\n") 
            except ValueError:
                uart.write("ERROR: Invalid head Y position - use number\n") 
                
        else:
            # Respond with error for invalid commands
            uart.write(f"ERROR: Invalid command '{cmd}'\n") 

    time.sleep(0.01) # Small delay to yield to other tasks
