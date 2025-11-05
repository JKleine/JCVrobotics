import machine # type: ignore
import time

# ============================================================================
# CONFIGURATION
# ============================================================================

# Pin assignments
SERVO_PIN_1 = 2  # Track Left
SERVO_PIN_2 = 3  # Track Right
SERVO_PIN_3 = 4  # Head X (pan)
SERVO_PIN_4 = 5  # Head Y (tilt)
LED_PIN = 6      # Headlight

# UART config
UART_ID = 0
BAUD_RATE = 115200
# Initialize UART on pins 0 (TX) and 1 (RX)
uart = machine.UART(UART_ID, baudrate=BAUD_RATE, tx=machine.Pin(0), rx=machine.Pin(1))

# PWM config
PWM_FREQ = 50
PERIOD_US = 20000

# Servo limits (calibrated)
HEAD_X_MIN = 1300
HEAD_X_MAX = 1900
HEAD_Y_MIN = 1400
HEAD_Y_MAX = 1869

# Track servo values (in microseconds)
STOP = 1500
FWD = 1800
REV = 1200

# LED levels (microseconds)
LED_OFF = 1000
LED_LOW = 1400
LED_HIGH = 2000

# Head movement parameters
HEAD_MOVE_SPEED = 10    # degrees per update cycle (adjustable for smoothness)
HEAD_UPDATE_RATE_MS = 20 # 50Hz update rate (20ms)

# Manual override timeout (milliseconds)
MANUAL_OVERRIDE_TIMEOUT_MS = 500  # Manual commands block automation for 0.5s

# Status reporting interval
STATUS_REPORT_INTERVAL_MS = 1000 # Send status every 1 second if enabled

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def us_to_duty(us):
    """Convert microseconds to PWM duty cycle (0-65535)."""
    return int(us * 65535 / PERIOD_US)

def degrees_to_us(degrees):
    """Convert 0-300 degrees to 1000-2000μs PWM range."""
    # Assumes a 0-300 degree scale mapped to standard 1000-2000us range
    return int(1000 + (degrees / 300.0) * 1000)

def clamp(value, min_val, max_val):
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))

def interpolate(current, target, step):
    """Move current toward target by step amount."""
    if abs(target - current) <= step:
        return target
    return current + step if target > current else current - step

# ============================================================================
# ROBOT STATE CLASS
# ============================================================================

class RobotState:
    """Maintains current and target state for all actuators."""
    
    def __init__(self):
        # Current state (actual hardware positions)
        self.track_left = STOP
        self.track_right = STOP
        self.head_x_degrees = 180.0  # Center
        self.head_y_degrees = 180.0  # Center
        self.led_level = 0  # 0=off, 1=low, 2=high
        
        # Target state (commanded positions)
        self.target_track_left = STOP
        self.target_track_right = STOP
        self.target_head_x = 180.0
        self.target_head_y = 180.0
        self.target_led_level = 0
        
        # Control flags
        self.manual_override_until_ms = 0  # Timestamp when manual override expires
        self.is_manual_control = False
        self.reporting_enabled = False     # Flag to enable/disable status reporting
        
    def set_manual_override(self):
        """Mark that manual control is active."""
        self.manual_override_until_ms = time.ticks_add(time.ticks_ms(), MANUAL_OVERRIDE_TIMEOUT_MS)
        self.is_manual_control = True
        
    def check_override_expired(self):
        """Check if manual override has expired."""
        # Update is_manual_control based on the timeout
        if time.ticks_diff(time.ticks_ms(), self.manual_override_until_ms) > 0:
            self.is_manual_control = False
        return not self.is_manual_control # Return True if override is EXPIRED
    
    def update_targets(self, **kwargs):
        """Update target state from command. Accepts any combination of actuators."""
        for key, value in kwargs.items():
            if key == 'track_left':
                self.target_track_left = value
            elif key == 'track_right':
                self.target_track_right = value
            elif key == 'head_x':
                # Clamp head degrees between 90 (max left) and 270 (max right)
                self.target_head_x = clamp(float(value), 90.0, 270.0)
            elif key == 'head_y':
                # Clamp head degrees between 120 and 260
                self.target_head_y = clamp(float(value), 120.0, 260.0)
            elif key == 'led_level':
                self.target_led_level = clamp(int(value), 0, 2)
    
    def smooth_update(self):
        """Incrementally move current state toward targets (for smooth movement)."""
        # Tracks update immediately
        self.track_left = self.target_track_left
        self.track_right = self.target_track_right
        
        # Head moves smoothly toward target
        self.head_x_degrees = interpolate(
            self.head_x_degrees,
            self.target_head_x,
            HEAD_MOVE_SPEED
        )
        self.head_y_degrees = interpolate(
            self.head_y_degrees,
            self.target_head_y,
            HEAD_MOVE_SPEED
        )
        
        # LED updates immediately
        self.led_level = self.target_led_level

# ============================================================================
# HARDWARE CONTROL
# ============================================================================

class HardwareController:
    """Controls physical hardware (servos, LED)."""
    
    def __init__(self):
        # Initialize PWM for servos and LED
        self.servo1 = machine.PWM(machine.Pin(SERVO_PIN_1))
        self.servo2 = machine.PWM(machine.Pin(SERVO_PIN_2))
        self.servo3 = machine.PWM(machine.Pin(SERVO_PIN_3))
        self.servo4 = machine.PWM(machine.Pin(SERVO_PIN_4))
        self.led_headlight = machine.PWM(machine.Pin(LED_PIN))
        
        # Set PWM frequency
        for pwm in [self.servo1, self.servo2, self.servo3, self.servo4, self.led_headlight]:
            pwm.freq(PWM_FREQ)
        
        # Status LED (onboard Pico LED)
        self.status_led = machine.Pin("LED", machine.Pin.OUT)
        
        # Startup blink
        for _ in range(3):
            self.status_led.on()
            time.sleep(0.15)
            self.status_led.off()
            time.sleep(0.15)
    
    def apply_state(self, state):
        """Apply robot state to hardware."""
        # Apply track speeds (continuous rotation servos)
        self.servo1.duty_u16(us_to_duty(state.track_left))
        self.servo2.duty_u16(us_to_duty(state.track_right))
        
        # Apply head positions (standard servos)
        head_x_us = degrees_to_us(state.head_x_degrees)
        head_y_us = degrees_to_us(state.head_y_degrees)
        # Apply hardware-specific limits
        head_x_us = clamp(head_x_us, HEAD_X_MIN, HEAD_X_MAX)
        head_y_us = clamp(head_y_us, HEAD_Y_MIN, HEAD_Y_MAX)
        self.servo3.duty_u16(us_to_duty(head_x_us))
        self.servo4.duty_u16(us_to_duty(head_y_us))
        
        # Apply LED level
        led_values = {0: LED_OFF, 1: LED_LOW, 2: LED_HIGH}
        self.led_headlight.duty_u16(us_to_duty(led_values[state.led_level]))
    
    def blink_status(self):
        """Quick blink to show command received."""
        self.status_led.on()
        time.sleep(0.05)
        self.status_led.off()

# ============================================================================
# COMMAND PARSER
# ============================================================================

class CommandParser:
    """Parses incoming UART commands and updates robot state."""
    
    def __init__(self, state, hardware):
        self.state = state
        self.hardware = hardware
    
    def report_status(self, on_demand=False):
        """
        Generates and sends the compact status report over UART.
        If on_demand is True (triggered by 'rstatus'), also prints a detailed
        version to the Pico's local console for debugging.
        """
        # --- Always construct and send the compact line over UART ---
        manual_flag = 1 if self.state.is_manual_control else 0
        status_line = (
            f"STATUS:{self.state.track_left},{self.state.track_right},"
            f"{self.state.head_x_degrees:.1f},{self.state.head_y_degrees:.1f},"
            f"{self.state.led_level},{manual_flag}\n"
        )
        try:
            uart.write(status_line.encode('utf-8'))
        except Exception as e:
            print(f"Error writing status to UART: {e}") # Print error locally

        # --- Optionally, print detailed status locally on Pico console ---
        if on_demand:
            levels = ["OFF", "LOW", "HIGH"]
            # Use print() to output to the Pico's console (e.g., Thonny)
            print("\n--- ROBOT STATUS (On Demand) ---")
            print(f"Tracks (L/R): {self.state.track_left}/{self.state.track_right} us")
            print(f"Head X (Pan): {self.state.head_x_degrees:.1f}° (Target: {self.state.target_head_x:.1f}°)")
            print(f"Head Y (Tilt): {self.state.head_y_degrees:.1f}° (Target: {self.state.target_head_y:.1f}°)")
            print(f"LED Status: {levels[self.state.led_level]} (Level {self.state.led_level})")
            print(f"Manual Override: {'ACTIVE' if self.state.is_manual_control else 'INACTIVE'}")
            print(f"Reporting: {'ENABLED' if self.state.reporting_enabled else 'DISABLED'}")
            print("--------------------")
    
    def _process_command(self, cmd_str, parts, is_manual):
        """Internal command processor."""
        
        # === MOVEMENT COMMANDS ===
        if cmd_str == "f":  # Forward
            self.state.update_targets(track_left=REV, track_right=FWD)
            uart.write("OK: Forward\n")
            
        elif cmd_str == "r":  # Reverse
            self.state.update_targets(track_left=FWD, track_right=REV)
            uart.write("OK: Reverse\n")
            
        elif cmd_str == "lt":  # Left turn (Spin left)
            self.state.update_targets(track_left=REV, track_right=REV)
            uart.write("OK: Left spin\n")
            
        elif cmd_str == "rt":  # Right turn (Spin right)
            self.state.update_targets(track_left=FWD, track_right=FWD)
            uart.write("OK: Right spin\n")
            
        elif cmd_str == "s":  # Stop
            self.state.update_targets(track_left=STOP, track_right=STOP)
            uart.write("OK: Stop\n")
        
        # === HEAD COMMANDS ===
        elif len(parts) == 2 and parts[0] == "hx":
            try:
                degrees = float(parts[1])
                self.state.update_targets(head_x=degrees)
                uart.write(f"OK: Head X target → {self.state.target_head_x:.1f}°\n")
            except Exception as e:
                uart.write(f"ERROR: Invalid head X value - {e}\n")
        
        elif len(parts) == 2 and parts[0] == "hy":
            try:
                degrees = float(parts[1])
                self.state.update_targets(head_y=degrees)
                uart.write(f"OK: Head Y target → {self.state.target_head_y:.1f}°\n")
            except Exception as e:
                uart.write(f"ERROR: Invalid head Y value - {e}\n")
        
        # === LED COMMANDS ===
        elif cmd_str == "ltoggle":  # Cycle through LED levels
            new_level = (self.state.target_led_level + 1) % 3
            self.state.update_targets(led_level=new_level)
            levels = ["OFF", "LOW", "HIGH"]
            uart.write(f"OK: Light set to {levels[new_level]}\n")
        
        elif cmd_str == "lon":  # Full brightness
            self.state.update_targets(led_level=2)
            uart.write("OK: Light HIGH\n")
            
        elif cmd_str == "loff":  # Off
            self.state.update_targets(led_level=0)
            uart.write("OK: Light OFF\n")
        
        # === COMPOSITE POSE COMMAND ===
        elif cmd_str.startswith("pose:"):
            # Format: "pose:left,right,headx,heady,led"
            try:
                values_str = cmd_str.split(":", 1)[1]
                values = values_str.split(",")
                if len(values) == 5:
                    self.state.update_targets(
                        track_left=int(values[0]),
                        track_right=int(values[1]),
                        head_x=float(values[2]),
                        head_y=float(values[3]),
                        led_level=int(values[4])
                    )
                    prefix = "OK: Auto" if not is_manual else "OK:"
                    uart.write(f"{prefix} Pose updated\n")
                else:
                    uart.write(f"ERROR: Pose requires 5 values (got {len(values)})\n")
            except Exception as e:
                uart.write(f"ERROR: Invalid pose - {e}\n")
        
        else:
            uart.write(f"ERROR: Unknown command '{cmd_str}'\n")

    def parse(self, cmd_str):
        """
        Parse command string and update state.
        """
        cmd_str = cmd_str.strip().lower()
        parts = cmd_str.split()
        
        if not cmd_str:
            return
        
        # === STATUS COMMANDS (Do NOT trigger manual override) ===
        if cmd_str == "rstatus":
            self.report_status(on_demand=True)
            return
            
        if cmd_str == "rtoggle":
            self.state.reporting_enabled = not self.state.reporting_enabled
            status = "ENABLED" if self.state.reporting_enabled else "DISABLED"
            uart.write(f"OK: Status Reporting {status}\n")
            return

        # === AUTOMATION COMMANDS ===
        if cmd_str.startswith("auto:"):
            # Only accept if not in manual override
            if self.state.check_override_expired():
                # Process the command but do NOT set manual override
                auto_cmd = cmd_str[5:]
                auto_parts = auto_cmd.split()
                self._process_command(auto_cmd, auto_parts, is_manual=False)
            else:
                uart.write("WARN: Manual override active, ignoring auto command\n")
            return # Exit after handling auto command

        # === ALL OTHER COMMANDS are MANUAL ===
        # Trigger manual override/block automation
        self.state.set_manual_override()
        self.hardware.blink_status()
        self._process_command(cmd_str, parts, is_manual=True)

# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    """Main control loop."""
    
    # Initialize
    state = RobotState()
    hardware = HardwareController()
    parser = CommandParser(state, hardware)
    
    # Set initial positions
    hardware.apply_state(state)
    uart.write("\nPico Ready\n")
    
    last_update_ms = time.ticks_ms()
    last_report_ms = time.ticks_ms()
    
    cmd_buffer = "" # Add a command buffer
    
    while True:
        current_time_ms = time.ticks_ms()
        
        # Process incoming commands (with buffering)
        if uart.any():
            try:
                # Read all available bytes
                raw_data = uart.read()
                if raw_data:
                    cmd_buffer += raw_data.decode('utf-8')
                    
                    # Process all complete lines in the buffer
                    while '\n' in cmd_buffer:
                        line, cmd_buffer = cmd_buffer.split('\n', 1)
                        cmd = line.strip()
                        if cmd: # Don't process empty lines
                            parser.parse(cmd)
                            
            except Exception as e:
                uart.write(f"ERROR: {e}\n")
        
        # Update state smoothly at fixed rate
        if time.ticks_diff(current_time_ms, last_update_ms) >= HEAD_UPDATE_RATE_MS:
            state.smooth_update()
            hardware.apply_state(state)
            last_update_ms = current_time_ms
        
        # Send periodic status if enabled
        if state.reporting_enabled and \
           time.ticks_diff(current_time_ms, last_report_ms) >= STATUS_REPORT_INTERVAL_MS:
            parser.report_status(on_demand=False)
            last_report_ms = current_time_ms
            
        # Check if manual override has expired and reset control flag
        state.check_override_expired() 

        time.sleep_ms(1)  # Small delay to prevent busy-waiting

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()

