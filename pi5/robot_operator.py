#!/usr/bin/python3
# -*- coding:utf-8 -*-

"""
robot_operator.py

The central 'brain' of the robot.
- Reads state from sensors.py and control.py (via JSON files)
- Manages high-level robot state (MANUAL vs. AUTONOMOUS)
- Sends commands to control.py, display.py, and speech.py (via Sockets)
- Provides a CLI for direct operator control.
- Loads and runs complex animation sequences from 'animations.json'.
"""

import socket
import threading
import time
import sys
import os
import json
import random
import queue
import datetime
import selectors

# --- Configuration ---

# Socket ports for other modules
CONTROL_PORT = 65001
DISPLAY_PORT = 65002
SPEECH_PORT = 65003
OPERATOR_PORT = 65004  # This script's port

# JSON state files
SENSOR_JSON_PATH = "/dev/shm/sensor_data.json"
CONTROL_JSON_PATH = "/dev/shm/control_state.json"
ANIMATIONS_JSON_PATH = "animations.json" 

# Timing
STATE_READ_INTERVAL = 0.1   # (10Hz) How often to read JSON state files
AUTONOMY_TICK_RATE = 0.5   # (2Hz) How often the "brain" thinks

# --- Socket Helper Class ---

class CommandSocketHelper:
    """
    A helper class to manage sending socket commands to other modules.
    """
    def __init__(self, port, name="Module"):
        self.address = ('127.0.0.1', port)
        self.name = name
        self.lock = threading.Lock()

    def send_command(self, command_str: str):
        """
        Connects, sends a command, and disconnects.
        Uses a lock to prevent simultaneous sends from different threads.
        """
        if not command_str:
            return

        command_str = command_str.strip() + '\n'
        
        with self.lock:
            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(self.address)
                s.sendall(command_str.encode('utf-8'))
            except socket.timeout:
                print(f"Socket Warning: Timeout sending to {self.name}")
            except ConnectionRefusedError:
                print(f"Socket Error: Connection refused for {self.name} at {self.address}")
            except Exception as e:
                print(f"Socket Error: Failed to send '{command_str.strip()}' to {self.name}: {e}")
            finally:
                if s:
                    s.close()

# --- Main Operator Class ---

class RobotOperator:
    """
    The main class for the robot's brain. Manages state, threads, and actions.
    """
    # --- ANSI Color Codes for Status Report ---
    C_RESET = "\033[0m"
    C_BOLD = "\033[1m"
    C_HEADER = "\033[1;33m" # Bold Yellow
    C_GREEN = "\033[32m"
    C_YELLOW = "\033[33m"
    C_RED = "\033[31m"
    C_CYAN = "\033[36m"
    C_DIM = "\033[2m" 
    # --- Head Control Constants ---
    # These values must match the limits in control.py
    HEAD_PAN_MIN = 90.0
    HEAD_PAN_MAX = 270.0
    HEAD_TILT_MIN = 120.0
    HEAD_TILT_MAX = 260.0
    HEAD_STEP_SIZE = 10.0 # Degrees to move per web button press
    # --- Moved Thresholds to Class Level ---
    LOW_BATTERY_THRESHOLD = 15.0
    CHARGED_RESET_THRESHOLD = 20.0 # Must charge above this to reset
    WARNING_INTERVAL_SEC = 15 * 60 # 15 minutes
    STARTUP_GRACE_SEC = 5.0 
    # Temperature Thresholds (Example - Adjust as needed)
    HIGH_TEMP_THRESHOLD = 60.0 
    SAFE_TEMP_THRESHOLD = 55.0

    def __init__(self):
        self.mode = 'MANUAL'  # 'MANUAL' or 'AUTONOMOUS'
        self.robot_state = {} # Full combined state from JSON files
        self.state_lock = threading.Lock()
        self.running = True
        self.start_time = time.time() # Record startup time
        
        # State flags for spam prevention
        self.last_battery_warning_time = 0.0 # 0.0 means "warn immediately"
        self.last_controller_status = None # None = "unknown" initial state

        # New CLI Queue
        self.cli_command_queue = queue.Queue()

        # Socket Helpers
        self.control = CommandSocketHelper(CONTROL_PORT, "Control")
        self.display = CommandSocketHelper(DISPLAY_PORT, "Display")
        self.speech = CommandSocketHelper(SPEECH_PORT, "Speech")

        # Load animations from file
        self.animations = {}
        self.load_animations()

        # Start background threads
        self.state_reader_thread = threading.Thread(target=self._state_reader_loop, daemon=True)
        self.autonomy_thread = threading.Thread(target=self._autonomy_loop, daemon=True)
        self.cli_reader_thread = threading.Thread(target=self._cli_reader_loop, daemon=True)

        self.state_reader_thread.start()
        print("State reader thread started.")
        self.autonomy_thread.start()
        print("Autonomy loop thread started.")
        self.cli_reader_thread.start()
        print("CLI input thread started.")

        # Socket Server for This Module
        self.sel = selectors.DefaultSelector()
        self.server_socket = self._setup_server_socket()
        if self.server_socket:
            print(f"Listening for commands on 127.0.0.1:{OPERATOR_PORT}")
        else:
            print(f"FATAL: Could not start server on 127.0.0.1:{OPERATOR_PORT}")
            self.running = False


    # --- 1. Background Threads ---

    def _state_reader_loop(self):
        """
        [PERCEIVE THREAD]
        Periodically reads all JSON files and updates the internal robot_state.
        """
        while self.running:
            sensor_data = {}
            control_data = {}

            try:
                with open(SENSOR_JSON_PATH, 'r') as f:
                    sensor_data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            except Exception as e:
                print(f"StateReader Error (Sensor): {e}")
                
            try:
                with open(CONTROL_JSON_PATH, 'r') as f:
                    control_data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            except Exception as e:
                print(f"StateReader Error (Control): {e}")

            # Safely update the shared state
            with self.state_lock:
                self.robot_state['sensors'] = sensor_data
                self.robot_state['control'] = control_data
                self.robot_state['timestamp'] = datetime.datetime.utcnow().isoformat()

            time.sleep(STATE_READ_INTERVAL)

    def _autonomy_loop(self):
        """
        [THINK THREAD]
        The main "brain" loop. Runs at AUTONOMY_TICK_RATE.
        Decides which tick function to run based on the current mode.
        """
        while self.running:
            if self.mode == 'AUTONOMOUS':
                self.run_autonomy_tick()
            elif self.mode == 'MANUAL':
                self.run_manual_monitoring() 
            
            time.sleep(AUTONOMY_TICK_RATE)

    def _cli_reader_loop(self):
        """
        [CLI THREAD]
        Dedicated thread to read from stdin (terminal input)
        without blocking the main server loop.
        """
        try:
            for line in sys.stdin:
                if not self.running:
                    break
                command = line.strip()
                if command:
                    self.cli_command_queue.put(command)
        except Exception as e:
            if self.running:
                print(f"CLI Reader Error: {e}")
                
    # --- 2. Socket Server ---

    def _setup_server_socket(self):
        """
        Initializes the main socket server for this operator module.
        """
        try:
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind(('127.0.0.1', OPERATOR_PORT))
            server_sock.listen()
            server_sock.setblocking(False)
            self.sel.register(server_sock, selectors.EVENT_READ, data=self._accept_connection)
            return server_sock
        except Exception as e:
            print(f"SocketServer Error: Failed to setup server: {e}")
            return None

    def _accept_connection(self, server_sock):
        """
Callback for new socket connections.
        """
        try:
            conn, addr = server_sock.accept()
            conn.setblocking(False)
            self.sel.register(conn, selectors.EVENT_READ, data=self._handle_client_data)
        except Exception as e:
            print(f"SocketServer Error: Failed to accept connection: {e}")

    def _handle_client_data(self, conn):
        """
        Callback for receiving data from a connected client.
        """
        try:
            data = conn.recv(1024)
            if data:
                command_str = data.decode('utf-8').strip()
                if command_str:
                    print(f"Socket Input: Received '{command_str}'")
                    # Put command on the same queue as the CLI
                    self.cli_command_queue.put(command_str)
            else:
                # No data means client disconnected
                self.sel.unregister(conn)
                conn.close()
        except Exception as e:
            print(f"SocketServer Error: Failed handling client data: {e}")
            self.sel.unregister(conn)
            conn.close()

    def check_socket_commands(self):
        """
        Polls the selector for any socket events.
        """
        try:
            events = self.sel.select(timeout=0)
            for key, mask in events:
                callback = key.data
                callback(key.fileobj)
        except Exception as e:
            print(f"Socket Poll Error: {e}")

    def check_cli_commands(self):
        """
        Checks the queue for commands from the CLI thread or socket server.
        """
        try:
            command_str = self.cli_command_queue.get_nowait()
            self.parse_and_execute_cli_command(command_str)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"CLI Queue Error: {e}")

    # --- 3. "Think" Logic ---

    def run_manual_monitoring(self): 
        """
        [THINK]
        Logic that runs when in MANUAL mode, monitoring for events.
        This now includes spam-prevention logic.
        """
        batt_pct = 100.0
        controller_connected = True
        
        # --- Get current state safely ---
        try:
            with self.state_lock:
                batt_pct = self.robot_state.get('sensors', {}).get('power', {}).get('battery_percent', 100.0)
                controller_connected = self.robot_state.get('control', {}).get('controller_connected', False)
        except Exception as e:
            print(f"Manual Monitoring Error (State-Read): {e}")
            return # Don't act on bad data

        current_time = time.time() # Get the current time
        
        # --- Startup Grace Period ---
        if (current_time - self.start_time) < self.STARTUP_GRACE_SEC:
            # Silently update controller status
            if self.last_controller_status is None:
                self.last_controller_status = controller_connected
            else:
                self.last_controller_status = controller_connected
            
            # Silently update battery warning time
            if batt_pct < self.LOW_BATTERY_THRESHOLD:
                if self.last_battery_warning_time == 0.0:
                    self.last_battery_warning_time = current_time
            
            return # Exit early, skipping all announcements.


        # --- Low Battery Logic (with Hysteresis and Timed Nag) ---
        if batt_pct < self.LOW_BATTERY_THRESHOLD:
            time_since_last_warning = current_time - self.last_battery_warning_time
            
            if time_since_last_warning > self.WARNING_INTERVAL_SEC:
                print(f"MANUAL MONITOR: Battery is low ({batt_pct:.0f}%)! Sending warning.")
                self.speak(f"Warning: Battery is at {batt_pct:.0f} percent")
                self.set_expression("SAD")
                self.last_battery_warning_time = current_time
                
        elif batt_pct > self.CHARGED_RESET_THRESHOLD:
            if self.last_battery_warning_time != 0.0:
                print("MANUAL MONITOR: Battery recharged. Resetting warning timer.")
                self.last_battery_warning_time = 0.0
        
        # --- Controller Connection Logic (Event-based) ---
        if self.last_controller_status is None:
            self.last_controller_status = controller_connected
        elif controller_connected and not self.last_controller_status:
            print("MANUAL MONITOR: Controller re-connected.")
            self.speak("Controller connected.")
            self.set_expression("IDLE") # Back to normal
        elif not controller_connected and self.last_controller_status:
            print("MANUAL MONITOR: Controller DISCONNECTED.")
            self.speak("Warning. Controller disconnected.")
            self.set_expression("ERROR")
            
        # Update our "memory" for the next tick
        self.last_controller_status = controller_connected


    def run_autonomy_tick(self):
        """
        [THINK]
        Logic that runs every tick when in AUTONOMOUS mode.
        (This is the placeholder "curious" behavior)
        """
        print("AUTONOMY TICK: Running...")

        # --- This is just a placeholder sequence ---
        self.pan_head(120)
        self.speak("I am in autonomous mode.")
        time.sleep(1.5)
        self.pan_head(240)
        self.show_accent("HEART")
        time.sleep(1.5)
        self.pan_head(180)
        self.move('f')
        time.sleep(1.0)
        self.move('s')


    def _show_help(self):
        """
        [CLI]
        Prints the help message for all CLI commands.
        """
        help_text = f"""
    --- Robot Operator CLI Help ---
    
    {self.C_HEADER}[Operator Commands]{self.C_RESET}
      HELP, H, ?          : Show this help message
      STATUS / STAT [full] : Show a robot status report (add 'full' for all data)
      SET_MODE [MODE]   : Set mode (MANUAL or AUTONOMOUS)
      QUIT / EXIT       : Stop this operator script
    
    {self.C_HEADER}[Complex Actions]{self.C_RESET}
      ANIMATION <name>    : Run a pre-defined animation (e.g., ANIMATION dance)
      RELOAD_ANIMATIONS   : Reloads the 'animations.json' file

    {self.C_HEADER}[Speech Commands]{self.C_RESET}
      SPEAK <text...>   : Make the robot say something (e.g., SPEAK Hello world)
    
    {self.C_HEADER}[Control Commands]{self.C_RESET}
      F, R, S, LT, RT   : Control tracks (Forward, Reverse, Stop, Left Turn, Right Turn)
      LTOGGLE           : Toggle the headlight
      RTOGGLE           : Toggle Pico's status reporting (for control.py)
      HX <angle>          : Set head pan (horizontal) angle (e.g., HX 180.0)
      HY <angle>          : Set head tilt (vertical) angle (e.g., HY 180.0)
    
    {self.C_HEADER}[Display Commands]{self.C_RESET}
      IDLE, HAPPY, SAD, ANGRY, ERROR, SLEEP : Set face expression
      EYE <x> <y>         : Set pupil position (e.g., EYE 0 0)
      BLUSH [ON/OFF]    : Toggle blush
      ACCENT <type>     : Show a brief accent (e.g., ACCENT HEART)
      BROW [params..]   : Control eyebrows (e.g., BROW ON 8 150 -100 15 0.3)
      WAVE <amp> <freq>   : Set mouth to wave (e.g., WAVE 20 5)
      MOUTH [params..]  : Set mouth shape (e.g., MOUTH ROUND)
    
    ---------------------------------
        """
        print(help_text)

    def _show_status_report(self, show_full=False):
        """
        [CLI]
        Grabs the latest robot state and prints a formatted report with colors.
        """
        print(f"\n--- {self.C_BOLD}ROBOT STATUS REPORT{self.C_RESET} ---")
        
        # Use .get() extensively to avoid errors if keys don't exist
        with self.state_lock:
            # --- Get all state sections first ---
            sensors_state = self.robot_state.get('sensors', {})
            control_state = self.robot_state.get('control', {})
            
            pico_state = control_state.get('pico_state', {})
            power_state = sensors_state.get('power', {})
            env_state = sensors_state.get('environment', {})
            light_state = sensors_state.get('light', {})
            adc_state = sensors_state.get('adc_volts', {})
            motion_state = sensors_state.get('motion', {})
            
            # --- Operator State (Always Show) ---
            print(f"\n{self.C_HEADER}[Operator]{self.C_RESET}")
            print(f"  Mode: {self.C_CYAN}{self.mode}{self.C_RESET}")
            
            # --- Control State (Always Show) ---
            print(f"\n{self.C_HEADER}[Control System]{self.C_RESET}")
            ctl_name = control_state.get('controller_name', 'N/A')
            ctl_connected = control_state.get('controller_connected', False)
            pico_connected = pico_state.get('serial_connected', False)
            
            ctl_conn_str = f"{self.C_GREEN}CONNECTED{self.C_RESET}" if ctl_connected else f"{self.C_RED}DISCONNECTED{self.C_RESET}"
            pico_conn_str = f"{self.C_GREEN}ONLINE{self.C_RESET}" if pico_connected else f"{self.C_RED}OFFLINE{self.C_RESET}"
            
            print(f"  Controller: {ctl_conn_str} ({self.C_CYAN}{ctl_name}{self.C_RESET})")
            print(f"  Pico Status: {pico_conn_str}")
            
            # --- Power State (Always Show) ---
            print(f"\n{self.C_HEADER}[Power System (UPS)]{self.C_RESET}")
            batt = power_state.get('battery_percent', 0.0)
            volt = power_state.get('bus_voltage_V', 0.0)
            curr = power_state.get('current_A', 0.0)
            power = power_state.get('power_W', 0.0)
            is_charging = (volt > 10.0 and curr > 0.02)
            charge_str = f"{self.C_YELLOW}(Charging){self.C_RESET}" if is_charging else f"{self.C_DIM}(Discharging){self.C_RESET}"
            
            batt_color = self.C_RED if batt < self.LOW_BATTERY_THRESHOLD else self.C_YELLOW if batt < 50 else self.C_GREEN
            
            print(f"  Battery: {batt_color}{batt:.1f}%{self.C_RESET} {charge_str}")
            print(f"  Voltage: {self.C_CYAN}{volt:.2f} V{self.C_RESET}")
            print(f"  Current: {self.C_CYAN}{curr:.3f} A{self.C_RESET}")
            print(f"  Power:   {self.C_CYAN}{power:.2f} W{self.C_RESET}")

            # --- Environment State (Always Show) ---
            print(f"\n{self.C_HEADER}[Environment]{self.C_RESET}")
            temp_c = env_state.get('temperature_shtc_c', 0.0)
            temp_f = (temp_c * 9/5) + 32
            humidity = env_state.get('humidity_rh', 0.0)
            pressure = env_state.get('pressure_hpa', 0.0)
            
            # [MODIFIED] Apply color based on temp thresholds
            temp_color = self.C_RED if temp_c > self.HIGH_TEMP_THRESHOLD else self.C_YELLOW if temp_c > self.SAFE_TEMP_THRESHOLD else self.C_GREEN
            
            print(f"  Core Temp:   {temp_color}{temp_c:.1f}°C{self.C_RESET} / {temp_f:.1f}°F")
            print(f"  Humidity:    {self.C_CYAN}{humidity:.1f}%{self.C_RESET}")
            print(f"  Pressure:    {self.C_CYAN}{pressure:.2f} hPa{self.C_RESET}")

            # --- Show these sections only if 'show_full' is True ---
            if show_full:
                print(f"\n{self.C_HEADER}[Pico - Live State]{self.C_RESET}")
                l_track = pico_state.get('left_track_speed', 0)
                r_track = pico_state.get('right_track_speed', 0)
                h_pan = pico_state.get('head_pan_actual', 0.0)
                h_tilt = pico_state.get('head_tilt_actual', 0.0)
                led = pico_state.get('led_state', 0)
                led_state_str = {0: "Off", 1: "Low", 2: "High"}.get(led, "Unknown")
                
                print(f"  Tracks (L/R): {self.C_CYAN}{l_track}{self.C_RESET} / {self.C_CYAN}{r_track}{self.C_RESET}")
                print(f"  Head Pan/Tilt: {self.C_CYAN}{h_pan:.1f}°{self.C_RESET} / {self.C_CYAN}{h_tilt:.1f}°{self.C_RESET}")
                print(f"  Headlight: {self.C_YELLOW if led > 0 else self.C_DIM}{led_state_str}{self.C_RESET} ({self.C_CYAN}{led}{self.C_RESET})")
                
                print(f"\n{self.C_HEADER}[IMU - Motion]{self.C_RESET}")
                accel = motion_state.get('accelerometer_raw', {})
                gyro = motion_state.get('gyroscope_raw', {})
                mag = motion_state.get('magnetometer_raw', {})
                
                print(f"  Accelerometer: X={self.C_CYAN}{accel.get('x', 0):<6}{self.C_RESET} Y={self.C_CYAN}{accel.get('y', 0):<6}{self.C_RESET} Z={self.C_CYAN}{accel.get('z', 0):<6}{self.C_RESET}")
                print(f"  Gyroscope:     X={self.C_CYAN}{gyro.get('x', 0):<6}{self.C_RESET} Y={self.C_CYAN}{gyro.get('y', 0):<6}{self.C_RESET} Z={self.C_CYAN}{gyro.get('z', 0):<6}{self.C_RESET}")
                print(f"  Magnetometer:  X={self.C_CYAN}{mag.get('x', 0):<6.1f}{self.C_RESET} Y={self.C_CYAN}{mag.get('y', 0):<6.1f}{self.C_RESET} Z={self.C_CYAN}{mag.get('z', 0):<6.1f}{self.C_RESET}")

                print(f"\n{self.C_HEADER}[Detailed Sensor Data]{self.C_RESET}")
                lps_temp = env_state.get('temperature_lps_c', 0.0)
                imu_temp = env_state.get('temperature_imu_c', 0.0)
                
                # Apply same color logic to other temps
                lps_temp_color = self.C_RED if lps_temp > self.HIGH_TEMP_THRESHOLD else self.C_YELLOW if lps_temp > self.SAFE_TEMP_THRESHOLD else self.C_GREEN
                imu_temp_color = self.C_RED if imu_temp > self.HIGH_TEMP_THRESHOLD else self.C_YELLOW if imu_temp > self.SAFE_TEMP_THRESHOLD else self.C_GREEN

                print(f"  LPS Temp:    {lps_temp_color}{lps_temp:.1f}°C{self.C_RESET}")
                print(f"  IMU Temp:    {imu_temp_color}{imu_temp:.1f}°C{self.C_RESET}")
                print(f"  Light (Lux): {self.C_CYAN}{light_state.get('lux', 0.0):.2f}{self.C_RESET}")
                print(f"  Light (CCT): {self.C_CYAN}{light_state.get('color_temp_k', 0.0):.0f} K{self.C_RESET}")
                print(f"  Light (Raw): R={self.C_CYAN}{light_state.get('red_raw', 0)}{self.C_RESET} G={self.C_CYAN}{light_state.get('green_raw', 0)}{self.C_RESET} B={self.C_CYAN}{light_state.get('blue_raw', 0)}{self.C_RESET} C={self.C_CYAN}{light_state.get('clear_raw', 0)}{self.C_RESET}")

            # --- ADC State (Always Show) ---
            print(f"\n{self.C_HEADER}[ADC Voltages]{self.C_RESET}")
            ain0 = adc_state.get('ain0', 0.0)
            ain1 = adc_state.get('ain1', 0.0)
            ain2 = adc_state.get('ain2', 0.0)
            ain3 = adc_state.get('ain3', 0.0)
            
            print(f"  AIN0: {self.C_CYAN}{ain0:.4f}V{self.C_RESET}   AIN1: {self.C_CYAN}{ain1:.4f}V{self.C_RESET}")
            print(f"  AIN2: {self.C_CYAN}{ain2:.4f}V{self.C_RESET}   AIN3: {self.C_CYAN}{ain3:.4f}V{self.C_RESET}")
            
        print(f"\n--- {self.C_DIM}END OF REPORT{self.C_RESET} ---\n")
    # --- ADD THIS ENTIRE METHOD ---
    def move_head_relative(self, pan_adjust=0.0, tilt_adjust=0.0):
        """
        [ACT]
        Calculates and sends a new absolute head position based on a relative adjustment.
        Reads the *current* position from the state file to prevent desync.
        """
        current_pan = 180.0
        current_tilt = 180.0

        # 1. Safely read the current position from the shared state
        with self.state_lock:
            try:
                # Get the *actual* position reported by the Pico
                current_pan = self.robot_state['control']['pico_state']['head_pan_actual']
                current_tilt = self.robot_state['control']['pico_state']['head_tilt_actual']
            except KeyError:
                print("move_head_relative: Warning! Pico state not yet available. Using default center (180, 180).")
            except Exception as e:
                print(f"move_head_relative: Error reading state: {e}. Using default center (180, 180).")
        
        # 2. Calculate new target if adjustment was provided
        if pan_adjust != 0.0:
            # Calculate new pan and clamp it within the allowed range
            new_pan = current_pan + pan_adjust
            new_pan = max(self.HEAD_PAN_MIN, min(self.HEAD_PAN_MAX, new_pan))
            
            print(f"ACTION: Rel Pan ({pan_adjust:+.1f}). Old: {current_pan:.1f} -> New: {new_pan:.1f}")
            self.pan_head(new_pan) # pan_head() already sends "hx ..."

        if tilt_adjust != 0.0:
            # Calculate new tilt and clamp it within the allowed range
            new_tilt = current_tilt + tilt_adjust
            new_tilt = max(self.HEAD_TILT_MIN, min(self.HEAD_TILT_MAX, new_tilt))

            print(f"ACTION: Rel Tilt ({tilt_adjust:+.1f}). Old: {current_tilt:.1f} -> New: {new_tilt:.1f}")
            self.tilt_head(new_tilt) # tilt_head() already sends "hy ..."

    # --- Animation Methods ---

    def load_animations(self):
        """
        Loads or reloads the animation sequences from the JSON file.
        """
        try:
            with open(ANIMATIONS_JSON_PATH, 'r') as f:
                self.animations = json.load(f)
            print(f"Successfully loaded {len(self.animations)} animations from '{ANIMATIONS_JSON_PATH}'.")
        except FileNotFoundError:
            print(f"Warning: Animation file not found: '{ANIMATIONS_JSON_PATH}'. No animations loaded.")
            self.animations = {}
        except json.JSONDecodeError:
            print(f"Error: Could not parse '{ANIMATIONS_JSON_PATH}'. Check for JSON syntax errors.")
        except Exception as e:
            print(f"Error loading animations: {e}")

    def run_animation_sequence(self, animation_name: str, sequence_list: list):
        """
        A generic function that executes any animation sequence in a thread.
        """
        print(f"ACTION: Starting animation '{animation_name}'...")
        
        for step in sequence_list:
            if not self.running: # Stop if the main operator is shutting down
                print(f"ACTION: Animation '{animation_name}' stopped early.")
                return
            
            command_str = step.get("command")
            pause_duration = float(step.get("pause", 0.0))

            if command_str:
                # We re-use the main CLI parser to execute the command.
                # This is powerful, as it lets animations call ANY command.
                print(f"  -> Anim Step: {command_str}")
                self.parse_and_execute_cli_command(command_str)
            
            if pause_duration > 0.0:
                time.sleep(pause_duration)
        
        if self.running:
            print(f"ACTION: Animation '{animation_name}' finished.")


    # --- 4. Public Command API (Helper methods) ---

    def set_mode(self, new_mode: str):
        """
        [ACT]
        Safely changes the robot's primary mode.
        """
        new_mode = new_mode.upper()
        if new_mode == self.mode:
            return
        if new_mode not in ['MANUAL', 'AUTONOMOUS']:
            print(f"Error: Unknown mode '{new_mode}'")
            return
            
        print(f"--- Changing mode from {self.mode} to {new_mode} ---")
        self.mode = new_mode

        if new_mode == 'MANUAL':
            # SAFETY: Stop any autonomous movement when switching to manual
            self.move('s') 
            self.speak("Manual mode engaged")
        
        elif new_mode == 'AUTONOMOUS':
            self.speak("Autonomous mode engaged")
            self.set_expression("HAPPY")

    def speak(self, text: str):
        """
        [ACT]
        Sends a command to the speech module.
        """
        self.speech.send_command(f"{text}") # speech.py doesn't need "SPEAK" prefix

    def move(self, direction: str):
        """
        [ACT]
        Sends a movement command to the control module.
        """
        self.control.send_command(direction)

    def pan_head(self, angle: float):
        """
        [ACT]
        Sends a head pan command to the control module.
        """
        self.control.send_command(f"hx {angle}")
        
    def tilt_head(self, angle: float):
        """
        [ACT]
        Sends a head tilt (vertical) angle to the control module.
        """
        self.control.send_command(f"hy {angle}")

    def set_expression(self, expression: str):
        """
        [ACT]
        Sends an expression command to the display module.
        """
        self.display.send_command(expression)

    def set_blush(self, on: bool):
        """
        [ACT]
        Sends a blush command to the display module.
        """
        self.display.send_command(f"BLUSH {'ON' if on else 'OFF'}")

    def set_brows(self, on: bool, params_str: str = ""):
        """
        [ACT]
        Sends a brow command to the display module.
        """
        if on:
            self.display.send_command(f"BROW ON {params_str}".strip())
        else:
            self.display.send_command("BROW OFF")

    def show_accent(self, accent_type: str):
        """
        [ACT]
        Sends an accent command to the display module.
        """
        self.display.send_command(f"ACCENT {accent_type}")

    def stop(self):
        """
        Shuts down the operator script cleanly.
        """
        print("Shutting down operator module...")
        self.running = False
        
        # Stop all robot motion
        self.move('s')
        self.set_expression("IDLE")
        
        # Close server socket
        if self.server_socket:
            self.sel.unregister(self.server_socket)
            self.server_socket.close()
            
        self.sel.close()
        print("Shutdown complete.")


    # --- 5. CLI and Command Parsing ---

    def parse_and_execute_cli_command(self, command_str: str):
        """
        [CLI / ACT]
        The master parser for all commands coming from the CLI or Sockets.
        It splits the command and routes it to the correct helper function or module.
        """
        if not command_str:
            return
            
        parts = command_str.strip().split()
        if not parts:
            return
            
        cmd = parts[0].upper()
        args = parts[1:]
        full_args_str = " ".join(args)

        try:
            # --- Help Command ---
            if cmd == 'HELP' or cmd == 'H' or cmd == '?':
                self._show_help()

            # --- Status Command ---
            elif cmd == 'STATUS' or cmd == 'STAT':
                show_full_report = False
                if len(args) == 1 and args[0].upper() == 'FULL':
                    show_full_report = True
                self._show_status_report(show_full_report) # Pass the flag
                
            # --- Operator Internal Commands ---
            elif cmd == 'SET_MODE' and len(args) == 1:
                self.set_mode(args[0])

            # --- Complex Action Triggers ---
            elif cmd == 'ANIMATION' and len(args) == 1:
                animation_name = args[0].lower() # Use lower for case-insensitivity
                sequence = self.animations.get(animation_name)
                
                if sequence:
                    # Run the action in a new thread so it doesn't block the CLI
                    threading.Thread(
                        target=self.run_animation_sequence, 
                        args=(animation_name, sequence,), 
                        daemon=True
                    ).start()
                else:
                    print(f"Error: Animation '{animation_name}' not found.")
            
            elif cmd == 'RELOAD_ANIMATIONS':
                self.load_animations()

            # --- Speech Commands ---
            elif cmd == 'SPEAK':
                if not args:
                    print("Error: SPEAK requires text. (e.g., SPEAK Hello)")
                else:
                    self.speak(full_args_str) # Send the full text

            # --- Relative Head Control (from Web UI) ---
            # NOTE: Based on your control.py logic:
            # 'Left' and 'Up' ADD to the angle value.
            # 'Right' and 'Down' SUBTRACT from the angle value.
            elif cmd == 'PAN_LEFT':
                self.move_head_relative(pan_adjust=self.HEAD_STEP_SIZE)
            elif cmd == 'PAN_RIGHT':
                self.move_head_relative(pan_adjust=-self.HEAD_STEP_SIZE)
            elif cmd == 'TILT_UP':
                self.move_head_relative(tilt_adjust=self.HEAD_STEP_SIZE)
            elif cmd == 'TILT_DOWN':
                self.move_head_relative(tilt_adjust=-self.HEAD_STEP_SIZE)
                
            # --- Control Commands (Direct passthrough) ---
            elif cmd in ['F', 'R', 'S', 'LT', 'RT', 'LTOGGLE', 'RTOGGLE']:
                self.control.send_command(cmd)
            
            elif cmd == 'HX' and len(args) == 1:
                self.control.send_command(f"hx {args[0]}")
                
            elif cmd == 'HY' and len(args) == 1:
                self.control.send_command(f"hy {args[0]}")

            # --- Display Commands (Direct passthrough) ---
            elif cmd in ['IDLE', 'HAPPY', 'SAD', 'ANGRY', 'ERROR', 'SLEEP']:
                self.display.send_command(cmd)
            
            elif cmd in ['EYE', 'BLUSH', 'ACCENT', 'WAVE', 'MOUTH', 'BROW']:
                # Pass the full command string directly to display.py
                self.display.send_command(command_str)

            # --- Quit ---
            elif cmd == 'QUIT' or cmd == 'EXIT':
                self.running = False
            
            # --- Unknown ---
            else:
                print(f"Unknown command: '{command_str}'. Type 'HELP' for a list.")

        except Exception as e:
            print(f"Error processing command '{command_str}': {e}")
            import traceback
            traceback.print_exc()


# --- Main Execution ---

if __name__ == "__main__":
    print("--- Robot Operator Module Active ---")
    print("This script is the 'brain' of the robot.")
    
    operator = None
    try:
        operator = RobotOperator()
        
        print("\nType 'HELP' for commands, or 'QUIT' to exit.")
        
        # Main loop: checks for socket commands and CLI commands
        while operator.running:
            operator.check_socket_commands()
            operator.check_cli_commands()
            time.sleep(0.05) # 20Hz poll rate for commands

    except KeyboardInterrupt:
        print("\nCaught Ctrl+C. Shutting down...")
        
    except Exception as e:
        print(f"\nFATAL ERROR in main loop: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if operator:
            operator.stop()