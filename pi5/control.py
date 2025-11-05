#!/usr/bin/python
# -*- coding:utf-8 -*-
import os
# Hide Pygame welcome message before importing pygame
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"

import pygame
import serial
import time
import sys
import threading
import queue
import json # For JSON output
import datetime # For timestamp
import socket # For IPC commands
import selectors # For handling multiple socket connections efficiently

# --- Configuration ---
SERIAL_PORT = '/dev/ttyAMA10'  # Pi 5 UART port
BAUD_RATE = 115200
CONTROL_JSON_PATH = "/dev/shm/control_state.json" # Output JSON file path
STATUS_REQUEST_INTERVAL = 0.5 # How often (seconds) to ask Pico for status
HEAD_UPDATE_RATE = 3 # Loops between sending head commands (rate limiting)
DEADZONE = 0.15 # Joystick deadzone
LOOP_RATE_HZ = 50 # Target loop frequency

# --- Socket Server Configuration ---
SERVER_HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
SERVER_PORT = 65001        # Port for control commands

# --- Controller Mapping (Nintendo Switch Pro Controller - Corrected Indices) ---
# Axes
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 2
RIGHT_STICK_Y = 3
LEFT_TRIGGER_AXIS = 4
RIGHT_TRIGGER_AXIS = 5
# Buttons
B_BUTTON = 0      # Center Head
A_BUTTON = 1      # Toggle Status Reporting (rtoggle)
X_BUTTON = 2      # Toggle/Cycle Light Level (ltoggle)
Y_BUTTON = 3      # State recorded in JSON
CAPTURE_BUTTON = 4 # State recorded in JSON
L_BUMPER = 5      # State recorded in JSON
R_BUMPER = 6      # State recorded in JSON
ZL_TRIGGER = 7    # State recorded in JSON
ZR_TRIGGER = 8    # State recorded in JSON
MINUS_BUTTON = 9  # State recorded in JSON
PLUS_BUTTON = 10  # State recorded in JSON
HOME_BUTTON = 11   # State recorded in JSON
LEFT_STICK_CLICK = 12 # State recorded in JSON
RIGHT_STICK_CLICK = 13 # State recorded in JSON
# Hat
DPAD_HAT = 0

# --- State Tracking ---
# Controller Targets (Sent TO Pico)
current_head_x = 180.0 # Target pan
current_head_y = 180.0 # Target tilt
HEAD_STEP = 5.0 # Degrees per D-pad press interval

# Pico Status (Received FROM Pico - with defaults)
pico_reporting_enabled = False
pico_last_status_str = "Unknown"
pico_current_head_x = 180.0
pico_current_head_y = 180.0
pico_left_track_speed = 0
pico_right_track_speed = 0
pico_led_state = 0
pico_manual_mode = True

# Other State
last_track_command = None
last_button_state = {}
last_sent_command = None
last_status_request_time = 0.0

# Command Queue for manual terminal input
command_queue = queue.Queue()

# --- Initialize Pygame and Controller ---
pygame.init()
pygame.joystick.init()
controller = None
controller_name = "N/A"
num_axes = 0
num_buttons = 0
num_hats = 0
if pygame.joystick.get_count() > 0:
    try:
        controller = pygame.joystick.Joystick(0)
        controller.init()
        controller_name = controller.get_name()
        num_axes = controller.get_numaxes()
        num_buttons = controller.get_numbuttons()
        num_hats = controller.get_numhats()
        print(f"Controller connected: {controller_name}")
        print(f"Axes: {num_axes}, Buttons: {num_buttons}, Hats: {num_hats}")
    except pygame.error as e:
        print(f"Error initializing controller: {e}")
        controller = None
else:
    print("No controller detected! Only terminal commands will be available.")

# --- Open serial connection to Pico ---
ser = None
pico_serial_connected = False
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    ser.read_all()
    pico_serial_connected = True
    print(f"Serial connected on {SERIAL_PORT}")
except Exception as e:
    print(f"Serial connection failed: {e}")
    # sys.exit() # Decide if script should exit if serial fails

# --- Core Functions ---

def read_response():
    """Reads data from Pico, parses status, updates global state, and prints conditionally."""
    global pico_reporting_enabled, pico_last_status_str
    global pico_current_head_x, pico_current_head_y, pico_left_track_speed
    global pico_right_track_speed, pico_led_state, pico_manual_mode
    global pico_serial_connected

    if not ser or not ser.is_open:
        if pico_serial_connected: print("Serial disconnected during read.")
        pico_serial_connected = False
        return

    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue

            if line.startswith("STATUS:"):
                pico_last_status_str = line
                if pico_reporting_enabled: print(f"PICO_STATE: {line[7:]}")

                parts = line[7:].split(',')
                if len(parts) >= 6:
                    try:
                        pico_left_track_speed = int(parts[0])
                        pico_right_track_speed = int(parts[1])
                        pico_current_head_x = float(parts[2])
                        pico_current_head_y = float(parts[3])
                        pico_led_state = int(parts[4])
                        pico_manual_mode = bool(int(parts[5]))
                    except (ValueError, IndexError):
                        print(f"Warning: Could not parse STATUS line: {line}")
                        pico_last_status_str = "Parse Error"
                else:
                    pico_last_status_str = "Incomplete Status"

            elif line.startswith("OK: Status Reporting ENABLED"):
                pico_reporting_enabled = True; print(f"Pico: {line}")
            elif line.startswith("OK: Status Reporting DISABLED"):
                pico_reporting_enabled = False; print(f"Pico: {line}")
            elif line.startswith("OK:") or line.startswith("ERR:"):
                print(f"Pico: {line}")

        except serial.SerialException as e:
            print(f"Serial read error: {e}"); pico_serial_connected = False; ser.close(); break
        except Exception as e: print(f"Error processing serial response: {e}")

def send_command(cmd):
    """Sends command string to Pico via serial if connected."""
    global last_sent_command, pico_serial_connected
    if not ser or not ser.is_open:
        if pico_serial_connected: print(f"Serial not connected. Cannot send: {cmd}")
        pico_serial_connected = False; return
    if not cmd: return

    try:
        ser.write((cmd + '\n').encode('utf-8'))
        last_sent_command = cmd
        # Only print non-spammy commands
        if cmd not in ['s', 'f', 'r', 'lt', 'rt', 'rstatus'] and not cmd.startswith('hx') and not cmd.startswith('hy'):
            print(f"Sent: {cmd}")
    except serial.SerialException as e:
        print(f"Serial send error for command '{cmd}': {e}"); pico_serial_connected = False; ser.close()
    except Exception as e: print(f"Error sending serial command '{cmd}': {e}")

def apply_deadzone(value, threshold=DEADZONE):
    """Applies a deadzone threshold to an axis value."""
    if abs(value) < threshold: return 0.0
    return value

def read_terminal_input(q):
    """Reads terminal input in a separate thread and puts lines into a queue."""
    # print("Terminal input thread started.") # Optional debug
    while True:
        try:
            line = sys.stdin.readline().strip()
            if line: q.put(line)
            else: time.sleep(0.1)
        except: break
    # print("Terminal input thread finished.") # Optional debug

# --- Socket Server Functions ---
sel = selectors.DefaultSelector() # Global selector object

def setup_server_socket(host, port):
    """Creates and sets up the non-blocking server socket."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen()
    server_sock.setblocking(False) # Make the server socket non-blocking
    sel.register(server_sock, selectors.EVENT_READ, data=accept_connection)
    print(f"Command server listening on {host}:{port}")
    return server_sock

def accept_connection(server_sock):
    """Accepts a new client connection."""
    try:
        conn, addr = server_sock.accept()  # Should be ready to accept
        print(f"Accepted connection from {addr}")
        conn.setblocking(False)
        sel.register(conn, selectors.EVENT_READ, data=handle_client_data)
    except Exception as e:
        print(f"Error accepting connection: {e}")

def handle_client_data(conn):
    """Handles receiving data from a client socket."""
    recv_data = None
    try:
        recv_data = conn.recv(1024)  # Should be ready to read
        if recv_data:
            command = recv_data.decode('utf-8').strip()
            if command:
                print(f"Received command via socket: '{command}'")
                send_command(command) # Send the command to the Pico
            else:
                # print("Received empty data, closing client connection.") # Optional debug
                sel.unregister(conn)
                conn.close()
        else:
            # No data received usually means the client disconnected
            # print("Client disconnected.") # Optional debug
            sel.unregister(conn)
            conn.close()
    except ConnectionResetError:
        # print("Client connection reset.") # Optional debug
        sel.unregister(conn)
        conn.close()
    except Exception as e:
        print(f"Error handling client data: {e}")
        try: # Attempt cleanup
            sel.unregister(conn)
            conn.close()
        except: pass

# --- Start Input Thread ---
input_thread = threading.Thread(target=read_terminal_input, args=(command_queue,), daemon=True)
input_thread.start()

# --- Main Loop Setup ---
print("\nRobot control active!")
if controller:
    print("Joystick Control:")
    print(" L/R sticks Y = Tracks | D-pad = Head | X=Light | A=Status | B=Center Head")
print("\n**MANUAL INPUT ACTIVE:** Type commands (e.g., hx 180, ltoggle) and press Enter.")
print("Press Ctrl+C to exit\n")

head_update_counter = 0
server_socket = None # Initialize server socket variable

# Setup Server Socket Before Loop
try:
    server_socket = setup_server_socket(SERVER_HOST, SERVER_PORT)
except Exception as e:
    print(f"FATAL: Could not start command server: {e}")
    # Consider exiting if the server is critical
    # sys.exit()

# --- Main Loop ---
try:
    while True:
        loop_start_time = time.monotonic()

        # --- Check for Socket Events (Non-blocking) ---
        events = sel.select(timeout=0)
        for key, mask in events:
            callback = key.data # accept_connection or handle_client_data
            callback(key.fileobj) # Call it with the socket object

        # --- Check for Manual Terminal Commands ---
        try:
            while not command_queue.empty(): send_command(command_queue.get_nowait())
        except queue.Empty: pass

        # --- Process Controller Input ---
        pygame.event.pump() # Update Pygame event queue

        head_update_counter += 1
        # Initialize dictionary to hold current controller state for this loop
        ctl_state = {
            "left_stick_x": 0.0, "left_stick_y": 0.0,
            "right_stick_x": 0.0, "right_stick_y": 0.0,
            "left_trigger_axis": -1.0, "right_trigger_axis": -1.0,
            "dpad_x": 0, "dpad_y": 0,
            "button_b": False, "button_a": False, "button_x": False, "button_y": False,
            "button_capture": False,
            "bumper_l": False, "bumper_r": False,
            "trigger_zl": False, "trigger_zr": False,
            "minus": False, "plus": False, "home": False,
            "stick_l_click": False, "stick_r_click": False,
        }

        if controller:
            # --- Read Axes ---
            if num_axes > LEFT_STICK_X: ctl_state["left_stick_x"] = apply_deadzone(controller.get_axis(LEFT_STICK_X))
            if num_axes > LEFT_STICK_Y: ctl_state["left_stick_y"] = apply_deadzone(controller.get_axis(LEFT_STICK_Y))
            if num_axes > RIGHT_STICK_X: ctl_state["right_stick_x"] = apply_deadzone(controller.get_axis(RIGHT_STICK_X))
            if num_axes > RIGHT_STICK_Y: ctl_state["right_stick_y"] = apply_deadzone(controller.get_axis(RIGHT_STICK_Y))
            if num_axes > LEFT_TRIGGER_AXIS: ctl_state["left_trigger_axis"] = controller.get_axis(LEFT_TRIGGER_AXIS)
            if num_axes > RIGHT_TRIGGER_AXIS: ctl_state["right_trigger_axis"] = controller.get_axis(RIGHT_TRIGGER_AXIS)

            # --- Read Buttons ---
            button_map = {
                B_BUTTON: "button_b", A_BUTTON: "button_a", X_BUTTON: "button_x", Y_BUTTON: "button_y",
                CAPTURE_BUTTON: "button_capture", L_BUMPER: "bumper_l", R_BUMPER: "bumper_r",
                ZL_TRIGGER: "trigger_zl", ZR_TRIGGER: "trigger_zr", MINUS_BUTTON: "minus",
                PLUS_BUTTON: "plus", HOME_BUTTON: "home", LEFT_STICK_CLICK: "stick_l_click",
                RIGHT_STICK_CLICK: "stick_r_click"
            }
            for btn_index, key_name in button_map.items():
                if num_buttons > btn_index:
                    ctl_state[key_name] = controller.get_button(btn_index)

            # --- Read Hat (D-Pad) ---
            if num_hats > DPAD_HAT:
                dpad_state = controller.get_hat(DPAD_HAT)
                ctl_state["dpad_x"] = dpad_state[0]
                ctl_state["dpad_y"] = dpad_state[1]

            # === Process Inputs ===

            # --- Movement Control (Sticks) ---
            track_command = 's' # Default to stop
            left_y = ctl_state["left_stick_y"]
            right_y = ctl_state["right_stick_y"]
            if left_y < -0.5 and right_y < -0.5: track_command = 'f'
            elif left_y > 0.5 and right_y > 0.5: track_command = 'r'
            elif left_y > 0.5 and right_y < -0.5: track_command = 'lt'
            elif left_y < -0.5 and right_y > 0.5: track_command = 'rt'

            if track_command != last_track_command:
                send_command(track_command)
                last_track_command = track_command

            # --- Head Control (D-pad - Rate Limited) ---
            if head_update_counter >= HEAD_UPDATE_RATE:
                head_update_counter = 0 # Reset counter
                target_x = current_head_x
                target_y = current_head_y

                dpad_x = ctl_state["dpad_x"]
                dpad_y = ctl_state["dpad_y"]

                # Pan (X-axis) - Controller Left moves Robot Right (+)
                if dpad_x == -1: target_x = min(270.0, current_head_x + HEAD_STEP)
                elif dpad_x == 1: target_x = max(90.0, current_head_x - HEAD_STEP)

                # Tilt (Y-axis) - Controller Up moves Robot Up (+)
                if dpad_y == 1: target_y = min(260.0, current_head_y + HEAD_STEP)
                elif dpad_y == -1: target_y = max(120.0, current_head_y - HEAD_STEP)

                # Send commands only if target actually changed
                if target_x != current_head_x:
                    current_head_x = target_x
                    send_command(f'hx {current_head_x:.1f}')

                if target_y != current_head_y:
                    current_head_y = target_y
                    send_command(f'hy {current_head_y:.1f}')

            # --- Assigned Button Commands (Edge Triggered) ---
            # Send command only on the initial press
            a_pressed = ctl_state["button_a"]
            if a_pressed and not last_button_state.get(A_BUTTON, False):
                send_command('rtoggle')
            last_button_state[A_BUTTON] = a_pressed

            x_pressed = ctl_state["button_x"]
            if x_pressed and not last_button_state.get(X_BUTTON, False):
                send_command('ltoggle')
            last_button_state[X_BUTTON] = x_pressed

            b_pressed = ctl_state["button_b"]
            if b_pressed and not last_button_state.get(B_BUTTON, False):
                current_head_x = 180.0
                current_head_y = 180.0
                send_command(f'hx {current_head_x}')
                send_command(f'hy {current_head_y}')
            last_button_state[B_BUTTON] = b_pressed

            # --- Update last_button_state for ALL other buttons (for JSON state) ---
            other_buttons_indices = [
                Y_BUTTON, CAPTURE_BUTTON, L_BUMPER, R_BUMPER, ZL_TRIGGER, ZR_TRIGGER,
                MINUS_BUTTON, PLUS_BUTTON, HOME_BUTTON, LEFT_STICK_CLICK, RIGHT_STICK_CLICK
            ]
            for btn_index in other_buttons_indices:
                 # Find the key name from the map
                 key_name = next((k for i, k in button_map.items() if i == btn_index), None)
                 if key_name:
                    is_pressed = ctl_state.get(key_name, False)
                    # No command sent here, just update the last state for next loop's edge detection
                    last_button_state[btn_index] = is_pressed
            # --- End Input Processing ---

        # --- Read responses that arrived from Pico ---
        read_response()

        # --- Request Status from Pico Periodically ---
        if pico_serial_connected and (loop_start_time - last_status_request_time > STATUS_REQUEST_INTERVAL):
            send_command('rstatus')
            last_status_request_time = loop_start_time

        # --- Gather State Data for JSON ---
        control_state = {
            "timestamp_utc": datetime.datetime.utcnow().isoformat(),
            "controller_connected": controller is not None,
            "controller_name": controller_name,
            "inputs": ctl_state, # Contains all current controller input states
            "command_state": {
                "last_track_command_sent": last_track_command,
                "head_pan_target_sent": round(current_head_x, 1),
                "head_tilt_target_sent": round(current_head_y, 1),
                "last_command_sent": last_sent_command,
            },
            "pico_state": {
                "serial_connected": pico_serial_connected,
                "reporting_enabled": pico_reporting_enabled,
                "status_raw": pico_last_status_str,
                "left_track_speed": pico_left_track_speed,
                "right_track_speed": pico_right_track_speed,
                "head_pan_actual": pico_current_head_x,
                "head_tilt_actual": pico_current_head_y,
                "led_state": pico_led_state,
                "in_manual_mode": pico_manual_mode
            }
        }

        # --- Write State to JSON file in RAM ---
        try:
            temp_file_path = CONTROL_JSON_PATH + ".tmp"
            with open(temp_file_path, 'w') as f:
                json.dump(control_state, f, indent=2)
            os.rename(temp_file_path, CONTROL_JSON_PATH)
        except Exception as e:
            print(f"Error writing control state JSON: {e}")

        # --- Loop Timing ---
        loop_end_time = time.monotonic()
        loop_duration = loop_end_time - loop_start_time
        sleep_time = (1.0 / LOOP_RATE_HZ) - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)
        # else: print(f"Warning: Control loop took too long: {loop_duration:.4f}s") # Optional debug


except KeyboardInterrupt:
    print("\n\nCtrl+C detected. Stopping robot...")
    send_command('s') # Attempt to send stop command
    if pico_reporting_enabled:
        send_command('rtoggle') # Attempt to disable reporting
        time.sleep(0.1)

finally:
    # Ensure resources are cleaned up robustly
    print("Cleaning up resources...")
    if server_socket:
        try:
            sel.unregister(server_socket)
            server_socket.close()
            print("Server socket closed.")
        except Exception as e:
            print(f"Error closing server socket: {e}")
    try:
        sel.close() # Close the selector
        print("Selector closed.")
    except Exception as e:
        print(f"Error closing selector: {e}")

    if ser and ser.is_open:
        try:
            ser.close(); print("Serial port closed.")
        except Exception as e:
            print(f"Error closing serial port: {e}")
    if controller: # Check if controller object exists
        try: pygame.joystick.quit()
        except Exception as e: print(f"Error quitting joystick: {e}")
    if pygame.get_init(): # Check if pygame itself was initialized
         try: pygame.quit(); print("Pygame quit.")
         except Exception as e: print(f"Error quitting pygame: {e}")
    print("Exited.")