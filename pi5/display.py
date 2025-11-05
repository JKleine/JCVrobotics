#!/usr/bin/python
# -*- coding:utf-8 -*-
import os
# Hide Pygame welcome message before importing pygame
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
# Set libcamera log level (not strictly needed, but harmless)
os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

import pygame
import sys
import math
import psutil  # For system stats
import random
import time
import threading
import queue
import json # For reading sensor data
import socket
import selectors
import datetime
from dataclasses import dataclass, field

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
ABSOLUTE_NPY_PATH = os.path.join(SCRIPT_DIR, "pygame_frame.npy")

# --- MODIFIED: We only need numpy now ---
import numpy as np

# --- Global Paths ---
CONTROL_JSON_PATH = "/dev/shm/control_state.json"
SENSOR_JSON_PATH = "/dev/shm/sensor_data.json"

# --- NEW: Path to the frame from visionstreamer.py ---
# This MUST match the path in visionstreamer.py
FRAME_NPY_PATH = ABSOLUTE_NPY_PATH


BROW_DEFAULT_THICKNESS = 8
BROW_DEFAULT_WIDTH = 150
BROW_DEFAULT_HEIGHT_OFFSET = -100
BROW_DEFAULT_ANGLE = 15
BROW_DEFAULT_CURVATURE = 0.3

# --- 
# --- ENTIRE CameraThread CLASS (lines ~83-176) HAS BEEN DELETED ---
# --- 


# ---
# --- NEW: FrameReaderThread ---
# ---
class FrameReaderThread(threading.Thread):
    """
    Reads the NumPy frame file created by visionstreamer.py,
    converts it to a Pygame Surface, and puts it in the queue.
    """
    def __init__(self, frame_queue, width, height, npy_path):
        super().__init__()
        self.daemon = True
        self.frame_queue = frame_queue
        self.running = True
        self.w = width
        self.h = height
        self.npy_path = npy_path
        
        # Create a template surface to blit into.
        # This matches the 24-bit RGB format.
        self.template_surface = pygame.Surface((self.w, self.h), 0, 24)
        
        print(f"[FrameReader] Thread started. Watching: {self.npy_path}")

    def run(self):
        """Main loop for reading frames."""
        while self.running:
            try:
                # 1. Load the NumPy array file
                frame_rgb = np.load(self.npy_path, allow_pickle=False) # Be explicit

                if frame_rgb.shape != (self.h, self.w, 3):
                    # This should no longer happen, but it's good to check
                    print(f"[FrameReader] Warning: Mismatched shape. Got {frame_rgb.shape}")
                    time.sleep(1/30)
                    continue

                # 2. Convert from NumPy (H, W, 3) to Pygame (W, H, 3)
                frame_whc_contiguous = np.ascontiguousarray(frame_rgb.transpose(1, 0, 2))
                
                # 3. Blit data into the surface
                surface_pixels = pygame.surfarray.pixels3d(self.template_surface)
                surface_pixels[...] = frame_whc_contiguous
                del surface_pixels # Release the lock
                
                # 4. Put a *copy* of the surface into the queue
                if self.frame_queue.empty():
                    self.frame_queue.put_nowait(self.template_surface.copy())

                time.sleep(1/30) # ~30 FPS

            except FileNotFoundError:
                # This is normal if visionstreamer.py hasn't started
                time.sleep(0.5)
            except Exception as e:
                # This will catch errors if we try to read a half-written file
                # or a 0-byte file. We just ignore it and try again.
                time.sleep(1/60) # Try again very quickly

    def stop(self):
        """Signals the thread to stop."""
        self.running = False
# ---
# --- END NEW FrameReaderThread ---
# ---


# --- Main application class ---
class LCARSDisplayApp:
    # ... (All layout, color, and status constants are UNCHANGED) ...
    DISPLAY_SERVER_HOST = '127.0.0.1'
    DISPLAY_SERVER_PORT = 65002
    DISPLAY_WIDTH = 800
    DISPLAY_HEIGHT = 480
    FACE_HEIGHT = int(DISPLAY_HEIGHT * 0.70)
    INFO_HEIGHT = DISPLAY_HEIGHT - FACE_HEIGHT
    HUD_Y_PADDING = 40
    BLOCK_WIDTH = 250
    BLOCK_HEIGHT = 30
    BLOCK_SPACING = 3
    HUD_ROW_Y = FACE_HEIGHT + 5
    HUD_ROW_WIDTH = 187
    HUD_ROW_SPACING = 4
    x_col1 = 20
    y_start_hud = FACE_HEIGHT + HUD_Y_PADDING
    CAMERA_BLOCK_WIDTH = BLOCK_WIDTH
    CAMERA_BLOCK_HEIGHT = 3 * BLOCK_HEIGHT + 2 * BLOCK_SPACING
    CAMERA_BOX_X = x_col1
    CAMERA_BOX_Y = y_start_hud
    EYE_Y_OFFSET = -30
    EYE_RADIUS = 80
    EYE_SPACING = 200
    MOUTH_Y_OFFSET = EYE_RADIUS + 15
    MOUTH_WIDTH = 250
    MOUTH_ARC_HEIGHT = 25
    DEFAULT_MOUTH_THICKNESS = 8
    MANUAL_EYE_MAX_OFFSET = 30
    RANDOM_EYE_MAX_OFFSET = 3
    MOUTH_OVERRIDE_TIMEOUT = 0.2
    ACCENT_DURATION = 1.5
    ACCENT_FADE_IN_TIME = 0.2
    ACCENT_FADE_OUT_TIME = 0.5
    ACCENT_Y_OFFSET = -100
    ACCENT_X_OFFSET = 120
    BLUSH_Y_OFFSET = 110
    BLUSH_X_OFFSET = -50
    BLUSH_RADIUS_X = 50
    BLUSH_RADIUS_Y = 25
    BLUSH_MAX_ALPHA = 100
    BLUSH_COLOR = (255, 182, 193)
    BLACK = (0, 0, 0)
    LCARS_BG_DARK = (20, 20, 40)
    LCARS_MAGENTA = (153, 102, 153)
    LCARS_ORANGE = (255, 153, 102)
    LCARS_BLUE = (102, 153, 204)
    LCARS_RED = (204, 51, 51)
    LCARS_YELLOW = (255, 204, 102)
    LCARS_WHITE = (204, 204, 204)
    LCARS_LIGHT_GRAY = (200, 200, 225)
    LCARS_DARK_GRAY = (100, 100, 125)
    STATUS_CONFIG = {
        'IDLE': {'color': LCARS_MAGENTA, 'mouth_type': 'FLAT', 'status_color': LCARS_MAGENTA, 'eye_type': 'NORMAL', 'curvature': 0},
        'ERROR': {'color': LCARS_RED, 'mouth_type': 'DIAGONAL', 'status_color': LCARS_RED, 'eye_type': 'NORMAL', 'curvature': 0},
        'HAPPY': {'color': LCARS_YELLOW, 'mouth_type': 'SMILE', 'status_color': LCARS_ORANGE, 'eye_type': 'NORMAL', 'curvature': 1},
        'SAD': {'color': LCARS_BLUE, 'mouth_type': 'FROWN', 'status_color': LCARS_BLUE, 'eye_type': 'NORMAL', 'curvature': -1},
        'ANGRY': {'color': LCARS_RED, 'mouth_type': 'FROWN', 'status_color': LCARS_RED, 'eye_type': 'NORMAL', 'curvature': -1},
        'SLEEP': {'color': LCARS_WHITE, 'mouth_type': 'FLAT', 'status_color': LCARS_WHITE, 'eye_type': 'SLEEP', 'curvature': 0}
    }
    STATUS_DISPLAY_LABELS = {'SLEEP': 'ASLEEP', 'SAD': 'SAD'}

    # ... (AppState dataclass is UNCHANGED) ...
    @dataclass
    class AppState:
        status: str = 'IDLE'
        target_status: str = 'IDLE'
        battery_percent: float = 50.0
        is_charging: bool = False
        temp_c: float = 25.0
        humidity: float = 50.0
        pressure: float = 1013.25
        eye_target: tuple = (0, 0)
        is_animating: bool = False
        animation_start_time: float = 0.0
        is_sleep_animating: bool = False
        sleep_animation_start_time: float = 0.0
        sleep_animation_dir: int = 0
        environment: dict = field(default_factory=dict)
        motion: dict = field(default_factory=dict)
        light: dict = field(default_factory=dict)
        adc_volts: dict = field(default_factory=dict)
        power: dict = field(default_factory=dict)
        inputs: dict = field(default_factory=dict)
        command_state: dict = field(default_factory=dict)
        pico_state: dict = field(default_factory=dict)
        controller_connected: bool = False
        controller_name: str = 'N/A'
        speech_mouth_override: str = None
        mouth_thickness_override: int = None
        mouth_curvature_override: float = None
        last_mouth_command_time: float = 0.0
        wave_amplitude: int = 0
        wave_frequency: int = 5
        brows_enabled: bool = False
        brow_thickness: int = BROW_DEFAULT_THICKNESS
        brow_width: int = BROW_DEFAULT_WIDTH
        brow_height_offset: int = BROW_DEFAULT_HEIGHT_OFFSET
        brow_angle: int = BROW_DEFAULT_ANGLE
        brow_curvature: float = BROW_DEFAULT_CURVATURE
        blush_enabled: bool = False
        blush_alpha: float = 0.0
        blush_target_alpha: float = 0.0
        blush_animation_speed: float = 150.0
        accent_list: list = field(default_factory=list)
        cpu_load: float = 0.0
        mem_load: float = 0.0
        disk_load: float = 0.0

    def __init__(self):
        pygame.init()
        
        # ... (Font initialization is UNCHANGED) ...
        self.font_large = None
        self.font_medium = None
        self.font_small = None
        self._initialize_fonts()
        
        self.screen = pygame.display.set_mode(
            (self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT), 
            pygame.FULLSCREEN | pygame.SCALED, 
            depth=32
        )
        pygame.display.set_caption("Project J1 - CV Robotics Platform")
        
        self.clock = pygame.time.Clock()
        self.running = True
        self.last_movement_update = time.time()
        
        # ... (Timers are UNCHANGED) ...
        self.last_json_read_time = 0.0
        self.json_read_interval = 0.1
        self.last_stats_read_time = 0.0
        self.stats_read_interval = 1.0
        self.default_animation_duration = 0.3
        self.sleep_animation_duration = 0.5
        
        self.state = self.AppState()
        
        # ... (Mouth drawers are UNCHANGED) ...
        self.mouth_drawers = {
            'SMILE': self._draw_mouth_arc,
            'FROWN': self._draw_mouth_arc,
            'FLAT': self._draw_mouth_line,
            'CLOSED': self._draw_mouth_line,
            'DIAGONAL': self._draw_mouth_diagonal,
            'WAVE': self._draw_mouth_wave,
            'ROUND': self._draw_mouth_round,
        }

        # --- MODIFIED: Queues, Events, Threads ---
        self.command_queue = queue.Queue()
        self.startup_complete_event = threading.Event()
        self.camera_frame_queue = queue.Queue(maxsize=1)
        self.camera_surface = None # Holds the latest surface

        self.terminal_thread = threading.Thread(
            target=self._terminal_reader_thread,
            daemon=True
        )
        
        # --- MODIFIED: Start the new FrameReaderThread ---
        # The old CameraThread and its 'if CAMERA_PRESENT' logic is gone.
        self.frame_reader_thread = FrameReaderThread(
            self.camera_frame_queue, 
            width=640, 
            height=480,
            npy_path=FRAME_NPY_PATH
        )
        self.frame_reader_thread.start()
        # --- END MODIFICATION ---

        # ... (Socket server setup is UNCHANGED) ...
        self.display_sel = selectors.DefaultSelector()
        self.display_server_socket = None
        try:
            self._setup_display_server_socket(
                self.DISPLAY_SERVER_HOST, 
                self.DISPLAY_SERVER_PORT
            )
        except Exception as e:
            print(f"WARNING: Could not start display command server: {e}")

        psutil.cpu_percent(interval=None) # Prime the CPU % call
        self.terminal_thread.start()
        self.startup_complete_event.set()

    #
    # ... (ALL OTHER METHODS from _initialize_fonts to main_loop are UNCHANGED) ...
    #
    
    def _initialize_fonts(self):
        """Set up Pygame fonts with simple fallbacks."""
        try:
            self.font_large = pygame.font.Font(None, 64)
            self.font_medium = pygame.font.Font(None, 36)
            self.font_small = pygame.font.Font(None, 24)
        except Exception:
            self.font_large = pygame.font.SysFont('Arial', 64, bold=True)
            self.font_medium = pygame.font.SysFont('Arial', 36, bold=True)
            self.font_small = pygame.font.SysFont('Arial', 24, bold=True)

    @staticmethod
    def _interpolate_color(color1, color2, ratio):
        """Linear interpolation between two RGB colors."""
        r = int(color1[0] + (color2[0] - color1[0]) * ratio)
        g = int(color1[1] + (color2[1] - color1[1]) * ratio)
        b = int(color1[2] + (color2[2] - color1[2]) * ratio)
        return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))

    def _draw_eye(self, screen, center_x, center_y, radius, pupil_offset, eye_progress, line_color):
        """Render one eye based on openness and pupil position."""
        if eye_progress >= 0.5:
            scaled_progress = (eye_progress - 0.5) * 2
            current_eye_height = int(radius * scaled_progress)
            eye_rect = pygame.Rect(center_x - radius, center_y - current_eye_height, radius * 2, current_eye_height * 2)
            pygame.draw.ellipse(screen, self.LCARS_WHITE, eye_rect)

            if eye_progress >= 0.95:
                pupil_radius = int(radius * 0.4)
                pupil_x = center_x + pupil_offset[0]
                pupil_y = center_y + pupil_offset[1]

                distance = math.sqrt((pupil_x - center_x) ** 2 + (pupil_y - center_y) ** 2)
                max_dist = radius - pupil_radius - 2
                if distance > max_dist and distance > 0:
                    ratio = max_dist / distance
                    pupil_x = center_x + int((pupil_x - center_x) * ratio)
                    pupil_y = center_y + int((pupil_y - center_y) * ratio)

                pygame.draw.circle(screen, self.BLACK, (pupil_x, pupil_y), pupil_radius)
                highlight_radius = int(pupil_radius * 0.35)
                highlight_offset_x = int(pupil_radius * 0.3)
                highlight_offset_y = int(pupil_radius * -0.3)
                pygame.draw.circle(screen, self.LCARS_WHITE, (pupil_x + highlight_offset_x, pupil_y + highlight_offset_y), highlight_radius)

        if eye_progress <= 0.5:
            sleep_line_width = 100
            sleep_line_height_offset = 5
            line_thickness = 10
            line_y = center_y + sleep_line_height_offset
            fade_ratio = max(0.0, min(1.0, 1.0 - (eye_progress * 2)))
            color_to_use = self._interpolate_color(line_color, self.LCARS_WHITE, fade_ratio)
            pygame.draw.line(screen, color_to_use, (center_x - sleep_line_width // 2, line_y), (center_x + sleep_line_width // 2, line_y), line_thickness)

    def _draw_lcars_block(self, screen, x, y, width, height, label, value, color, font_label, block_type='DATA', value_data=None):
        """Render an LCARS data block or segmented meter."""
        RADIUS = height // 2
        TEXT_PADDING = 20 # General padding for text
        TEXT_FONT = font_label
        TEXT_HEIGHT = TEXT_FONT.get_height()
        TEXT_Y = y + (height / 2) - (TEXT_HEIGHT / 2)

        if color in [self.LCARS_ORANGE]:
            label_color = self.LCARS_DARK_GRAY
            value_color = self.BLACK
        elif color in [self.LCARS_YELLOW]:
            label_color = self.LCARS_DARK_GRAY
            value_color = self.BLACK
        else:
            label_color = self.LCARS_WHITE
            value_color = self.BLACK

        try:
            # Use border_radius for full pill shape
            pygame.draw.rect(screen, color, (x, y, width, height), border_radius=RADIUS)
        except Exception:
            # Fallback
            pygame.draw.rect(screen, color, (x, y, width, height))

        if block_type == 'DATA':
            label_text = TEXT_FONT.render(label, True, label_color)
            screen.blit(label_text, (x + TEXT_PADDING, TEXT_Y))
            if value:
                value_text = TEXT_FONT.render(value, True, value_color)
                screen.blit(value_text, (x + width - value_text.get_width() - TEXT_PADDING, TEXT_Y))
        
        elif block_type == 'METER':
            METER_PADDING_X = 20
            METER_PADDING_Y = 5 

            percentage = value_data[0] if isinstance(value_data, tuple) else value_data
            is_charging = value_data[1] if isinstance(value_data, tuple) else False

            if percentage < 15: fill_color = self.LCARS_RED
            elif percentage < 30 and not is_charging: fill_color = self.LCARS_ORANGE
            elif is_charging: fill_color = self.LCARS_YELLOW
            else: fill_color = self.LCARS_MAGENTA

            NUM_SEGMENTS = 20
            SEGMENT_GAP = 2
            total_gap_space = (NUM_SEGMENTS - 1) * SEGMENT_GAP
            
            SEG_WIDTH = (width - (METER_PADDING_X * 2) - total_gap_space) // NUM_SEGMENTS 
            if SEG_WIDTH < 1: 
                NUM_SEGMENTS = (width - (METER_PADDING_X * 2)) // (1 + SEGMENT_GAP)
                if NUM_SEGMENTS < 1: NUM_SEGMENTS = 1
                total_gap_space = (NUM_SEGMENTS - 1) * SEGMENT_GAP
                SEG_WIDTH = (width - (METER_PADDING_X * 2) - total_gap_space) // NUM_SEGMENTS
                if SEG_WIDTH < 1: SEG_WIDTH = 1

            SEG_HEIGHT = height - (METER_PADDING_Y * 2)
            if SEG_HEIGHT < 1: SEG_HEIGHT = 1
            
            segments_to_fill = math.ceil(percentage / (100 / NUM_SEGMENTS))

            for i in range(NUM_SEGMENTS):
                seg_x = x + METER_PADDING_X + (i * (SEG_WIDTH + SEGMENT_GAP))
                seg_y = y + METER_PADDING_Y
                if i < segments_to_fill:
                    pygame.draw.rect(screen, fill_color, (seg_x, seg_y, SEG_WIDTH, SEG_HEIGHT))
                else:
                    pygame.draw.rect(screen, self.BLACK, (seg_x, seg_y, SEG_WIDTH, SEG_HEIGHT))
                    pygame.draw.rect(screen, self.LCARS_BG_DARK, (seg_x, seg_y, SEG_WIDTH, SEG_HEIGHT), 1)

    def _parse_command(self, command_full):
        if not command_full:
            return None
            
        command_parts = command_full.upper().split()
        command = command_parts[0]

        if command == 'EYE' and len(command_parts) == 3:
            try:
                x_offset = int(command_parts[1])
                y_offset = int(command_parts[2])
                x_offset = max(-self.MANUAL_EYE_MAX_OFFSET, min(self.MANUAL_EYE_MAX_OFFSET, x_offset))
                y_offset = max(-self.MANUAL_EYE_MAX_OFFSET, min(self.MANUAL_EYE_MAX_OFFSET, y_offset))
                return ('EYE', x_offset, y_offset)
            except ValueError:
                print("  -> Invalid EYE command format. Use: EYE X Y (integers)")
                return None

        elif command == 'MOUTH' and (len(command_parts) == 2 or len(command_parts) == 4):
            mouth_shape = command_parts[1]
            if len(command_parts) == 4:
                try:
                    thickness = int(command_parts[2])
                    curvature = float(command_parts[3])
                    return ('MOUTH', mouth_shape, thickness, curvature)
                except ValueError:
                    print("  -> Invalid MOUTH command. Use: MOUTH SHAPE or MOUTH SHAPE <int> <float>")
                    return None
            else:
                return ('MOUTH', mouth_shape, None, None) 

        elif command == 'WAVE' and len(command_parts) == 3:
            try:
                amplitude = int(command_parts[1])
                frequency = int(command_parts[2])
                return ('WAVE', amplitude, frequency)
            except ValueError:
                print("  -> Invalid WAVE command. Use: WAVE <int_amplitude> <int_frequency>")
                return None

        elif command == 'BROW' and len(command_parts) >= 2:
            toggle = command_parts[1]
            if toggle not in ['ON', 'OFF']:
                print("  -> Invalid BROW command. Use: BROW ON/OFF [thick width height_offset angle curve]")
                return None
            
            is_on = (toggle == 'ON')
            
            if len(command_parts) == 7:
                try:
                    thick = int(command_parts[2])
                    width = int(command_parts[3])
                    height_offset = int(command_parts[4])
                    angle = int(command_parts[5])
                    curve = float(command_parts[6])
                    return ('BROW', is_on, thick, width, height_offset, angle, curve)
                except ValueError:
                    print("  -> Invalid BROW parameters. Use: BROW ON/OFF <int> <int> <int> <int> <float>")
                    return None
            elif len(command_parts) == 2:
                # Use defaults
                return ('BROW', is_on, BROW_DEFAULT_THICKNESS, BROW_DEFAULT_WIDTH, BROW_DEFAULT_HEIGHT_OFFSET, BROW_DEFAULT_ANGLE, BROW_DEFAULT_CURVATURE)
            else:
                print("  -> Invalid BROW command. Use: BROW ON/OFF [thick width height_offset angle curve]")
                return None

        elif command == 'BLUSH' and len(command_parts) == 2:
            toggle = command_parts[1]
            if toggle not in ['ON', 'OFF']:
                print("  -> Invalid BLUSH command. Use: BLUSH ON/OFF")
                return None
            return ('BLUSH', toggle == 'ON')

        elif command == 'ACCENT' and len(command_parts) == 2:
            accent_type = command_parts[1]
            return ('ACCENT', accent_type)

        elif command == 'QUIT' or command in self.STATUS_CONFIG:
            return command

        else:
            print(f"  -> Unknown command: '{command_full}'")
            return None

    def _terminal_reader_thread(self):
        """Reads commands from stdin in a dedicated thread."""
        self.startup_complete_event.wait()
        
        # --- MODIFIED: Removed camera_ready_event ---
        # self.camera_ready_event.wait() 

        print("Terminal input enabled. Type status, EYE X Y, MOUTH... or QUIT.")

        while True:
            try:
                line = sys.stdin.readline()
                if line == '':
                    break # EOF
                
                command_item = self._parse_command(line.strip())
                if command_item:
                    self.command_queue.put(command_item)
                    print(f"  -> Queued command from terminal: {command_item}")

            except EOFError:
                break
            except Exception as e:
                print(f"TerminalReader: Error reading input: {e}")
                break

    # --- Socket Server Methods (UNCHANGED) ---
    def _setup_display_server_socket(self, host, port):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((host, port))
        server_sock.listen()
        server_sock.setblocking(False)
        self.display_sel.register(server_sock, selectors.EVENT_READ, data=self._accept_display_connection)
        self.display_server_socket = server_sock
        print(f"Display command server listening on {host}:{port}")

    def _accept_display_connection(self, server_sock):
        try:
            conn, addr = server_sock.accept()
            conn.setblocking(False)
            self.display_sel.register(conn, selectors.EVENT_READ, data=self._handle_display_client_data)
        except Exception as e:
            print(f"Error accepting display connection: {e}")

    def _handle_display_client_data(self, conn):
        recv_data = None
        try:
            recv_data = conn.recv(1024)
            if recv_data:
                command_full = recv_data.decode('utf-8').strip()
                if command_full:
                    print(f"Received display command via socket: '{command_full}'")
                    command_item = self._parse_command(command_full)
                    if command_item:
                        self.command_queue.put(command_item)
                else:
                    self.display_sel.unregister(conn); conn.close()
            else:
                self.display_sel.unregister(conn); conn.close()
        except ConnectionResetError:
            self.display_sel.unregister(conn); conn.close()
        except Exception as e:
            print(f"Error handling display client data: {e}")
            try: self.display_sel.unregister(conn); conn.close()
            except: pass

    # --- JSON and State Update Methods (UNCHANGED) ---
    def _read_data_json(self):
        sensor_data_loaded = False
        control_data_loaded = False
        sensor_data = {}
        control_data = {}

        try:
            with open(SENSOR_JSON_PATH, 'r') as f:
                sensor_data = json.load(f)
                sensor_data_loaded = True
        except (FileNotFoundError, json.JSONDecodeError): pass
        except Exception as e: print(f"Warning: Error reading {SENSOR_JSON_PATH}: {e}")

        try:
            with open(CONTROL_JSON_PATH, 'r') as f:
                control_data = json.load(f)
                control_data_loaded = True
        except (FileNotFoundError, json.JSONDecodeError): pass
        except Exception as e: print(f"Warning: Error reading {CONTROL_JSON_PATH}: {e}")

        return (sensor_data if sensor_data_loaded else None,
                control_data if control_data_loaded else None)
    
    def _update_state_from_json(self, current_time):
        if current_time - self.last_json_read_time < self.json_read_interval:
            return
        
        self.last_json_read_time = current_time
        sensor_data, control_data = self._read_data_json()

        if sensor_data is not None:
            self.state.environment = sensor_data.get('environment', self.state.environment)
            self.state.motion = sensor_data.get('motion', self.state.motion)
            self.state.light = sensor_data.get('light', self.state.light)
            self.state.adc_volts = sensor_data.get('adc_volts', self.state.adc_volts)
            self.state.power = sensor_data.get('power', self.state.power)
            
            env_data = self.state.environment
            power_data = self.state.power
            
            self.state.temp_c = env_data.get('temperature_shtc_c', self.state.temp_c)
            self.state.humidity = env_data.get('humidity_rh', self.state.humidity)
            self.state.pressure = env_data.get('pressure_hpa', self.state.pressure)
            self.state.battery_percent = power_data.get('battery_percent', self.state.battery_percent)
            
            bus_voltage = power_data.get('bus_voltage_V', 0)
            current_a = power_data.get('current_A', 0)
            self.state.is_charging = bus_voltage > 10.0 and current_a > 0.02

        if control_data is not None:
            self.state.inputs = control_data.get('inputs', self.state.inputs)
            self.state.command_state = control_data.get('command_state', self.state.command_state)
            self.state.pico_state = control_data.get('pico_state', self.state.pico_state)
            self.state.controller_connected = control_data.get('controller_connected', self.state.controller_connected)
            self.state.controller_name = control_data.get('controller_name', self.state.controller_name)

    def _update_system_stats(self):
        try:
            self.state.cpu_load = psutil.cpu_percent(interval=None)
            self.state.mem_load = psutil.virtual_memory().percent
            self.state.disk_load = psutil.disk_usage('/').percent
        except Exception as e:
            print(f"Error reading psutil stats: {e}")
            self.state.cpu_load = -1.0
            self.state.mem_load = -1.0
            self.state.disk_load = -1.0

    # --- Drawing Methods (UNCHANGED) ---
    def _get_face_style(self):
        current_time = time.time()
        state = self.state
        start_config = self.STATUS_CONFIG.get(state.status, self.STATUS_CONFIG['IDLE'])
        end_config = self.STATUS_CONFIG.get(state.target_status, self.STATUS_CONFIG['IDLE'])

        ratio = 0.0
        sleep_ratio = 0.0
        if state.is_animating:
            time_elapsed = current_time - state.animation_start_time
            ratio = min(1.0, time_elapsed / self.default_animation_duration)
        if state.is_sleep_animating:
            sleep_time_elapsed = current_time - state.sleep_animation_start_time
            sleep_ratio = min(1.0, sleep_time_elapsed / self.sleep_animation_duration)
            interpolation_ratio = sleep_ratio
            if state.sleep_animation_dir == 1: eye_progress = 1.0 - sleep_ratio
            else: eye_progress = sleep_ratio
        else:
            interpolation_ratio = ratio if state.is_animating else 1.0
            eye_progress = 0.0 if state.status == 'SLEEP' else 1.0

        if self.state.speech_mouth_override is not None:
                        if current_time - self.state.last_mouth_command_time > self.MOUTH_OVERRIDE_TIMEOUT:
                                self.state.speech_mouth_override = None
                                self.state.mouth_thickness_override = None
                                self.state.mouth_curvature_override = None

        if interpolation_ratio < 1.0:
            line_color = self._interpolate_color(start_config['color'], end_config['color'], interpolation_ratio)
            current_curvature = start_config.get('curvature', 0) + (end_config.get('curvature', 0) - start_config.get('curvature', 0)) * interpolation_ratio
            current_status_color = self._interpolate_color(start_config['status_color'], end_config['status_color'], interpolation_ratio)
            base_mouth_type = end_config.get('mouth_type', 'FLAT')
        else:
            status_key = state.status
            config = self.STATUS_CONFIG.get(status_key, self.STATUS_CONFIG['IDLE'])
            line_color = config['color']
            current_curvature = config.get('curvature', 0)
            current_status_color = config['status_color']
            base_mouth_type = config.get('mouth_type', 'FLAT')

        final_mouth_type = self.state.speech_mouth_override if self.state.speech_mouth_override is not None else base_mouth_type
        
        final_thickness = self.state.mouth_thickness_override if self.state.mouth_thickness_override is not None else self.DEFAULT_MOUTH_THICKNESS

        if self.state.mouth_curvature_override is not None:
            final_curvature = self.state.mouth_curvature_override
        else:
            speech_shapes = ['OPEN', 'SMALL_O', 'WIDE_E']
            final_curvature = current_curvature if final_mouth_type not in speech_shapes else 0

        return {
            'line_color': line_color,
            'mouth_color': self.LCARS_WHITE if eye_progress < 0.1 else line_color,
            'mouth_type': final_mouth_type,
            'mouth_thickness': final_thickness,
            'curvature': final_curvature,
            'eye_progress': eye_progress,
            'status_color': current_status_color
        }

    def _draw_mouth_arc(self, style, center_x, center_y):
        curvature = style['curvature']
        thickness = style['mouth_thickness']
        color = style['mouth_color']
        
        if abs(curvature) > 0.001:
            final_arc_height = int(self.MOUTH_ARC_HEIGHT * abs(curvature))
            arc_rect = pygame.Rect(center_x - self.MOUTH_WIDTH // 2, center_y - final_arc_height, self.MOUTH_WIDTH, final_arc_height * 2)
            if curvature > 0.0: # Smile
                pygame.draw.arc(self.screen, color, arc_rect, math.pi, 2 * math.pi, thickness)
            else: # Frown
                pygame.draw.arc(self.screen, color, arc_rect, 0, math.pi, thickness)
        else:
            self._draw_mouth_line(style, center_x, center_y)

    def _draw_mouth_line(self, style, center_x, center_y):
        thickness = style['mouth_thickness']
        color = style['mouth_color']
        pygame.draw.line(self.screen, color, (center_x - self.MOUTH_WIDTH // 2, center_y), (center_x + self.MOUTH_WIDTH // 2, center_y), thickness)

    def _draw_mouth_diagonal(self, style, center_x, center_y):
        thickness = style['mouth_thickness']
        color = style['mouth_color']
        offset = 30
        pygame.draw.line(self.screen, color, (center_x - self.MOUTH_WIDTH // 2, center_y - offset), (center_x + self.MOUTH_WIDTH // 2, center_y + offset), thickness)

    def _draw_mouth_wave(self, style, center_x, center_y):
        amplitude = self.state.wave_amplitude
        frequency = self.state.wave_frequency
        mouth_color = style['mouth_color']
        
        amplitude = max(0, min(self.EYE_RADIUS * 0.8, amplitude)) 
        if frequency <= 0: frequency = 1 

        num_points = self.MOUTH_WIDTH // 4 
        if num_points < 2: num_points = 2
        
        start_x = center_x - self.MOUTH_WIDTH // 2
        current_time_sec = pygame.time.get_ticks() / 1000.0

        color1 = mouth_color 
        color2 = tuple(max(0, c - 40) for c in mouth_color) 
        color3 = tuple(max(0, c - 80) for c in mouth_color)
        colors = [color1, color2, color3]
        
        phase_shifts = [0, math.pi / 3, 2 * math.pi / 3]
        amp_factors = [1.0, 0.85, 0.7]       
        freq_factors = [1.0, 0.9, 1.15]      
        scroll_factors = [3.0, 2.5, 3.7]   

        for wave_index in range(3):
            wave_points = []
            phase_shift = phase_shifts[wave_index]
            current_amp = amplitude * amp_factors[wave_index]
            current_freq = frequency * freq_factors[wave_index]
            current_scroll = current_time_sec * frequency * scroll_factors[wave_index]

            for i in range(num_points + 1):
                x = start_x + (i * (self.MOUTH_WIDTH / num_points))
                norm_x = i / num_points
                
                angle = (norm_x * current_freq * 2 * math.pi) + current_scroll + phase_shift
                
                envelope = math.sin(math.pi * norm_x)
                y_offset = current_amp * math.sin(angle) * envelope
                y = center_y + y_offset
                
                wave_points.append((x, y))

            if len(wave_points) >= 2:
                wave_thickness = self.DEFAULT_MOUTH_THICKNESS // 2 
                if current_amp < 3:
                        wave_thickness = max(1, wave_thickness // 2)
                else:
                        wave_thickness = max(2, wave_thickness) 
                
                pygame.draw.lines(self.screen, colors[wave_index], False, wave_points, wave_thickness)

    def _draw_mouth_round(self, style, center_x, center_y):
        thickness = style['mouth_thickness']
        color = style['mouth_color']
        o_radius = 30 
        pygame.draw.circle(self.screen, color, (center_x, center_y + 5), o_radius, thickness + 1) 

    def _draw_face(self, current_time):
        state = self.state
        style = self._get_face_style()

        line_color = style['line_color']
        eye_progress = style['eye_progress']
        mouth_to_draw = style['mouth_type']

        self.screen.fill(self.BLACK, (0, 0, self.DISPLAY_WIDTH, self.FACE_HEIGHT))

        face_center_x = self.DISPLAY_WIDTH // 2
        face_center_y = self.FACE_HEIGHT // 2
        left_eye_center = (face_center_x - self.EYE_SPACING, face_center_y + self.EYE_Y_OFFSET)
        right_eye_center = (face_center_x + self.EYE_SPACING, face_center_y + self.EYE_Y_OFFSET)

        self._draw_blush(left_eye_center, right_eye_center)
        self._draw_brows(left_eye_center, right_eye_center)

        self._draw_eye(self.screen, left_eye_center[0], left_eye_center[1], self.EYE_RADIUS, state.eye_target, eye_progress, line_color)
        self._draw_eye(self.screen, right_eye_center[0], right_eye_center[1], self.EYE_RADIUS, state.eye_target, eye_progress, line_color)

        mouth_y = face_center_y + self.MOUTH_Y_OFFSET
        mouth_center_x = self.DISPLAY_WIDTH // 2
        
        draw_function = self.mouth_drawers.get(mouth_to_draw, self._draw_mouth_line)
        
        draw_function(style, mouth_center_x, mouth_y)

        self._draw_accents(current_time, left_eye_center, right_eye_center)

    def _draw_brows(self, left_eye_center, right_eye_center):
        if not self.state.brows_enabled:
            return

        style = self._get_face_style()
        color = style['line_color']
        
        params = {
            'width': self.state.brow_width,
            'thickness': self.state.brow_thickness,
            'height_offset': self.state.brow_height_offset,
            'angle_deg': self.state.brow_angle,
            'curvature': self.state.brow_curvature
        }

        self._draw_single_brow(
            left_eye_center[0], 
            left_eye_center[1] + params['height_offset'], 
            params, 
            color, 
            is_right_side=False
        )
        
        self._draw_single_brow(
            right_eye_center[0], 
            right_eye_center[1] + params['height_offset'], 
            params, 
            color, 
            is_right_side=True
        )

    def _draw_single_brow(self, center_x, center_y, params, color, is_right_side):
        width = params['width']
        thickness = params['thickness']
        angle_rad = math.radians(params['angle_deg'])
        curvature = params['curvature']
        
        if is_right_side:
            angle_rad = -angle_rad

        half_width = width // 2
        
        start_x = center_x - half_width
        start_y = center_y
        end_x = center_x + half_width
        end_y = center_y
        
        def rotate_point(px, py, cx, cy, angle):
            s = math.sin(angle)
            c = math.cos(angle)
            px -= cx
            py -= cy
            x_new = px * c - py * s
            y_new = px * s + py * c
            return x_new + cx, y_new + cy

        start_x_rot, start_y_rot = rotate_point(start_x, start_y, center_x, center_y, angle_rad)
        end_x_rot, end_y_rot = rotate_point(end_x, end_y, center_x, center_y, angle_rad)
        
        if abs(curvature) < 0.01:
            pygame.draw.line(self.screen, color, (start_x_rot, start_y_rot), (end_x_rot, end_y_rot), thickness)
        else:
            control_offset_y = -width * curvature * 0.5
            
            control_x_rot, control_y_rot = rotate_point(center_x, center_y + control_offset_y, center_x, center_y, angle_rad)

            points = []
            num_segments = 20
            for i in range(num_segments + 1):
                t = i / num_segments
                inv_t = 1 - t
                
                x = (inv_t ** 2) * start_x_rot + 2 * inv_t * t * control_x_rot + (t ** 2) * end_x_rot
                y = (inv_t ** 2) * start_y_rot + 2 * inv_t * t * control_y_rot + (t ** 2) * end_y_rot
                points.append((x, y))
            
            if len(points) >= 2:
                pygame.draw.lines(self.screen, color, False, points, thickness)

    def _draw_blush(self, left_eye_center, right_eye_center):
        alpha = self.state.blush_alpha
        if alpha <= 1.0:
            return
            
        blush_surface = pygame.Surface((self.DISPLAY_WIDTH, self.FACE_HEIGHT), pygame.SRCALPHA)
        blush_surface.fill((0, 0, 0, 0))
        
        color_with_alpha = self.BLUSH_COLOR + (int(alpha),)
        
        left_blush_rect = pygame.Rect(
            left_eye_center[0] - self.BLUSH_RADIUS_X + self.BLUSH_X_OFFSET,
            left_eye_center[1] + self.BLUSH_Y_OFFSET - self.BLUSH_RADIUS_Y,
            self.BLUSH_RADIUS_X * 2,
            self.BLUSH_RADIUS_Y * 2
        )
        right_blush_rect = pygame.Rect(
            right_eye_center[0] - self.BLUSH_RADIUS_X - self.BLUSH_X_OFFSET,
            right_eye_center[1] + self.BLUSH_Y_OFFSET - self.BLUSH_RADIUS_Y,
            self.BLUSH_RADIUS_X * 2,
            self.BLUSH_RADIUS_Y * 2
        )
        
        pygame.draw.ellipse(blush_surface, color_with_alpha, left_blush_rect)
        pygame.draw.ellipse(blush_surface, color_with_alpha, right_blush_rect)
        
        self.screen.blit(blush_surface, (0, 0))

    def _draw_single_heart(self, surface, center_x, center_y, size, color_with_alpha):
        if size < 4: size = 4
        
        half_size = size // 2
        quarter_size = size // 4

        left_circle_center = (int(center_x - quarter_size), int(center_y - quarter_size))
        right_circle_center = (int(center_x + quarter_size), int(center_y - quarter_size))
        
        pygame.draw.circle(surface, color_with_alpha, left_circle_center, quarter_size)
        pygame.draw.circle(surface, color_with_alpha, right_circle_center, quarter_size)
        
        p1 = (center_x - half_size, center_y - quarter_size)
        p2 = (center_x + half_size, center_y - quarter_size)
        p3 = (center_x, center_y + half_size)
        pygame.draw.polygon(surface, color_with_alpha, [p1, p2, p3])

    def _draw_accents(self, current_time, left_eye_center, right_eye_center):
        if not self.state.accent_list:
            return

        accent_surface = pygame.Surface((self.DISPLAY_WIDTH, self.FACE_HEIGHT), pygame.SRCALPHA)
        accent_surface.fill((0, 0, 0, 0))

        heart_colors_rgb = [
            (255, 182, 193), # Light Pink (original)
            (255, 105, 180), # Hot Pink
            self.LCARS_RED,  # (204, 51, 51)
            (255, 20, 147)   # Deep Pink
        ]
        
        heart_cluster = [
            (0, 0, 30, 0),     # Main heart
            (-30, 12, 21, 1),   # Left heart (was -20, 8)
            (28, -15, 18, 2),   # Top-right heart (was 18, -10)
            (25, 25, 14, 3)    # Bottom-right heart (was 15, 18)
        ]

        for accent in self.state.accent_list:
            alpha = accent.get('alpha', 0)
            if alpha <= 1.0:
                continue
            
            if accent['side'] == 'LEFT':
                base_center_x = left_eye_center[0] - self.ACCENT_X_OFFSET
                base_center_y = left_eye_center[1] + self.ACCENT_Y_OFFSET
            else:
                base_center_x = right_eye_center[0] + self.ACCENT_X_OFFSET
                base_center_y = right_eye_center[1] + self.ACCENT_Y_OFFSET

            if accent['type'] == 'HEART':
                for h_x, h_y, h_size, h_color_idx in heart_cluster:
                    color_rgb = heart_colors_rgb[h_color_idx % len(heart_colors_rgb)]
                    color_with_alpha = color_rgb + (int(alpha),)
                    
                    heart_center_x = base_center_x + h_x
                    heart_center_y = base_center_y + h_y
                    
                    self._draw_single_heart(
                        accent_surface, 
                        heart_center_x, 
                        heart_center_y, 
                        h_size, 
                        color_with_alpha
                    )

            elif accent['type'] == 'ANGRY':
                color_rgb = self.LCARS_RED
                color_with_alpha = color_rgb + (int(alpha),)
                center_x, center_y = base_center_x, base_center_y

                size = 25
                half_size = size // 2
                offset = int(size * 0.2)
                
                p1_start = (center_x - half_size, center_y - half_size)
                p1_end = (center_x - offset, center_y - offset)
                pygame.draw.line(accent_surface, color_with_alpha, p1_start, p1_end, 6)
                
                p2_start = (center_x + half_size, center_y - half_size)
                p2_end = (center_x + offset, center_y - offset)
                pygame.draw.line(accent_surface, color_with_alpha, p2_start, p2_end, 6)

                p3_start = (center_x - half_size, center_y + half_size)
                p3_end = (center_x - offset, center_y + offset)
                pygame.draw.line(accent_surface, color_with_alpha, p3_start, p3_end, 6)

                p4_start = (center_x + half_size, center_y + half_size)
                p4_end = (center_x + offset, center_y + offset)
                pygame.draw.line(accent_surface, color_with_alpha, p4_start, p4_end, 6)

            else:
                color_with_alpha = self.LCARS_YELLOW + (int(alpha),)
                center_x, center_y = base_center_x, base_center_y

                size = 25
                half_size = size // 2
                pygame.draw.line(accent_surface, color_with_alpha, (center_x, center_y - half_size), (center_x, center_y + half_size), 5)
                pygame.draw.line(accent_surface, color_with_alpha, (center_x - half_size, center_y), (center_x + half_size, center_y), 5)
                pygame.draw.line(accent_surface, color_with_alpha, (center_x - half_size, center_y - half_size), (center_x + half_size, center_y + half_size), 5)
                pygame.draw.line(accent_surface, color_with_alpha, (center_x - half_size, center_y + half_size), (center_x + half_size, center_y - half_size), 5)

        self.screen.blit(accent_surface, (0, 0))

    def _draw_hud(self):
        state = self.state
        info_top = self.FACE_HEIGHT

        style = self._get_face_style()
        status_key = state.target_status if state.is_animating or state.is_sleep_animating else state.status
        
        pygame.draw.rect(self.screen, self.BLACK, (0, info_top, self.DISPLAY_WIDTH, self.INFO_HEIGHT))
        
        status_label = self.STATUS_DISPLAY_LABELS.get(status_key, status_key)
        status_color = style['status_color']
        cpu_val = f"{self.state.cpu_load:.1f} %"
        cpu_color = self.LCARS_BLUE if self.state.cpu_load < 75 else self.LCARS_ORANGE
        mem_val = f"{self.state.mem_load:.1f} %"
        mem_color = self.LCARS_MAGENTA if self.state.mem_load < 75 else self.LCARS_ORANGE
        disk_val = f"{self.state.disk_load:.1f} %"
        disk_color = self.LCARS_ORANGE if self.state.disk_load < 75 else self.LCARS_RED

        x = self.x_col1
        y = self.HUD_ROW_Y
        w = self.HUD_ROW_WIDTH
        h = self.BLOCK_HEIGHT 

        self._draw_lcars_block(self.screen, x, y, w, h, "STATUS", status_label, status_color, self.font_small)

        x += w + self.HUD_ROW_SPACING
        self._draw_lcars_block(self.screen, x, y, w, h, "CPU", cpu_val, cpu_color, self.font_small)

        x += w + self.HUD_ROW_SPACING
        self._draw_lcars_block(self.screen, x, y, w, h, "MEMORY", mem_val, mem_color, self.font_small)

        x += w + self.HUD_ROW_SPACING
        self._draw_lcars_block(self.screen, x, y, w, h, "DISK", disk_val, disk_color, self.font_small)

        x_col2 = self.DISPLAY_WIDTH // 2 - self.BLOCK_WIDTH // 2
        x_col3 = self.DISPLAY_WIDTH - self.BLOCK_WIDTH - 20

        # --- COLUMN 1: LIVE CAMERA FEED ---
        pygame.draw.rect(
            self.screen, 
            self.LCARS_BLUE, 
            (self.CAMERA_BOX_X, self.CAMERA_BOX_Y, self.CAMERA_BLOCK_WIDTH, self.CAMERA_BLOCK_HEIGHT),
            border_radius=self.BLOCK_HEIGHT // 2
        )
        
        BORDER_THICKNESS = 4
        INNER_X = self.CAMERA_BOX_X + BORDER_THICKNESS
        INNER_Y = self.CAMERA_BOX_Y + BORDER_THICKNESS
        INNER_W = self.CAMERA_BLOCK_WIDTH - 2 * BORDER_THICKNESS
        INNER_H = self.CAMERA_BLOCK_HEIGHT - 2 * BORDER_THICKNESS
        INNER_RADIUS = (self.BLOCK_HEIGHT // 2) - BORDER_THICKNESS
        if INNER_RADIUS < 0: INNER_RADIUS = 0
        
        pygame.draw.rect(
            self.screen, 
            self.BLACK, 
            (INNER_X, INNER_Y, INNER_W, INNER_H),
            border_radius=INNER_RADIUS
        )
        
        if self.camera_surface:
            clip_surface = pygame.Surface((INNER_W, INNER_H), pygame.SRCALPHA)
            pygame.draw.rect(clip_surface, (255, 255, 255, 255), (0, 0, INNER_W, INNER_H), border_radius=INNER_RADIUS)
            
            scaled_camera_surface = pygame.transform.scale(self.camera_surface, (INNER_W, INNER_H))
            try:
                scaled_camera_surface.blit(clip_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
            except pygame.error:
                pass 
            
            self.screen.blit(scaled_camera_surface, (INNER_X, INNER_Y))
        else:
            # --- MODIFIED: More helpful fallback text ---
            msg = "WAITING FOR NPY..."
            if not os.path.exists(FRAME_NPY_PATH) and (time.time() - self.last_json_read_time > 2.0):
                 msg = "VISION OFFLINE"
            
            fallback_text = self.font_small.render(msg, True, self.LCARS_RED)
            text_x = INNER_X + (INNER_W - fallback_text.get_width()) // 2
            text_y = INNER_Y + (INNER_H - fallback_text.get_height()) // 2
            self.screen.blit(fallback_text, (text_x, text_y))

        # --- ROBOT STATUS (Column 2) ---
        pico_state = state.pico_state
        pico_connected = pico_state.get('serial_connected', False)
        pico_val = "ONLINE" if pico_connected else "OFFLINE"
        pico_color = self.LCARS_BLUE if pico_connected else self.LCARS_RED
        self._draw_lcars_block(self.screen, x_col2, self.y_start_hud, self.BLOCK_WIDTH, self.BLOCK_HEIGHT, "PICO STATUS", pico_val, pico_color, self.font_small)

        controller_connected = state.controller_connected
        if controller_connected:
            controller_name = state.controller_name
            controller_val = (controller_name[:16] + "...") if len(controller_name) > 18 else controller_name
            controller_color = self.LCARS_ORANGE
        else:
            controller_val = "OFFLINE"
            controller_color = self.LCARS_RED
        self._draw_lcars_block(self.screen, x_col2, self.y_start_hud + self.BLOCK_HEIGHT + self.BLOCK_SPACING, self.BLOCK_WIDTH, self.BLOCK_HEIGHT, "REMOTE", controller_val, controller_color, self.font_small)

        temp_f = state.temp_c * (9/5) + 32
        humidity = state.humidity
        pressure_inhg = state.pressure * 0.02953
        env_val = f"{temp_f:.1f}°F    {humidity:.0f}%    {pressure_inhg:.2f}inHg"
        block_y = self.y_start_hud + 2 * (self.BLOCK_HEIGHT + self.BLOCK_SPACING)
        
        try: 
            pygame.draw.rect(
                self.screen, 
                self.LCARS_MAGENTA, 
                (x_col2, block_y, self.BLOCK_WIDTH, self.BLOCK_HEIGHT), 
                border_radius=self.BLOCK_HEIGHT // 2
            )
        except Exception: 
            pygame.draw.rect(
                self.screen, 
                self.LCARS_MAGENTA, 
                (x_col2, block_y, self.BLOCK_WIDTH, self.BLOCK_HEIGHT)
            )
        
        env_text_surface = self.font_small.render(env_val, True, self.LCARS_WHITE)
        text_rect = env_text_surface.get_rect(center=(x_col2 + self.BLOCK_WIDTH // 2, block_y + self.BLOCK_HEIGHT // 2))
        self.screen.blit(env_text_surface, text_rect)

        # --- POWER/BATTERY (Column 3) ---
        batt_pct = state.battery_percent
        is_charging = state.is_charging
        if is_charging:
            top_block_color, label_text, meter_block_color = self.LCARS_YELLOW, "POWER (CHARGING)", self.LCARS_BLUE
        elif batt_pct < 15:
            top_block_color, label_text, meter_block_color = self.LCARS_RED, "POWER (LOW)", self.LCARS_RED
        else:
            top_block_color, label_text, meter_block_color = self.LCARS_BLUE, "POWER", self.LCARS_BLUE
            
        self._draw_lcars_block(self.screen, x_col3, self.y_start_hud, self.BLOCK_WIDTH, self.BLOCK_HEIGHT, label_text, f"{batt_pct:.0f} %", top_block_color, self.font_small)
        self._draw_lcars_block(self.screen, x_col3, self.y_start_hud + self.BLOCK_HEIGHT + self.BLOCK_SPACING, self.BLOCK_WIDTH, self.BLOCK_HEIGHT, "", "", meter_block_color, self.font_small, block_type='METER', value_data=(batt_pct, is_charging))

        led_state_num = pico_state.get('led_state', 0)
        if led_state_num == 0: led_status_str = "OFF"
        elif led_state_num == 1: led_status_str = "ON (LOW)"
        elif led_state_num == 2: led_status_str = "ON (HIGH)"
        else: led_status_str = "UNKNOWN"
        led_color = self.LCARS_YELLOW if led_state_num > 0 else self.LCARS_BLUE
        self._draw_lcars_block(self.screen, x_col3, self.y_start_hud + 2*(self.BLOCK_HEIGHT + self.BLOCK_SPACING), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, "LED STATUS", led_status_str, led_color, self.font_small)

    def _update_animation(self, current_time):
        if self.state.is_sleep_animating:
            sleep_time_elapsed = current_time - self.state.sleep_animation_start_time
            if sleep_time_elapsed >= self.sleep_animation_duration:
                self.state.is_sleep_animating = False
                self.state.sleep_animation_dir = 0
                self.state.status = self.state.target_status
                self.state.target_status = self.state.status
                self.state.is_animating = False
        
        elif self.state.is_animating:
            time_elapsed = current_time - self.state.animation_start_time
            if time_elapsed >= self.default_animation_duration:
                self.state.is_animating = False
                self.state.status = self.state.target_status

        if not self.state.is_sleep_animating and not self.state.is_animating and self.state.status != self.state.target_status:
            self.state.status = self.state.target_status

        delta_time = self.clock.get_time() / 1000.0 
        if delta_time > 0.1: delta_time = 0.1 
        
        current_alpha = self.state.blush_alpha
        target_alpha = self.state.blush_target_alpha
        
        if abs(current_alpha - target_alpha) > 0.01:
            change = self.state.blush_animation_speed * delta_time
            if current_alpha < target_alpha:
                self.state.blush_alpha = min(current_alpha + change, target_alpha)
            else:
                self.state.blush_alpha = max(current_alpha - change, target_alpha)
        elif target_alpha == 0.0:
            self.state.blush_alpha = 0.0

        active_accents = []
        for accent in self.state.accent_list:
            time_elapsed = current_time - accent['start_time']
            if time_elapsed < self.ACCENT_DURATION:
                if time_elapsed < self.ACCENT_FADE_IN_TIME:
                    accent['alpha'] = (time_elapsed / self.ACCENT_FADE_IN_TIME) * 255
                elif time_elapsed > (self.ACCENT_DURATION - self.ACCENT_FADE_OUT_TIME):
                    time_left = self.ACCENT_DURATION - time_elapsed
                    accent['alpha'] = (time_left / self.ACCENT_FADE_OUT_TIME) * 255
                else:
                    accent['alpha'] = 255
                
                accent['alpha'] = max(0, min(255, accent['alpha']))
                active_accents.append(accent)
        
        self.state.accent_list = active_accents

    # --- Main Loop Helper Methods (UNCHANGED) ---
    def _process_pygame_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                self.running = False
            elif event.type == pygame.KEYDOWN:
                ctrl_pressed = event.mod & pygame.KMOD_CTRL

                if event.key == pygame.K_ESCAPE: 
                    self.running = False
                
                elif event.key == pygame.K_d and ctrl_pressed:
                    print("Minimize command (Ctrl+D) received. Iconifying window.")
                    pygame.display.iconify()

    def _process_socket_events(self):
        if not self.display_server_socket:
            return
            
        display_events = self.display_sel.select(timeout=0)
        for key, mask in display_events:
            callback = key.data
            callback(key.fileobj)

    def _update_camera_frame(self):
        """Check the camera queue for a new frame."""
        try: 
            self.camera_surface = self.camera_frame_queue.get_nowait()
        except queue.Empty: 
            pass

    def _process_command_queue(self, current_time):
        try:
            command_item = self.command_queue.get_nowait()

            if isinstance(command_item, tuple) and command_item[0] == 'EYE':
                _, x_offset, y_offset = command_item
                self.state.eye_target = (x_offset, y_offset)
                self.last_movement_update = current_time

            elif isinstance(command_item, tuple) and command_item[0] == 'MOUTH':
                _, mouth_shape, thickness, curvature = command_item
                self.state.speech_mouth_override = mouth_shape
                self.state.mouth_thickness_override = thickness
                self.state.mouth_curvature_override = curvature
                self.state.last_mouth_command_time = current_time

            elif isinstance(command_item, tuple) and command_item[0] == 'WAVE':
                _, amp, freq = command_item
                self.state.wave_amplitude = amp
                self.state.wave_frequency = freq
                self.state.last_mouth_command_time = current_time

            elif isinstance(command_item, tuple) and command_item[0] == 'BROW':
                _, is_on, thick, width, height, angle, curve = command_item
                self.state.brows_enabled = is_on
                self.state.brow_thickness = thick
                self.state.brow_width = width
                self.state.brow_height_offset = height
                self.state.brow_angle = angle
                self.state.brow_curvature = curve

            elif isinstance(command_item, tuple) and command_item[0] == 'BLUSH':
                _, is_on = command_item
                self.state.blush_enabled = is_on
                self.state.blush_target_alpha = self.BLUSH_MAX_ALPHA if is_on else 0.0

            elif isinstance(command_item, tuple) and command_item[0] == 'ACCENT':
                _, accent_type = command_item
                self.state.accent_list.append({
                    'type': accent_type, 'start_time': current_time, 'side': 'LEFT'
                })
                self.state.accent_list.append({
                    'type': accent_type, 'start_time': current_time, 'side': 'RIGHT'
                })

            elif isinstance(command_item, str):
                command = command_item
                if command == 'QUIT':
                    self.running = False
                elif command in self.STATUS_CONFIG:
                    if command != self.state.status and command != self.state.target_status:
                        if command == 'SLEEP' or self.state.status == 'SLEEP':
                            self.state.is_sleep_animating = True
                            self.state.sleep_animation_start_time = current_time
                            self.state.sleep_animation_dir = 1 if command == 'SLEEP' else 0
                        self.state.is_animating = True
                        self.state.animation_start_time = current_time
                        self.state.target_status = command

                        if command == 'ANGRY':
                            self.state.brows_enabled = True
                            self.state.brow_thickness = BROW_DEFAULT_THICKNESS
                            self.state.brow_width = BROW_DEFAULT_WIDTH
                            self.state.brow_height_offset = BROW_DEFAULT_HEIGHT_OFFSET
                            self.state.brow_angle = BROW_DEFAULT_ANGLE
                            self.state.brow_curvature = BROW_DEFAULT_CURVATURE
                        elif self.state.status == 'ANGRY':
                            self.state.brows_enabled = False 
                            self.state.brow_thickness = BROW_DEFAULT_THICKNESS
                            self.state.brow_width = BROW_DEFAULT_WIDTH
                            self.state.brow_height_offset = BROW_DEFAULT_HEIGHT_OFFSET
                            self.state.brow_angle = BROW_DEFAULT_ANGLE
                            self.state.brow_curvature = BROW_DEFAULT_CURVATURE

        except queue.Empty:
            pass

    def _update_random_eye_movement(self, current_time):
        if self.state.status != 'SLEEP':
            if current_time - self.last_movement_update > random.uniform(2, 5):
                target_x = random.randint(-self.RANDOM_EYE_MAX_OFFSET, self.RANDOM_EYE_MAX_OFFSET)
                target_y = random.randint(-self.RANDOM_EYE_MAX_OFFSET, self.RANDOM_EYE_MAX_OFFSET)
                self.state.eye_target = (target_x, target_y)
                self.last_movement_update = current_time

    def _handle_input(self):
        current_time = time.time()
        
        self._process_pygame_events()
        self._process_socket_events()
        self._update_state_from_json(current_time)
        
        if current_time - self.last_stats_read_time > self.stats_read_interval:
            self._update_system_stats()
            self.last_stats_read_time = current_time
            
        self._update_camera_frame()
        self._process_command_queue(current_time)
        self._update_random_eye_movement(current_time)

    def main_loop(self):
        """Main application loop."""
        while self.running:
            current_time = time.time()
            self._handle_input()
            self._update_animation(current_time)

            self.screen.fill(self.LCARS_BG_DARK)
            self._draw_face(current_time)
            self._draw_hud()

            pygame.display.flip()
            self.clock.tick(60)

    # --- MODIFIED: Cleanup ---
    def cleanup(self):
        """Stops threads and quits Pygame cleanly."""
        print("Cleaning up...")
        
        # Stop the new frame reader thread
        if self.frame_reader_thread:
            self.frame_reader_thread.stop()

        if self.display_server_socket:
            try:
                self.display_sel.unregister(self.display_server_socket)
                self.display_server_socket.close()
                print("Display server socket closed.")
            except Exception as e:
                print(f"Error closing display server socket: {e}")
        try:
            self.display_sel.close()
            print("Display selector closed.")
        except Exception as e:
            print(f"Error closing display selector: {e}")

        pygame.quit()
        print("Exiting.")
        sys.exit()
    # --- END MODIFICATION ---

# --- Application Entry Point ---
if __name__ == "__main__":
    app = None
    try:
        app = LCARSDisplayApp()
        app.main_loop()
    except Exception as e:
        print(f"An error occurred in main execution: {e}", file=sys.stderr)
    finally:
        if app:
            app.cleanup()