import socket
import time
import math
import sys

# --- Configuration ---
# These must match the settings in your display.py
HOST = '127.0.0.1'
PORT = 65002

def send_cmd(command_str):
    """
    Connects to the display server, sends a single command,
    and returns True on success or False on failure.
    """
    try:
        # Create a new socket for each command
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(command_str.encode('utf-8'))
        return True
    except ConnectionRefusedError:
        print(f"Error: Connection refused. Is display.py running?", file=sys.stderr)
        return False
    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
        return False

def animate_speech(frames, frame_duration_ms):
    """
    Helper function to iterate through a list of animation frames.
    
    :param frames: A list of tuples, where each tuple is
                   (SHAPE, THICKNESS, CURVATURE)
    :param frame_duration_ms: How long (in ms) to hold each frame.
    """
    sleep_sec = frame_duration_ms / 1000.0
    for shape, thickness, curvature in frames:
        cmd_str = f"MOUTH {shape} {thickness} {curvature:.2f}"
        if not send_cmd(cmd_str):
            # Stop animation if connection fails
            return False
        time.sleep(sleep_sec)
    return True

def run_demo():
    """Runs through a sequence of animation showcases."""
    
    print("Starting face animation demo in 3 seconds...")
    print("Make sure display.py is running on the same machine.")
    print("Press Ctrl+C to stop this demo script at any time.")
    time.sleep(3)

    # --- 1. STATUS Demo ---
    print("\n--- 1. STATUS Demo (Built-in Transitions) ---")
    if not send_cmd("HAPPY"): return
    time.sleep(2)
    if not send_cmd("SAD"): return
    time.sleep(2)
    if not send_cmd("ERROR"): return
    time.sleep(2)
    if not send_cmd("IDLE"): return
    time.sleep(2)

    # --- 2. EYE Animation Demo ---
    print("\n--- 2. EYE Animation Demo (Smooth Look) ---")
    print("...Looking around (Lissajous curve)...")
    start_time = time.time()
    while time.time() - start_time < 5:
        t = time.time() - start_time
        x_pos = int(math.sin(t * 3.0) * 30) 
        y_pos = int(math.cos(t * 2.0) * 20)
        if not send_cmd(f"EYE {x_pos} {y_pos}"): return
        time.sleep(0.016) # ~60fps
        
    if not send_cmd("EYE 0 0"): return
    time.sleep(1)

    # --- 3. MOUTH Animation (Basic Speech) ---
    print("\n--- 3. MOUTH Animation Demo (Basic Speech) ---")
    speech_sequence = [
        ("OPEN", 120), ("WIDE_E", 150), ("SMALL_O", 200), ("FLAT", 100),
        ("OPEN", 180), ("WIDE_E", 150), ("CLOSED", 100)
    ]
    for shape, duration_ms in speech_sequence:
        if not send_cmd(f"MOUTH {shape}"): return
        time.sleep(duration_ms / 1000.0)
    time.sleep(2)

    # --- 4. Advanced MOUTH Demo (High-Frame-Rate Speech) ---
    # *** THIS SECTION IS MODIFIED ***
    print("\n--- 4. Advanced MOUTH Demo (High-Frame-Rate Speech) ---")
    print("... Simulating 'Hello, how are you?' with 44 frames...")

    # A 44-frame animation for "Hello, how are you?"
    # (Shape, Thickness, Curvature)
    full_speech_frames = [
        # --- "Hello" ---
        ("CLOSED",  2, 0.0),  # (Start)
        ("FLAT",    4, 0.0),  # "H..."
        ("FLAT",    6, 0.0),
        ("WIDE_E",  8, 0.0),  # "...e..."
        ("WIDE_E", 10, 0.0),
        ("WIDE_E",  8, 0.0),
        ("SMILE",   7, 0.1),  # "...l..." (transition)
        ("SMILE",   8, 0.3),
        ("SMILE",   9, 0.5),  # (Peak 'L')
        ("SMILE",   8, 0.3),
        ("SMALL_O",10, 0.0),  # "...o..." (transition)
        ("SMALL_O",14, 0.0),  # (Peak 'O')
        ("SMALL_O",10, 0.0),
        ("SMALL_O", 6, 0.0),
        ("CLOSED",  4, 0.0),  # (End 'o')
        
        # --- Pause (comma) ---
        ("CLOSED", 2, 0.0),
        ("CLOSED", 2, 0.0),
        ("CLOSED", 2, 0.0),
        
        # --- "How" ---
        ("FLAT",    4, 0.0),  # "H..."
        ("OPEN",    8, 0.0),  # "...o..."
        ("OPEN",   12, 0.0),
        ("OPEN",   14, 0.0),  # (Peak 'ow')
        ("SMALL_O",12, 0.0),  # (transition to 'w')
        ("SMALL_O", 8, 0.0),
        ("CLOSED",  4, 0.0),  # (End 'w')
        
        # --- "Are" ---
        ("OPEN",    6, 0.0),  # "A..."
        ("OPEN",   10, 0.0),
        ("SMILE",   8, 0.2),  # "...r..." (using SMILE for 'r')
        ("SMILE",   6, 0.1),
        ("CLOSED",  4, 0.0),  # (End 'r')
        
        # --- Pause ---
        ("CLOSED", 2, 0.0),
        ("CLOSED", 2, 0.0),

        # --- "You" ---
        ("SMILE",   6, 0.1),  # "Y..."
        ("SMILE",   8, 0.2),
        ("SMALL_O",10, 0.0),  # "...ou..."
        ("SMALL_O",12, 0.0),
        ("SMALL_O",10, 0.0),
        ("SMALL_O", 8, 0.0),
        ("CLOSED",  4, 0.0),  # (End 'ou')
        
        # --- Question lilt (?) ---
        ("FLAT", 4, 0.0),
        ("SMILE", 5, 0.1),
        ("SMILE", 6, 0.2), # Hold slight smile
        ("SMILE", 6, 0.2),
    ]

    # Run the animation. Each frame is held for 35ms.
    # 44 frames * 35ms = 1540ms (approx 1.5 seconds)
    if not animate_speech(full_speech_frames, 35): return
    
    time.sleep(2)


    # --- 5. Sleep Demo ---
    print("\n--- 5. Sleep Demo ---")
    if not send_cmd("SLEEP"): return
    print("...Sleeping for 3 seconds...")
    time.sleep(3)
    
    print("...Waking up...")
    if not send_cmd("IDLE"): return
    time.sleep(2)

    print("\n--- Demo Finished ---")


# --- Main execution ---
if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        print("\nDemo stopped by user (Ctrl+C).")
    finally:
        # Ensure the face is reset to a neutral state on exit
        print("Resetting face to IDLE and eyes to center.")
        send_cmd("IDLE")
        send_cmd("EYE 0 0")