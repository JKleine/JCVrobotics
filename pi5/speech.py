import pyttsx3
import threading
import time
import socket
import random
import atexit
import queue
import selectors

# --- 1. Configuration Constants ---

# Socket address for the display.py script (for MOUTH/WAVE commands)
DISPLAY_SOCKET_ADDRESS = ('127.0.0.1', 65002) 

# Socket address for the *this* module to listen for text input
MODULE_SERVER_HOST = '127.0.0.1'
MODULE_SERVER_PORT = 65003 # New dedicated port for the Speech Module

# Animation parameters
WAVE_UPDATE_INTERVAL_MS = 50 
MIN_AMPLITUDE = 5
MAX_AMPLITUDE = 35
MIN_FREQUENCY = 3
MAX_FREQUENCY = 9


# --- Global State for Animation Thread and Engine ---
animation_running = threading.Event()
audio_engine = None 
text_queue = queue.Queue() # Queue for text received by the socket server
sel = selectors.DefaultSelector() # Selector for managing socket connections

# --- pyttsx3 Callbacks ---
def onStart(name):
    """Signal animation thread to start running when speaking begins."""
    print("pyttsx3 Callback: Started speaking utterance")
    animation_running.set() 

def onEnd(name, completed):
    """Signal animation thread to stop when speaking ends."""
    print(f"pyttsx3 Callback: Finished utterance (Completed: {completed})")
    animation_running.clear() 

# --- Core Socket Functions ---

def send_display_command(command_str: str):
    """Sends a command string to the display module via TCP socket."""
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(DISPLAY_SOCKET_ADDRESS)
        s.sendall(command_str.encode('utf-8'))
    except socket.timeout: pass
    except ConnectionRefusedError:
        # Silently fail if the display isn't running, but log the refusal.
        if command_str.strip() not in ["WAVE 0 5", "MOUTH FLAT 0 0"]: 
            print(f"Connection refused sending: {command_str.strip()} (Is display.py running?)")
    except Exception as e:
        print(f"Socket error sending '{command_str.strip()}': {e}")
    finally:
        if s: s.close()

# --- TTS and Synchronization Logic ---

def animation_loop():
    """
    Dedicated thread for generating and sending WAVE animation commands.
    Runs when audio is playing, as signaled by the pyttsx3 callbacks.
    """
    print("Animation thread started, waiting for onStart signal...")
    sleep_duration_sec = WAVE_UPDATE_INTERVAL_MS / 1000.0
    last_amp = 0
    last_freq = 5

    while True:
        animation_running.wait() 
        
        # --- Animation Phase ---
        # print("Animation thread: Event set, starting WAVE commands.")
        while animation_running.is_set():
            if not threading.main_thread().is_alive(): return 

            # Generate semi-random wave parameters
            target_amp = random.randint(MIN_AMPLITUDE, MAX_AMPLITUDE)
            target_freq = random.randint(MIN_FREQUENCY, MAX_FREQUENCY)

            # Smooth the transition
            current_amp = int(last_amp * 0.7 + target_amp * 0.3)
            current_freq = int(last_freq * 0.7 + target_freq * 0.3)
            current_freq = max(1, current_freq) 

            command = f"WAVE {current_amp} {current_freq}\n"
            send_display_command(command)

            last_amp = current_amp
            last_freq = current_freq

            time.sleep(sleep_duration_sec) 
        
        # --- Stop Phase ---
        # print("Animation thread: Event cleared, stopping WAVE commands and resetting mouth.")
        # Send zero amplitude command
        send_display_command(f"WAVE 0 {last_freq}\n") 
        time.sleep(sleep_duration_sec) 
        # Reset mouth mode to FLAT (This is what overrides the speech_mouth_override)
        send_display_command("MOUTH FLAT 0 0\n") 
        
        last_amp = 0 
        # Loop restarts, goes back to animation_running.wait()

def initialize_tts():
    """Initializes pyttsx3 engine and connects callbacks."""
    global audio_engine
    try:
        print("Initializing pyttsx3 engine...")
        audio_engine = pyttsx3.init()
        if not audio_engine: return False

        # --- SET VOICE AND RATE ---
        try:
            american_voice_id = 'mb-us1' 
            audio_engine.setProperty('voice', american_voice_id)
            audio_engine.setProperty('rate', 60) 
            print("Engine properties set (Voice: mb-us1, Rate: 60).")
        except Exception as e_prop:
            print(f"Warning: Could not set voice properties: {e_prop}")

        # --- CONNECT CALLBACKS ---
        audio_engine.connect('started-utterance', onStart)
        audio_engine.connect('finished-utterance', onEnd)
        print("Callbacks connected.")
        
        return True

    except Exception as e:
        print(f"FATAL: Error during TTS initialization: {e}")
        audio_engine = None
        return False

def speak_and_sync(text: str):
    """
    The main public function: queues MOUTH WAVE, speaks the text (blocking), 
    and lets the callbacks manage the animation loop.
    """
    if not audio_engine:
        print("pyttsx3 engine not initialized.")
        return

    print(f"\n--- Processing: '{text}' ---")

    # 1. Tell display.py to enter WAVE mode *before* starting audio
    send_display_command("MOUTH WAVE 0 0\n")
    time.sleep(0.02) 

    # 2. Queue the speech
    audio_engine.say("a " + text)

    # 3. Play the speech and wait for it to complete (BLOCKING)
    # onStart/onEnd will manage the 'animation_loop' thread
    audio_engine.runAndWait() 
    
    # 4. Give animation_loop a moment to receive onEnd signal and reset mouth
    time.sleep(WAVE_UPDATE_INTERVAL_MS / 1000.0 * 2) 

    print(f"--- Finished speaking. ---")

# --- Socket Server for Input ---

def setup_module_server_socket(host, port, input_queue):
    """Creates and sets up the non-blocking server socket for receiving text."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen()
    server_sock.setblocking(False) 
    sel.register(server_sock, selectors.EVENT_READ, data=lambda conn: accept_text_connection(conn, sel))
    print(f"Speech module listening for text input on {host}:{port}")
    return server_sock

def accept_text_connection(server_sock, sel_obj):
    """Accepts a new client connection."""
    try:
        conn, addr = server_sock.accept()
        print(f"Accepted text input connection from {addr}")
        conn.setblocking(False)
        sel_obj.register(conn, selectors.EVENT_READ, data=lambda conn: handle_text_data(conn, sel_obj))
    except Exception as e:
        print(f"Error accepting text connection: {e}")

def handle_text_data(conn, sel_obj):
    """Handles receiving data from a client socket and puts it in the queue."""
    try:
        recv_data = conn.recv(1024)
        if recv_data:
            text = recv_data.decode('utf-8').strip()
            if text:
                print(f"Socket Input: Received '{text}'")
                text_queue.put(text)
        else:
            sel_obj.unregister(conn)
            conn.close()
    except ConnectionResetError:
        sel_obj.unregister(conn)
        conn.close()
    except Exception as e:
        print(f"Error handling socket text data: {e}")
        try: sel_obj.unregister(conn); conn.close()
        except: pass

def server_and_queue_manager():
    """Manages the socket server and processes the text queue."""
    while True:
        # 1. Handle incoming socket requests
        events = sel.select(timeout=0.1) # Check for events briefly
        for key, mask in events:
            callback = key.data
            callback(key.fileobj) 
            
        # 2. Process the text queue
        try:
            text_to_speak = text_queue.get_nowait()
            speak_and_sync(text_to_speak)
        except queue.Empty:
            pass # No text to speak

        # Wait briefly before re-checking sockets and queue
        time.sleep(0.01)

# --- Cleanup Function ---
@atexit.register
def cleanup_speech():
    """Ensures mouth is reset and sockets are closed on script exit."""
    print("\nCleaning up speech module (atexit)...")
    send_display_command("MOUTH FLAT 0 0\n") 
    try:
        for fd, key in sel.get_map().items():
            sel.unregister(key.fileobj)
            key.fileobj.close()
        sel.close()
        print("Module server sockets closed.")
    except Exception as e:
        print(f"Error during server cleanup: {e}")

# --- Main Execution Block ---
if __name__ == "__main__":
    if not initialize_tts():
        print("Failed to initialize TTS. Exiting.")
        exit()

    # 1. Start the single animation thread (daemon)
    anim_thread_instance = threading.Thread(target=animation_loop, daemon=True)
    anim_thread_instance.start()

    # 2. Setup the server socket for text input
    server_socket = setup_module_server_socket(MODULE_SERVER_HOST, MODULE_SERVER_PORT, text_queue)

    # 3. Start the main listening/processing thread
    manager_thread = threading.Thread(target=server_and_queue_manager, daemon=True)
    manager_thread.start()

    print("\n--- Speech Module Active ---")
    print(f"Waiting for text input on {MODULE_SERVER_HOST}:{MODULE_SERVER_PORT}")
    print("Example command (from another terminal): echo 'Hello robot' | nc 127.0.0.1 65003")
    print("Press Ctrl+C to exit.")

    # Keep the main thread alive so daemon threads continue to run
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nModule interrupted by user.")
    finally:
        print("Exiting main thread. Cleanup handled by atexit.")