#!/usr/bin/python3
# -*- coding:utf-8 -*-

"""
webserver.py - J1 Robot Teleoperation Server

This single script runs three services in parallel threads:
1.  HTTP File Server (Port 8080): Serves the 'index.html', 'classic.css', etc.
2.  WebSocket Command Server (Port 65005): Receives commands from the browser 
    and relays them to robot_operator.py (on port 65004).
3.  MJPEG Video Bridge (UDP Port 8000 -> HTTP Port 8001): Catches the UDP 
    stream from vision.py and re-serves it as a standard MJPEG HTTP stream 
    for the browser.
"""

import http.server
import socketserver
import threading
import socket
import asyncio
import websockets # You may need to install this: pip install websockets
import sys
import os
import time

# --- Configuration ---
FILE_SERVER_PORT = 8080
FILE_SERVER_HOST = '0.0.0.0' # Serve on all interfaces (for phone/PC access)

WS_COMMAND_PORT = 65005
WS_COMMAND_HOST = '0.0.0.0' # Listen on all interfaces

VIDEO_BRIDGE_HTTP_PORT = 8001
VIDEO_BRIDGE_HTTP_HOST = '0.0.0.0' # Serve video on all interfaces

VIDEO_BRIDGE_UDP_PORT = 8000
VIDEO_BRIDGE_UDP_HOST = '127.0.0.1' # Listen for UDP from vision.py

ROBOT_OPERATOR_PORT = 65004
ROBOT_OPERATOR_HOST = '127.0.0.1'

# --- 1. Command Relay Helper (Sends to robot_operator.py) ---

class CommandRelayHelper:
    """Helper to send commands to the RobotOperator."""
    def __init__(self, port, host):
        self.address = (host, port)
        self.lock = threading.Lock()

    def send_command(self, command_str: str):
        if not command_str:
            return False
        
        command_str = command_str.strip() + '\n'
        
        with self.lock:
            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(self.address)
                s.sendall(command_str.encode('utf-8'))
                return True
            except ConnectionRefusedError:
                print(f"[CommandRelay] Error: Connection refused by RobotOperator on {self.address}")
            except socket.timeout:
                print(f"[CommandRelay] Error: Timeout sending to RobotOperator")
            except Exception as e:
                print(f"[CommandRelay] Error: {e}")
            finally:
                if s:
                    s.close()
            return False

# Initialize the one global relay helper
robot_relay = CommandRelayHelper(ROBOT_OPERATOR_PORT, ROBOT_OPERATOR_HOST)

# --- 2. WebSocket Command Server (Thread 1) ---

async def websocket_command_handler(websocket, path=None):
    """
    Handles a single browser WebSocket connection.
    Receives commands and passes them to the robot_relay.
    """
    print(f"[WebSocket] Client connected from {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"[WebSocket] Received command: '{message}'")
            if robot_relay.send_command(message):
                # Optional: Send a confirmation back to the browser
                # await websocket.send(f"ACK: {message}")
                pass
            else:
                print(f"[WebSocket] Failed to relay command: '{message}'")
                # Optional: Send an error back to the browser
                # await websocket.send(f"NACK: {message}")

    except websockets.exceptions.ConnectionClosed:
        print(f"[WebSocket] Client disconnected from {websocket.remote_address}")
    except Exception as e:
        print(f"[WebSocket] Error: {e}")

def start_websocket_server():
    """Starts the WebSocket command server in its own thread."""
    print(f"[WebSocket] Starting command server on {WS_COMMAND_HOST}:{WS_COMMAND_PORT}")
    
    # This setup is required to run websockets server in a thread
    async def run_server():
        async with websockets.serve(websocket_command_handler, WS_COMMAND_HOST, WS_COMMAND_PORT):
            await asyncio.Future() # Run forever

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(run_server())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        print("[WebSocket] Server stopped.")

# --- 3. HTTP File Server (Thread 2) ---

def start_file_server():
    """Starts the HTTP server to serve index.html and other assets."""
    # We must change directory to where the HTML/CSS files are
    # Assuming they are in the same directory as this script
    script_dir = os.path.dirname(os.path.realpath(__file__))
    os.chdir(script_dir)
    
    Handler = http.server.SimpleHTTPRequestHandler
    
    try:
        with socketserver.ThreadingTCPServer((FILE_SERVER_HOST, FILE_SERVER_PORT), Handler) as httpd:
            print(f"[FileServer] Serving files from '{script_dir}' on http://{FILE_SERVER_HOST}:{FILE_SERVER_PORT}")
            httpd.serve_forever()
    except Exception as e:
        print(f"[FileServer] FATAL ERROR: {e}")
    finally:
        print("[FileServer] Server stopped.")

# --- 4. MJPEG Video Bridge (Thread 3) ---

# Global buffer to hold the latest JPEG frame from the UDP stream
latest_frame_lock = threading.Lock()
latest_frame_data = None

def udp_listener_thread():
    """Catches UDP packets from vision.py and stores the latest frame."""
    global latest_frame_data
    
    print(f"[VideoBridge] Listening for UDP stream from vision.py on {VIDEO_BRIDGE_UDP_HOST}:{VIDEO_BRIDGE_UDP_PORT}")
    
    # We must bind to the host that GStreamer is *sending* to
    # This must match the 'multiudpsink clients=' host in vision.py
    # If vision.py sends to 127.0.0.1, we listen on 127.0.0.1
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Set a receive buffer size
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024) # 1MB buffer
    
    try:
        udp_socket.bind((VIDEO_BRIDGE_UDP_HOST, VIDEO_BRIDGE_UDP_PORT))
    except Exception as e:
        print(f"[VideoBridge] FATAL: Could not bind to UDP port {VIDEO_BRIDGE_UDP_PORT}. {e}")
        print("    Is another process (or old vision.py) running?")
        return

    while True:
        try:
            # GStreamer's jpegenc sends one full JPEG per UDP packet
            # But packets can be fragmented. We'll assume for now they fit
            # in a large buffer (65535 bytes is max UDP packet size)
            data, _ = udp_socket.recvfrom(65536) 
            
            # Check for valid JPEG headers (SOI and EOI)
            if data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'):
                with latest_frame_lock:
                    latest_frame_data = data
            else:
                # This can happen if packets are dropped or out of order
                print("[VideoBridge] Warning: Received partial or invalid JPEG packet.")
                pass 
                
        except Exception as e:
            print(f"[VideoBridge] UDP listener error: {e}")
            time.sleep(1)

class MJPEGStreamHandler(http.server.BaseHTTPRequestHandler):
    """
    Handles HTTP requests for the /stream.mjpg endpoint.
    """
    def do_GET(self):
        global latest_frame_data
        
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            
            last_sent_frame = None
            
            try:
                while True:
                    time.sleep(1/60) # Cap at 60fps
                    
                    frame_to_send = None
                    with latest_frame_lock:
                        if latest_frame_data and latest_frame_data != last_sent_frame:
                            frame_to_send = latest_frame_data
                            last_sent_frame = latest_frame_data

                    if frame_to_send:
                        self.wfile.write(b'--jpgboundary\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(frame_to_send)))
                        self.end_headers()
                        self.wfile.write(frame_to_send)
                        self.wfile.write(b'\r\n')
            
            except (BrokenPipeError, ConnectionResetError):
                print("[VideoBridge] Client disconnected from MJPEG stream.")
            except Exception as e:
                print(f"[VideoBridge] MJPEG streaming error: {e}")
        else:
            # Handle other paths (e.g., favicon.ico)
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Error: Path not found. Use /stream.mjpg')

def start_video_bridge_server():
    """Starts the MJPEG HTTP server in its own thread."""
    try:
        httpd = socketserver.ThreadingTCPServer(
            (VIDEO_BRIDGE_HTTP_HOST, VIDEO_BRIDGE_HTTP_PORT), 
            MJPEGStreamHandler
        )
        print(f"[VideoBridge] Serving MJPEG stream on http://{VIDEO_BRIDGE_HTTP_HOST}:{VIDEO_BRIDGE_HTTP_PORT}/stream.mjpg")
        httpd.serve_forever()
    except Exception as e:
        print(f"[VideoBridge] FATAL: Could not start MJPEG server: {e}")
    finally:
        print("[VideoBridge] Server stopped.")

# --- 5. Main Execution ---

if __name__ == '__main__':
    try:
        print("--- J1 Teleoperation Server Starting ---")
        
        # Tells the OS it's OK to re-use ports that are already in use
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        # 1. Start the Video UDP Listener (Highest priority, must catch data)
        t_udp_listener = threading.Thread(target=udp_listener_thread, daemon=True)
        t_udp_listener.start()

        # 2. Start the Video HTTP Server
        t_video_server = threading.Thread(target=start_video_bridge_server, daemon=True)
        t_video_server.start()

        # 3. Start the File Server
        t_file_server = threading.Thread(target=start_file_server, daemon=True)
        t_file_server.start()

        # 4. Start the WebSocket Server (This one runs on the main thread
        #    because its 'asyncio.run_forever()' blocks)
        start_websocket_server()

    except KeyboardInterrupt:
        print("\n--- Server shutting down (Ctrl+C) ---")
    except Exception as e:
        print(f"\n--- FATAL MAIN ERROR --- \n{e}")
    finally:
        sys.exit()