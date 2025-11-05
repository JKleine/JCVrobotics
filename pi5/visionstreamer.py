#!/usr/bin/python3
# -*- coding:utf-8 -*-

import os
import sys
import time
import threading
import json
import datetime
import math
import numpy as np # Adding NumPy import for NMS utility

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
ABSOLUTE_NPY_PATH = os.path.join(SCRIPT_DIR, "pygame_frame.npy")

try:
    import hailo
    HAILO_AVAILABLE = True
    print("Hailo libraries loaded successfully.")
except ImportError as e:
    print(f"ERROR: Could not load Hailo/GStreamer libraries: {e}")
    sys.exit(1)

# --- CONFIGURATION (TUNED FOR STABILITY) ---
CONFIG = {
    "json_path": "/dev/shm/vision_data.json",
    # ADDED: Path for Pygame shared memory frame
    "PYGAME_FRAME_PATH": ABSOLUTE_NPY_PATH,
    "NUMPY_UPDATE_RATE": 30, # Target 30 FPS for local display
    
    "camera_width": 640, 
    "camera_height": 480,
    "camera_framerate": 30,
    "hef_path": "/usr/share/hailo-models/yolox_s_leaky_h8l_rpi.hef",
    "json_update_rate": 10, 
    
    # Vision Tuning Parameters
    "MIN_DETECTION_CONFIDENCE": 0.60, 
    "NMS_IOU_THRESHOLD": 0.08,

    # MJPEG Stream Configuration
    "MJPEG_PORT": 8000,
    # --- 
    # --- FIX 1: Set host to 127.0.0.1 to send to webserver.py
    # --- 
    "MJPEG_HOST": "127.0.0.1" 
}

# --- 
# --- 
# --- FIX 2: Restored the CORRECT path from your original stable script ---
# --- 
# --- 
HAILOFILTER_SO = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"
HAILOFILTER_FUNCTION = "filter" 

# --- GLOBAL STATE & LOCKS ---
VISION_STATE = {}
STATE_LOCK = threading.Lock()
RUNNING = True
DEVICE_ID = "0001:01:00.0" 

# NEW: Global buffer and lock for Pygame raw frame
PYGAME_FRAME_BUFFER = None
PYGAME_BUFFER_LOCK = threading.Lock()
# Dimensions for the raw buffer (W*H*3 for RGB)
RAW_BUFFER_SIZE = CONFIG['camera_width'] * CONFIG['camera_height'] * 3


# ============================================================================
# NMS UTILITIES (UNCHANGED)
# ============================================================================

def _iou(boxA, boxB):
    """Calculates Intersection over Union (IoU) between two bounding boxes."""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)

    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    # --- 
    # --- FIX 3: Add 1e-6 (epsilon) to prevent ZeroDivisionError
    # --- 
    iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6)
    return iou

def _non_max_suppression(detections, iou_threshold):
    """
    Performs NMS on a list of detection objects to remove redundant, 
    overlapping boxes of the same class.
    """
    if not detections:
        return []

    # 1. Group detections by label
    detections_by_label = {}
    for det in detections:
        label = det['label']
        if label not in detections_by_label:
            detections_by_label[label] = []
        detections_by_label[label].append(det)

    final_detections = []

    # 2. Process each class independently
    for label, dets in detections_by_label.items():
        # Convert bounding boxes from [xmin, ymin, width, height] to [xmin, ymin, xmax, ymax]
        boxes = np.array([
            [d['box'][0], d['box'][1], d['box'][0] + d['box'][2], d['box'][1] + d['box'][3]]
            for d in dets
        ])
        confidences = np.array([d['confidence'] for d in dets])
        
        # Sort by confidence
        indices = np.argsort(confidences)[::-1]
        
        keep = []
        while len(indices) > 0:
            i = indices[0]
            keep.append(i)

            # Calculate IoU with all remaining boxes
            iou_values = np.array([_iou(boxes[i], boxes[j]) for j in indices[1:]])
            
            # Find indices to delete (those with high IoU)
            indices_to_delete = np.where(iou_values > iou_threshold)[0] + 1
            
            # Delete indices with high IoU, but keep the list sorted (best practice for NMS)
            indices = np.delete(indices, indices_to_delete)
            indices = np.delete(indices, 0) # Remove the current highest confidence box
        
        # Add suppressed boxes to the final list
        for index in keep:
            final_detections.append(dets[index])

    return final_detections


# ============================================================================
# CORE LOGIC: DETECTION, METADATA, AND RAW FRAME PARSING
# ============================================================================

def _process_detections(gst_buffer):
    """
    Extracts Hailo metadata, performs initial Python confidence filtering, 
    and applies Non-Maximum Suppression (NMS).
    (Detection logic is unchanged)
    """
    frame_start = time.monotonic()
    raw_detections = []

    # Define a list of classes to ignore (common false positives)
    CLASSES_TO_IGNORE = ["tie", "backpack", "bed", "couch", "wine glass"]
    
    try:
        # ... (Detection logic remains the same)
        roi = hailo.get_roi_from_buffer(gst_buffer)
        if not roi:
            return

        objects = roi.get_objects()
        
        # Step 1: Apply filtering
        for detection in objects:
            
            if detection.get_label() in CLASSES_TO_IGNORE:
                continue

            confidence = detection.get_confidence()
            
            if confidence < CONFIG["MIN_DETECTION_CONFIDENCE"]:
                continue
                
            # --- Map HailoObject to Python Dictionary ---
            raw_detections.append({
                "label": detection.get_label(),
                "confidence": confidence,
                "center": [int((detection.get_bbox().xmin() + detection.get_bbox().xmax()) / 2 * CONFIG['camera_width']),
                           int((detection.get_bbox().ymin() + detection.get_bbox().ymax()) / 2 * CONFIG['camera_height'])],
                "box": [round(detection.get_bbox().xmin(), 3), round(detection.get_bbox().ymin(), 3),
                        round(detection.get_bbox().width(), 3), round(detection.get_bbox().height(), 3)]
            })
            
        # Step 2: Apply NMS to clean up overlapping and duplicate boxes
        final_detections = _non_max_suppression(raw_detections, CONFIG["NMS_IOU_THRESHOLD"])

    except Exception as e:
        print(f"\n[ERROR] Failed to process detection objects: {e}", flush=True)
        return

    # Update global state
    with STATE_LOCK:
        VISION_STATE['timestamp_utc'] = datetime.datetime.utcnow().isoformat()
        VISION_STATE['detections'] = final_detections
        VISION_STATE['fps_monitor'] = 1.0 / (time.monotonic() - frame_start)
    
def _on_new_vision_sample(appsink):
    """GStreamer callback for new frames processed by Hailo."""
    try:
        sample = appsink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        _process_detections(buffer) 
    except Exception as e:
        print(f"\n[CRITICAL] _on_new_vision_sample error: {e}", flush=True)
        return Gst.FlowReturn.ERROR
    return Gst.FlowReturn.OK


def _on_new_pygame_sample(appsink):
    """
    NEW: GStreamer callback for raw RGB frames destined for Pygame.
    Grabs the raw data and stores it in a global buffer.
    """
    global PYGAME_FRAME_BUFFER
    
    try:
        sample = appsink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.OK

        # 1. Map the Gst.Buffer to access the raw data
        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        
        if not success:
            print("[WARNING] Failed to map buffer for Pygame.")
            return Gst.FlowReturn.OK

        # 2. Get the raw bytes
        raw_data = map_info.data
        
        # 3. Copy data to a global buffer for the NumPy publisher thread
        with PYGAME_BUFFER_LOCK:
            if PYGAME_FRAME_BUFFER is None:
                # Initialize buffer on first call
                PYGAME_FRAME_BUFFER = np.zeros(RAW_BUFFER_SIZE, dtype=np.uint8)

            # Copy raw_data into the NumPy array
            # Assuming buffer is exactly RAW_BUFFER_SIZE bytes (W*H*3) in RGB format
            if len(raw_data) == RAW_BUFFER_SIZE:
                PYGAME_FRAME_BUFFER[:] = np.frombuffer(raw_data, dtype=np.uint8)
            else:
                print(f"[WARNING] Pygame buffer size mismatch. Expected {RAW_BUFFER_SIZE}, got {len(raw_data)}")


        # 4. Unmap the buffer
        buffer.unmap(map_info)
        
    except Exception as e:
        print(f"\n[CRITICAL] _on_new_pygame_sample error: {e}", flush=True)
        return Gst.FlowReturn.ERROR
        
    return Gst.FlowReturn.OK

# ============================================================================
# UTILITY THREADS (MODIFIED)
# ============================================================================

def numpy_publisher_loop():
    """
    NEW: Periodically save the raw NumPy frame buffer to shared memory
    for low-latency consumption by display_app.py (Pygame).
    """
    global RUNNING
    print(f"NumPy publisher started. Saving frames to {CONFIG['PYGAME_FRAME_PATH']}")
    update_interval = 1.0 / CONFIG["NUMPY_UPDATE_RATE"]
    
    # --- THIS IS THE FIX ---
    # 1. Define the FINAL path (e.g., ".../pygame_frame.npy")
    final_path = CONFIG["PYGAME_FRAME_PATH"]
    
    # 2. Define a TEMP path *base* (without the .npy extension)
    #    We'll use a different name to be safe.
    temp_path_base = os.path.join(SCRIPT_DIR, "pygame_frame_temp")
    
    # 3. Define the *actual* file that np.save will create
    temp_file_with_ext = temp_path_base + ".npy"
    # --- END FIX ---

    while RUNNING:
        try:
            frame_to_save = None
            if PYGAME_FRAME_BUFFER is not None:
                with PYGAME_BUFFER_LOCK:
                    frame_to_save = PYGAME_FRAME_BUFFER.reshape(
                        (CONFIG['camera_height'], CONFIG['camera_width'], 3)
                    ).copy()
            
            if frame_to_save is not None:
                
                # 1. Save to the temporary file base. 
                #    np.save will add ".npy" automatically, creating "pygame_frame_temp.npy"
                np.save(temp_path_base, frame_to_save, allow_pickle=False)
                
                # 2. Atomically rename the *actual* temp file ("..._temp.npy")
                #    to the *final* file ("..._frame.npy")
                os.rename(temp_file_with_ext, final_path)

            time.sleep(update_interval)
            
        except Exception as e:
            if RUNNING:
                print(f"[NumPy Publisher Error] {e}") 
            time.sleep(1.0)
            
    print("NumPy publisher stopped.")


def json_publisher_loop():
    """Periodically write vision state to JSON file."""
    # ... (This function remains largely the same, just keeping the state publisher running)
    global RUNNING
    print("JSON publisher started.")
    update_interval = 1.0 / CONFIG["json_update_rate"]
    
    while RUNNING:
        try:
            with STATE_LOCK:
                state = VISION_STATE.copy()

            if not state:
                state = {
                    "timestamp_utc": datetime.datetime.utcnow().isoformat(),
                    "status": "waiting_for_detections",
                    "detections": [],
                    "fps_monitor": 0.0
                }
            
            temp_path = CONFIG["json_path"] + ".tmp"
            with open(temp_path, 'w') as f:
                json.dump(state, f, indent=2)
            os.rename(temp_path, CONFIG["json_path"])
            
            time.sleep(update_interval)
            
        except Exception as e:
            if RUNNING:
                print(f"[JSON Publisher Error] {e}")
            time.sleep(1.0)
    
    print("JSON publisher stopped.")


# ============================================================================
# MAIN APPLICATION FLOW (MODIFIED)
# ============================================================================

def build_pipeline():
    """
    Construct the GStreamer pipeline string, now incorporating a 'tee' element
    to split the stream for Vision Processing, MJPEG Web Streaming, and 
    Pygame Raw Frame Export.
    """
    
    if not os.path.exists(HAILOFILTER_SO):
        print(f"[FATAL] Hailofilter SO not found at {HAILOFILTER_SO}. Cannot proceed.")
        print("    This is the #1 cause of pipeline failure.")
        return None

    # CRITICAL: Pipeline uses two 'tee' elements.
    # TEE 1 (t1): Splits raw camera output.
    #   Branch 1.1: Pygame Raw Frame export (to appsink-pygame)
    #   Branch 1.2: General Video Processing (to TEE 2)

    pipeline_template = (
        "libcamerasrc name=cam_src ! "
        "video/x-raw, format=NV12, width={c_w}, height={c_h}, framerate={c_fps}/1 ! " 
        
        # --- TEE 1: Split for Raw Pygame Capture (Lowest latency branch) ---
        "tee name=t1 ! "
        
        # --- Branch 1.1: Pygame Raw Frame Export ---
        "queue name=q_pygame ! "
        "videoconvert ! " # Convert NV12 to RGB for Pygame
        "video/x-raw, format=RGB, width={c_w}, height={c_h} ! " 
        "appsink name=appsink-pygame sync=false drop=true max-buffers=1 "
        
        # --- Branch 1.2: General Processing (to TEE 2) ---
        "t1. ! queue name=q_processing ! tee name=t2 ! "
        
        # --- Branch 2.1: Vision (via Hailo) ---
        "queue name=q_vision ! "
        "videoconvert name=vc1 ! "
        "videoscale name=vs ! "
        "video/x-raw, width=640, height=640, format=RGB ! " 
        "hailonet hef-path={hef} device-id={dev_id} ! " 
        f"hailofilter so-path={HAILOFILTER_SO} function-name={HAILOFILTER_FUNCTION} ! " 
        "queue name=q_appsink ! "
        "appsink name=vision_sink sync=false drop=true max-buffers=1 "
        
        # --- Branch 2.2: Web Stream (via MJPEG) ---
        "t2. ! queue name=q_streamer ! "
        "videoconvert ! " # Convert NV12 to I420 (common for jpegenc)
        "jpegenc quality=80 ! " 
        f"multiudpsink clients={CONFIG['MJPEG_HOST']}:{CONFIG['MJPEG_PORT']}" 
    )
    
    pipeline_str = pipeline_template.format(
        c_w=CONFIG['camera_width'], 
        c_h=CONFIG['camera_height'], 
        c_fps=CONFIG['camera_framerate'],
        hef=CONFIG['hef_path'],
        dev_id=DEVICE_ID,
    )
    
    print(f"--- Pipeline String (Triple Stream) ---\n{pipeline_str}\n-----------------------")
    
    try:
        pipeline = Gst.parse_launch(pipeline_str)
        return pipeline
    except GLib.Error as e:
        print(f"\n[FATAL] Failed to create pipeline: {e}")
        return None

def start_system():
    """Initializes GStreamer, starts the threads, and runs the main loop."""
    global RUNNING
    
    print("Starting TRIPLE stream vision system...")
    Gst.init(None)
    
    # Start publisher threads
    json_thread = threading.Thread(target=json_publisher_loop, daemon=True)
    json_thread.start()
    
    numpy_thread = threading.Thread(target=numpy_publisher_loop, daemon=True)
    numpy_thread.start()
    
    pipeline = build_pipeline()
    if not pipeline:
        RUNNING = False
        return
    
    # --- Connect Vision Appsink ---
    vision_appsink = pipeline.get_by_name("vision_sink")
    if not vision_appsink:
        print("[FATAL] Could not find 'vision_sink' element.")
        pipeline.set_state(Gst.State.NULL)
        RUNNING = False
        return
        
    vision_appsink.set_property('emit-signals', True)
    vision_appsink.connect("new-sample", _on_new_vision_sample)
    print("Vision Appsink callback connected.")

    # --- Connect Pygame Appsink ---
    pygame_appsink = pipeline.get_by_name("appsink-pygame")
    if not pygame_appsink:
        print("[FATAL] Could not find 'appsink-pygame' element for Pygame feed.")
        pipeline.set_state(Gst.State.NULL)
        RUNNING = False
        return
        
    pygame_appsink.set_property('emit-signals', True)
    pygame_appsink.connect("new-sample", _on_new_pygame_sample)
    print("Pygame Appsink callback connected.")


    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message::error", on_bus_error)
    bus.connect("message::eos", lambda bus, msg: GLib.MainLoop().quit())

    print("Setting pipeline state to PLAYING...")
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("[FATAL] Unable to set pipeline to PLAYING state.")
        RUNNING = False
        return
        
    print(f"Pipeline started successfully. Vision outputting to JSON. MJPEG streaming to {CONFIG['MJPEG_HOST']}:{CONFIG['MJPEG_PORT']}...")
    
    try:
        mainloop = GLib.MainLoop()
        mainloop.run()
    except Exception as e:
        print(f"\nGLib main loop error: {e}")
    finally:
        stop_system(pipeline)

def on_bus_error(bus, message):
    """Handle pipeline errors."""
    err, debug = message.parse_error()
    print(f"\n[PIPELINE ERROR] {err}")
    print(f"Debug info: {debug}")
    GLib.MainLoop().quit()
    
def stop_system(pipeline):
    """Stop vision system and cleanup."""
    global RUNNING
    if not RUNNING:
        return
        
    print("Stopping vision system...")
    RUNNING = False
    
    if pipeline:
        pipeline.set_state(Gst.State.NULL)
    
    try:
        if os.path.exists(CONFIG["json_path"]):
            os.remove(CONFIG["json_path"])
        if os.path.exists(CONFIG["PYGAME_FRAME_PATH"]):
            os.remove(CONFIG["PYGAME_FRAME_PATH"])
            print(f"Cleaned up shared NumPy frame at {CONFIG['PYGAME_FRAME_PATH']}")
    except Exception as e:
        print(f"Error during cleanup: {e}")
        
    print("Vision system stopped. Exited.")

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    start_system()