#!/usr/bin/python3
# -*- coding:utf-8 -*-

"""
vision.py - Stable J1 Robot Vision System (Final Tuned Version)

Implements Python-side Non-Maximum Suppression (NMS) and a tuned 
confidence threshold (0.30) to reduce false positives and clean up overlapping detections.
"""

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
    "camera_width": 640, 
    "camera_height": 480,
    "camera_framerate": 30,
    "hef_path": "/usr/share/hailo-models/yolox_s_leaky_h8l_rpi.hef",
    "json_update_rate": 10, 
    
    # 🌟 NEW: Raised threshold above the observed noise floor (0.49)
    "MIN_DETECTION_CONFIDENCE": 0.60, 
    
    # 🌟 NEW: Slightly tightened IoU threshold for better cleanup of overlapping boxes
    "NMS_IOU_THRESHOLD": 0.08
}

# --- CRITICAL CONFIGURATION PATHS (Needed for pipeline linking) ---
HAILOFILTER_SO = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"
HAILOFILTER_FUNCTION = "filter" 

# --- GLOBAL STATE & LOCKS ---
VISION_STATE = {}
STATE_LOCK = threading.Lock()
RUNNING = True
DEVICE_ID = "0001:01:00.0" 

# ============================================================================
# NMS UTILITIES (NEW)
# ============================================================================

def _iou(boxA, boxB):
    """Calculates Intersection over Union (IoU) between two bounding boxes."""
    # box format: (xmin, ymin, xmax, ymax)
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)

    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    iou = interArea / float(boxAArea + boxBArea - interArea)
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
            iou_values = [_iou(boxes[i], boxes[j]) for j in indices[1:]]
            
            # Find indices to delete (those with high IoU)
            indices_to_delete = np.where(np.array(iou_values) > iou_threshold)[0] + 1
            
            # Delete indices with high IoU, but keep the list sorted (best practice for NMS)
            indices = np.delete(indices, indices_to_delete)
            indices = np.delete(indices, 0) # Remove the current highest confidence box
        
        # Add suppressed boxes to the final list
        for index in keep:
            final_detections.append(dets[index])

    return final_detections


# ============================================================================
# CORE LOGIC: DETECTION AND METADATA PARSING 
# ============================================================================

def _process_detections(gst_buffer):
    """
    Extracts Hailo metadata, performs initial Python confidence filtering, 
    and applies Non-Maximum Suppression (NMS).
    """
    frame_start = time.monotonic()
    raw_detections = []

    # Define a list of classes to ignore (common false positives)
    # Adjust this list based on the exact false labels you still see!
    CLASSES_TO_IGNORE = ["tie", "backpack", "bed", "couch", "wine glass"]
    
    try:
        roi = hailo.get_roi_from_buffer(gst_buffer)
        if not roi:
            return

        objects = roi.get_objects()
        
        # Step 1: Apply filtering
        for detection in objects:
            
            # --- NEW BLACKLIST FILTER ---
            if detection.get_label() in CLASSES_TO_IGNORE:
                continue
            # ----------------------------

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
    
def _on_new_sample(appsink):
    """GStreamer callback for new frames."""
    try:
        sample = appsink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        _process_detections(buffer) 
    except Exception as e:
        print(f"\n[CRITICAL] _on_new_sample error: {e}", flush=True)
        return Gst.FlowReturn.ERROR
    return Gst.FlowReturn.OK

# ============================================================================
# UTILITY THREADS (No Change)
# ============================================================================

def json_publisher_loop():
    """Periodically write vision state to JSON file."""
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
# MAIN APPLICATION FLOW
# ============================================================================

def build_pipeline():
    """Construct the final, stable GStreamer pipeline string."""
    
    if not os.path.exists(HAILOFILTER_SO):
        print(f"[FATAL] Hailofilter SO not found at {HAILOFILTER_SO}. Cannot proceed.")
        return None

    # CRITICAL: Pipeline includes hailofilter to correctly generate metadata.
    pipeline_template = (
        "libcamerasrc name=cam_src ! "
        "video/x-raw, format=NV12, width={c_w}, height={c_h}, framerate={c_fps}/1 ! " 
        "videoconvert name=vc1 ! "
        "videoscale name=vs ! "
        "video/x-raw, width=640, height=640, format=RGB ! " 
        "hailonet hef-path={hef} device-id={dev_id} ! " 
        # Reverting to the configuration that generated metadata without crashing
        f"hailofilter so-path={HAILOFILTER_SO} function-name={HAILOFILTER_FUNCTION} ! " 
        "queue name=q_appsink ! "
        "appsink name=vision_sink sync=false drop=true max-buffers=1"
    )
    
    pipeline_str = pipeline_template.format(
        c_w=CONFIG['camera_width'], 
        c_h=CONFIG['camera_height'], 
        c_fps=CONFIG['camera_framerate'],
        hef=CONFIG['hef_path'],
        dev_id=DEVICE_ID,
    )
    
    print(f"--- Pipeline String ---\n{pipeline_str}\n-----------------------")
    
    try:
        pipeline = Gst.parse_launch(pipeline_str)
        return pipeline
    except GLib.Error as e:
        print(f"\n[FATAL] Failed to create pipeline: {e}")
        return None

def start_system():
    """Initializes GStreamer, starts the threads, and runs the main loop."""
    global RUNNING
    
    print("Starting minimal vision system...")
    Gst.init(None)
    
    json_thread = threading.Thread(target=json_publisher_loop, daemon=True)
    json_thread.start()
    
    pipeline = build_pipeline()
    if not pipeline:
        RUNNING = False
        return
    
    appsink = pipeline.get_by_name("vision_sink")
    if not appsink:
        print("[FATAL] Could not find 'vision_sink' element.")
        pipeline.set_state(Gst.State.NULL)
        RUNNING = False
        return
        
    appsink.set_property('emit-signals', True)
    appsink.connect("new-sample", _on_new_sample)
    print("Appsink callback connected.")

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
        
    print("Pipeline started successfully. Running GLib main loop...")
    
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
    except:
        pass
        
    print("Vision system stopped. Exited.")

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    start_system()