#!/bin/bash
#
# J1 Robot Launcher Script (for SSH / tmux)
#

echo "--- Starting J1 Robot Services in tmux session 'J1_Robot' ---"

# --- Define Paths ---
PROJECT_DIR="$HOME/Desktop/J1"
HAILO_DIR="$HOME/hailo-rpi5-examples"
SESSION_NAME="J1_Robot"

# --- Kill any old session to ensure a clean start ---
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

# --- Create the new detached session ---
# We create the first window (index 0) and run sensors.py
echo "Starting window 0: Sensors"
tmux new-session -d -s $SESSION_NAME -n "Sensors" \
    "bash -c 'cd $PROJECT_DIR; python sensors.py'"

# --- 2. Control (window 1) ---
echo "Starting window 1: Control"
tmux new-window -t $SESSION_NAME: -n "Control" \
    "bash -c 'cd $PROJECT_DIR; python control.py'"

# --- 3. Webserver (window 2) ---
echo "Starting window 2: Webserver (venv)"
tmux new-window -t $SESSION_NAME: -n "Webserver" \
    "bash -c 'cd $PROJECT_DIR; source venv/bin/activate; python webserver.py'"

# --- 4. Display (window 3) ---
echo "Starting window 3: Display"
tmux new-window -t $SESSION_NAME: -n "Display" \
    "bash -c 'cd $PROJECT_DIR; python display.py'"

# --- 5. Operator (window 4) ---
echo "Starting window 4: Operator"
tmux new-window -t $SESSION_NAME: -n "Operator" \
    "bash -c 'cd $PROJECT_DIR; python robot_operator.py'"

# --- 6. Speech (window 5) ---
echo "Starting window 5: Speech (venv)"
tmux new-window -t $SESSION_NAME: -n "Speech" \
    "bash -c 'cd $PROJECT_DIR; source venv/bin/activate; python speech.py'"

# --- 7. Vision Streamer (window 6) ---
echo "Starting window 6: Vision (hailo)"
tmux new-window -t $SESSION_NAME: -n "Vision" \
    "bash -c 'cd $HAILO_DIR; source setup_env.sh; python $PROJECT_DIR/visionstreamer.py'"

# Select the first window to be the one you see
tmux select-window -t $SESSION_NAME:0

echo "--- All services started. ---"
echo "To attach to the session, run:"
echo "tmux attach -t $SESSION_NAME"