"""
================================================================================
INTERACT-A-RAVE: STANDALONE MYO DSP NODE
================================================================================
Ensure correct port connection for the Myo dongle and run this script

Usage: python3 myo_node.py --port /dev/tty.usbmodem11
Usage (Remote): python3 myo_node.py --ip 192.168.2.1 --ip_port 8000 --ip2 192.168.2.2 --ip2_port 8001
================================================================================
"""

import sys
import math
import argparse
import time
import numpy as np
import scipy.signal as signal
from pyomyo import Myo, emg_mode
from pythonosc.udp_client import SimpleUDPClient

# ===============================================================================
# NETWORK & HARDWARE CONFIGURATION
# ===============================================================================
parser = argparse.ArgumentParser(description="Standalone Myo DSP Node")
parser.add_argument("--port", default="/dev/tty.usbmodem11", help="Myo Dongle Port")
parser.add_argument("--ip", default="127.0.0.1", help="Target IP for OSC (Default: Localhost)")
parser.add_argument("--ip_port", type=int, default=8000, help="Port for OSC output (Default: 8000)")

parser.add_argument("--ip2", default="127.0.0.1", help="Second Target IP for remote")
parser.add_argument("--ip2_port", type=int, default=8001, help="Port for the second IP")


args = parser.parse_args()

a_client = SimpleUDPClient(args.ip, args.ip_port)
b_client = SimpleUDPClient(args.ip2, args.ip2_port)

FS = 200.0  # Myo hardware sampling frequency

# 1. 60Hz Notch Filter (AC Mains hum)
b_notch, a_notch = signal.iirnotch(w0=60.0, Q=30.0, fs=FS)

# 2. Bandpass Filter (10Hz to 85Hz, 2nd-order Butterworth)
b_bp, a_bp = signal.butter(N=2, Wn=[10.0, 85.0], btype='bandpass', fs=FS)

# 3. Initialize Filter States (zi) for 8 independent channels
# The state arrays must be shape (8, filter_order) to track each pod independently
zi_notch = np.zeros((8, len(a_notch) - 1))
zi_bp = np.zeros((8, len(a_bp) - 1))

def broadcast_osc(address, payload):
    """Sends data simultaneously to TouchDesigner, Max, and the Remote IP"""
    a_client.send_message(address, payload)
    b_client.send_message(address, payload)

# ===============================================================================
# DSP STATE & INDEPENDENT EMA MATH
# ===============================================================================
# 1.0 = No smoothing (raw). 0.01 = Maximum sluggish smoothing.
EMA_ALPHA_ACCEL = 0.30  # Fast: G-force spikes need to be snappy for impact hits
EMA_ALPHA_GYRO  = 0.15  # Medium: Rotational speed needs a slight glide
EMA_ALPHA_EMG   = 0.05  # Slow: Muscle noise is chaotic and needs a heavy leash

myo_state = {
    "emg_pods": [0.0] * 8, # Tracks the previous state of all 8 individual sensors
    "emg_amp": 0.0,
    "accel": [0.0, 0.0, 0.0],
    "gyro":  [0.0, 0.0, 0.0]
}

def apply_ema(current_val, prev_val, alpha):
    return (alpha * current_val) + ((1.0 - alpha) * prev_val)

def map_and_clamp(value, in_min, in_max, out_min, out_max):
    # Find where the raw value sits as a percentage (0.0 to 1.0)
    mapped_ratio = (value - in_min) / (in_max - in_min)
    
    # Clamp it so it doesn't exceed our boundaries
    clamped_ratio = max(0.0, min(mapped_ratio, 1.0))
    
    # Scale that percentage up to our new output range
    return (clamped_ratio * (out_max - out_min)) + out_min

# ===============================================================================
# HARDWARE HANDLERS
# ===============================================================================
def on_emg(emg, moving):
    """Applies DSP filtering, normalizes the 8 pods, and calculates overall tension."""
    global zi_notch, zi_bp
    
    # 1. Reshape the incoming tuple into a numpy array of shape (8, 1)
    # This represents 8 independent channels, each receiving 1 new sample
    raw_samples = np.array(emg).reshape(8, 1)
    
    # 2. Apply Notch Filter (60 Hz)
    notched, zi_notch = signal.lfilter(b_notch, a_notch, raw_samples, axis=1, zi=zi_notch)
    
    # 3. Apply Bandpass Filter (10 Hz - 85 Hz)
    filtered, zi_bp = signal.lfilter(b_bp, a_bp, notched, axis=1, zi=zi_bp)
    
    # 4. Flatten the clean data back to a 1D array for standard Python iteration
    clean_emg = filtered.flatten()
    
    # 5. Normalize hardware 8-bit (-128 to 127) values to 0.0-1.0 floats
    norm_emg = [max(0.0, min((val + 128) / 255.0, 1.0)) for val in clean_emg]
    
    # 6. Smooth and Broadcast all 8 individual pods
    for i in range(8):
        myo_state["emg_pods"][i] = apply_ema(norm_emg[i], myo_state["emg_pods"][i], EMA_ALPHA_EMG)
        broadcast_osc(f"/myo/emg/norm/{i}", myo_state["emg_pods"][i])
        
    # 7. Overall Amplitude (RMS) - The "Clench" Metric
    rms = math.sqrt(sum(val**2 for val in clean_emg) / 8.0)
    raw_amp = min(rms / 100.0, 1.0)
    
    # Apply the heavy EMG smoothing leash to the overall amplitude
    myo_state["emg_amp"] = apply_ema(raw_amp, myo_state["emg_amp"], EMA_ALPHA_EMG)
    broadcast_osc("/myo/emg/amplitude", myo_state["emg_amp"])

def on_imu(quat, acc, gyro):
    """Catches spatial data, maps ranges, and applies independent smoothing."""
    
    # 1. Accelerometer (Mapped from -4G to +4G --> Output -4.0 to 4.0)
    norm_acc_x = map_and_clamp(acc[0], -4.0, 4.0, -1.0, 1.0)
    norm_acc_y = map_and_clamp(acc[1], -4.0, 4.0, -1.0, 1.0)
    norm_acc_z = map_and_clamp(acc[2], -4.0, 4.0, -1.0, 1.0)

    myo_state["accel"][0] = apply_ema(norm_acc_x, myo_state["accel"][0], EMA_ALPHA_ACCEL)
    myo_state["accel"][1] = apply_ema(norm_acc_y, myo_state["accel"][1], EMA_ALPHA_ACCEL)
    myo_state["accel"][2] = apply_ema(norm_acc_z, myo_state["accel"][2], EMA_ALPHA_ACCEL)
    
    broadcast_osc("/myo/accel/x", myo_state["accel"][0])
    broadcast_osc("/myo/accel/y", myo_state["accel"][1])
    broadcast_osc("/myo/accel/z", myo_state["accel"][2])
    
    # 2. Gyroscope (Mapped from -2000 to +2000 deg/sec --> Output 0 to 360)
    # Dead center (0 rotational velocity) will perfectly output 180.
    deg_gyro_x = map_and_clamp(gyro[0], -2000.0, 2000.0, 0.0, 360.0)
    deg_gyro_y = map_and_clamp(gyro[1], -2000.0, 2000.0, 0.0, 360.0)
    deg_gyro_z = map_and_clamp(gyro[2], -2000.0, 2000.0, 0.0, 360.0)

    myo_state["gyro"][0] = apply_ema(deg_gyro_x, myo_state["gyro"][0], EMA_ALPHA_GYRO)
    myo_state["gyro"][1] = apply_ema(deg_gyro_y, myo_state["gyro"][1], EMA_ALPHA_GYRO)
    myo_state["gyro"][2] = apply_ema(deg_gyro_z, myo_state["gyro"][2], EMA_ALPHA_GYRO)
    
    broadcast_osc("/myo/gyro/x", myo_state["gyro"][0])
    broadcast_osc("/myo/gyro/y", myo_state["gyro"][1])
    broadcast_osc("/myo/gyro/z", myo_state["gyro"][2])

def on_battery(level):
    """Tracks battery health."""
    print(f"[BATTERY] Myo level: {level}%")
    broadcast_osc("/myo/state/battery", level)

# ===============================================================================
# MAIN NODE LOOP
# ===============================================================================
def run_myo_node():
    print("==========================================")
    print("MYO DSP NODE: BULLETPROOF EDITION V2")
    print(f"Targeting OSC outputs at {args.ip}:{args.ip_port} and {args.ip2}:{args.ip2_port}")
    print("==========================================")

    myo_device = Myo(mode=emg_mode.RAW)
    
    myo_device.add_emg_handler(on_emg)
    myo_device.add_imu_handler(on_imu)
    myo_device.add_battery_handler(on_battery)
    
    try:
        print(f"Attempting to connect to dongle on {args.port}...")
        myo_device.connect()
        print("[CONNECTED] Streaming data...")
        
        # Turn on data streaming
        myo_device.write_attr(0x11, b'\x01\x10')
        
        while True:
            # Native pyomyo poll
            myo_device.run() 
            # Protect the CPU
            time.sleep(0.001)
            
    except Exception as e:
        print(f"\n[FATAL ERROR] Check dongle connection or port string: {e}")
        sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n[STOP] Myo DSP Node shutting down safely.")
        
    finally:
        myo_device.disconnect()
        print("Hardware disconnected.")

if __name__ == '__main__':
    run_myo_node()