"""
================================================================================
INTERACT-A-RAVE: STANDALONE MUSE DSP NODE
================================================================================
Usage (Local):  python3 muse_node.py
Usage (Remote): python3 muse_node.py --ip 192.168.2.1 --ip2 192.xxx. --no_delta
================================================================================
"""

import time
import sys
import math
import argparse
import numpy as np
from scipy.signal import welch, butter, lfilter
from pylsl import resolve_streams, StreamInlet
from pythonosc.udp_client import SimpleUDPClient

# ===============================================================================
# NETWORK CONFIGURATION (ADAPTABLE VIA CLI)
# ===============================================================================
parser = argparse.ArgumentParser(description="Standalone Muse DSP Node")
parser.add_argument("--ip", default="127.0.0.1", help="Target IP for OSC (Default: Localhost)")
parser.add_argument("--ip_port", type=int, default=8000, help="Port for OSC output (Default: 8000)")
parser.add_argument("--ip2", default="127.0.0.1", help="Second Target IP for remote")
parser.add_argument("--ip2_port", type=int, default=8001, help="Port for the second IP")
parser.add_argument("--no_delta", action="store_true", help="Excludes Delta from the relative power calculation")
args = parser.parse_args()

a_client = SimpleUDPClient(args.ip, args.ip_port)
b_client = SimpleUDPClient(args.ip2, args.ip2_port)

def broadcast_osc(address, payload):
    a_client.send_message(address, payload)
    b_client.send_message(address, payload)

# ===============================================================================
# DSP & THRESHOLD CONFIGURATION
# ===============================================================================
BLINK_THRESHOLD = 150.0      
MOTION_THRESHOLD = 20.0      

# EEG Window Settings
FS_EEG = 256
WIN_SEC = 2.0
WIN_SAMPLES = int(FS_EEG * WIN_SEC)

BANDS_EXTENDED = {
    "delta": (2, 4),
    "theta": (4, 8),
    "alpha": (8, 13),
    "beta": (13, 30),
    "gamma": (30, 40)
}

def create_bandpass_filter(lowcut=2.0, highcut=40.0, fs=256, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, [lowcut/nyq, highcut/nyq], btype='band')
    return b, a

B_FILT, A_FILT = create_bandpass_filter()

def calculate_bandpower(psd_matrix, freqs, fmin, fmax):
    idx = np.logical_and(freqs >= fmin, freqs <= fmax)
    if not np.any(idx): return 0.0
    return float(np.mean(np.sum(psd_matrix[:, idx], axis=1)))

# ===============================================================================
# MAIN NODE LOOP
# ===============================================================================
def run_muse_node():
    print("MUSE DSP NODE: INITIALIZING")
    print(f"Targeting Audio/Visual Rig at {args.ip} (Ports {args.ip_port} & {args.ip2_port})")
    print("\nSearching for LSL Streams... (Ensure 'muselsl stream --acc --gyro' is running)")
    
    streams = resolve_streams(wait_time=4.0)
    
    eeg_stream = next((s for s in streams if s.type() == 'EEG'), None)
    acc_stream = next((s for s in streams if s.type() in ['Accelerometer', 'ACC']), None)
    gyro_stream = next((s for s in streams if s.type() in ['Gyroscope', 'GYRO']), None)
    
    if not eeg_stream:
        print("[ERROR] No EEG stream found. Shutting down node.")
        sys.exit(1)

    eeg_inlet = StreamInlet(eeg_stream, max_buflen=360, max_chunklen=100)
    print("[SUCCESS] EEG Connected.")
    
    acc_inlet = StreamInlet(acc_stream, max_chunklen=100) if acc_stream else None
    if acc_inlet: print("[SUCCESS] Accelerometer Connected.")
        
    gyro_inlet = StreamInlet(gyro_stream, max_chunklen=100) if gyro_stream else None
    if gyro_inlet: print("[SUCCESS] Gyroscope Connected.")

    print("\n>>> NODE ACTIVE: Processing DSP & Broadcasting OSC <<<")

    # Local State Variables
    eeg_buffer = []
    current_gyro_mag = 0.0

    try:
        while True:
            if acc_inlet:
                acc_chunk, _ = acc_inlet.pull_chunk(timeout=0.0)
                if acc_chunk:
                    latest_acc = acc_chunk[-1][:3]
                    broadcast_osc("/muse/accel/x", latest_acc[0])
                    broadcast_osc("/muse/accel/y", latest_acc[1])
                    broadcast_osc("/muse/accel/z", latest_acc[2])

            if gyro_inlet:
                gyro_chunk, _ = gyro_inlet.pull_chunk(timeout=0.0)
                if gyro_chunk:
                    latest_gyro = gyro_chunk[-1][:3]
                    broadcast_osc("/muse/gyro/x", latest_gyro[0])
                    broadcast_osc("/muse/gyro/y", latest_gyro[1])
                    broadcast_osc("/muse/gyro/z", latest_gyro[2])
                    # Update magnitude for motion tracking
                    current_gyro_mag = math.sqrt(latest_gyro[0]**2 + latest_gyro[1]**2 + latest_gyro[2]**2)

            eeg_chunk, _ = eeg_inlet.pull_chunk(timeout=0.0)
            if eeg_chunk:
                for sample in eeg_chunk:
                    eeg_buffer.append(sample[:4])
            
            # Execute math only when buffer is full (2 Seconds of Data)
            if len(eeg_buffer) >= WIN_SAMPLES:
                raw_data = np.array(eeg_buffer).T
                
                # Blink Detection (Channels 1 & 2 are AF7 and AF8 above the eyes)
                frontal_data = raw_data[[1, 2], :]
                frontal_p2p = np.max(frontal_data) - np.min(frontal_data)
                broadcast_osc("/muse/state/blink", 1 if frontal_p2p > BLINK_THRESHOLD else 0)

                if current_gyro_mag > MOTION_THRESHOLD:
                    broadcast_osc("/muse/state/motion_artifact", 1)
                    # Shift buffer slightly to clear bad data and skip DSP calculation
                    eeg_buffer = eeg_buffer[int(WIN_SAMPLES * 0.2):] 
                    continue 
                else:
                    broadcast_osc("/muse/state/motion_artifact", 0)
                
                clean_data = lfilter(B_FILT, A_FILT, raw_data, axis=1)
                freqs, psd = welch(clean_data, fs=FS_EEG, nperseg=256, axis=1)
                
                abs_powers = {}
                total_power = 0.0
                
                for band, (fmin, fmax) in BANDS_EXTENDED.items():
                    p_val = calculate_bandpower(psd, freqs, fmin, fmax)
                    abs_powers[band] = p_val
                    total_power += p_val
                    broadcast_osc(f"/muse/power/absolute/{band}", p_val)

                # The Logic Gate: Skip adding to total_power if flag is triggered on Delta
                    if args.no_delta and band == "delta":
                        pass 
                    else:
                        total_power += p_val

                broadcast_osc("/muse/power/absolute/total", total_power)

                # Prevent divide-by-zero crashes
                if total_power <= 0.0:
                    total_power = 0.0001
                    
                for band, p_val in abs_powers.items():
                    # The Logic Gate: Force relative Delta to 0.0 if excluded
                    if args.no_delta and band == "delta":
                        broadcast_osc(f"/muse/power/relative/{band}", 0.0)
                    else:
                        rel_val = p_val / total_power
                        broadcast_osc(f"/muse/power/relative/{band}", rel_val)

                # Slide the sliding window by 10% for the next iteration
                eeg_buffer = eeg_buffer[int(WIN_SAMPLES * 0.1):]

            # Sleep briefly to yield CPU
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[STOP] Muse Node shutting down safely.")
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")

if __name__ == '__main__':
    run_muse_node()