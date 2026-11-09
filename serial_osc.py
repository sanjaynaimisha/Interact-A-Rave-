import serial
import serial.tools.list_ports
import time
import sys
import threading
import math
import argparse
from collections import deque
from pythonosc import udp_client
from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server

BAUD_RATE = 115200

parser = argparse.ArgumentParser(description="Standalone ESP32 Serial to OSC Bridge")
parser.add_argument("--port", default="AUTO", help="ESP32 Serial Port (Default: Auto-detect)")
parser.add_argument("--ip", default="127.0.0.1", help="Target IP for OSC (Default: Localhost)")
parser.add_argument("--ip_port", type=int, default=8000, help="Port for OSC output (Default: 8000)")
parser.add_argument("--ip2", default="192.168.1.100", help="Second Target IP for remote")
parser.add_argument("--ip2_port", type=int, default=8001, help="Port for the second IP")
args = parser.parse_args()

CALIBRATION_TIME = 10.0 
GSR_WINDOW_SECONDS = 30 
GSR_SAMPLE_RATE = 20 
GSR_BUFFER_SIZE = GSR_WINDOW_SECONDS * GSR_SAMPLE_RATE

OSC_IN_IP = "0.0.0.0" 
OSC_IN_PORT = 8003

# ==========================================
# STATE MACHINE VARIABLES
# ==========================================
class PPGState:
    def __init__(self):
        self.mode = "IDLE" 
        self.cal_start_time = 0
        self.buffer = []
        self.min_val = 0
        self.max_val = 1

class GSRState:
    def __init__(self):
        self.mode = "IDLE"
        self.cal_start_time = 0
        self.history = deque(maxlen=GSR_BUFFER_SIZE)
        self.prev_filtered = None
        self.prev_time = None
        self.deriv_buffer = [] 
        self.noise_floor = 0.0 

ppg = PPGState()
gsr = GSRState()

# Initialize dual OSC clients using argparse variables
a_client = udp_client.SimpleUDPClient(args.ip, args.ip_port)
b_client = udp_client.SimpleUDPClient(args.ip2, args.ip2_port)

def broadcast_osc(address, payload):
    """Sends data simultaneously to TouchDesigner, Max, and the Remote IP"""
    a_client.send_message(address, payload)
    b_client.send_message(address, payload)

def trigger_ppg_cal(address=None, *args):
    if args and args[0] == 0: return 
    print("\n[!] PPG Calibration Started. Hold still for 10 seconds...")
    ppg.buffer = []
    ppg.cal_start_time = time.time()
    ppg.mode = "CALIBRATING"
    broadcast_osc("/sys/state/ppg", "CALIBRATING")

def finish_ppg_cal():
    if len(ppg.buffer) < 10:
        print("\n[X] PPG Calibration Failed: Not enough data.")
        ppg.mode = "IDLE"
        return
    ppg.min_val = min(ppg.buffer)
    ppg.max_val = max(ppg.buffer)
    ppg.baseline = sum(ppg.buffer) / len(ppg.buffer)
    if ppg.max_val == ppg.min_val: ppg.max_val += 1 
    ppg.mode = "ACTIVE"
    print(f"\n[*] PPG Calibrated! Min: {ppg.min_val:.1f} | Max: {ppg.max_val:.1f}")
    broadcast_osc("/sys/state/ppg", "ACTIVE")

def trigger_gsr_cal(address=None, *args):
    if args and args[0] == 0: return 
    print("\n[!] GSR Calibration Started. Take a deep breath to set your range...")
    gsr.history.clear()
    gsr.deriv_buffer = []
    gsr.cal_start_time = time.time()
    gsr.mode = "CALIBRATING"
    broadcast_osc("/sys/state/gsr", "CALIBRATING")

def finish_gsr_cal():
    if len(gsr.history) < 20:
        print("\n[X] GSR Calibration Failed: Not enough data.")
        gsr.mode = "IDLE"
        return
    
    if gsr.deriv_buffer:
        gsr.noise_floor = max(gsr.deriv_buffer) * 1.1
    else:
        gsr.noise_floor = 0.0
        
    gsr.mode = "ACTIVE"
    print(f"\n[*] GSR Calibrated! Baseline locked. Glitch threshold: {gsr.noise_floor:.2f}")
    broadcast_osc("/sys/state/gsr", "ACTIVE")

def normalize(value, min_v, max_v):
    norm = (value - min_v) / (max_v - min_v)
    return max(0.0, min(1.0, norm))

def terminal_listener():
    while True:
        cmd = sys.stdin.readline().strip().lower()
        if cmd == 'p': trigger_ppg_cal()
        elif cmd == 'g': trigger_gsr_cal()

def start_osc_server():
    dispatcher = Dispatcher()
    dispatcher.map("/cal/ppg", trigger_ppg_cal)
    dispatcher.map("/cal/gsr", trigger_gsr_cal)
    server = osc_server.ThreadingOSCUDPServer((OSC_IN_IP, OSC_IN_PORT), dispatcher)
    server.serve_forever()

threading.Thread(target=terminal_listener, daemon=True).start()
threading.Thread(target=start_osc_server, daemon=True).start()

def find_esp_port():
    print("Scanning for USB Serial devices...")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("ERROR: No serial ports found. Is the ESP32 plugged in?")
        return None

    for port in ports:
        if "usbserial" in port.device.lower() or "usbmodem" in port.device.lower():
            print(f"-> Auto-detected ESP32 on: {port.device} ({port.description})")
            return port.device
            
    print("\nCould not automatically identify the ESP32. Available ports:")
    for port in ports:
        print(f" - {port.device} : {port.description}")
        
    return None

# Use argparse port if specified, otherwise auto-detect
if args.port == "AUTO":
    SERIAL_PORT = find_esp_port()
else:
    SERIAL_PORT = args.port

if SERIAL_PORT is None:
    print("\nExiting. Please check your USB connection.")
    sys.exit()

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"\nSUCCESS: Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    print(f"-> Sending Primary OSC to {args.ip}:{args.ip_port}")
    print(f"-> Sending Remote OSC to {args.ip2}:{args.ip2_port}")
    print(f"<- Listening for incoming TouchOSC triggers on Port {OSC_IN_PORT}")
    print("\nCOMMANDS: Type 'p' + Enter for PPG | Type 'g' + Enter for GSR\n")
except Exception as e:
    print(f"\nFAILED to open serial port: {e}")
    print("Ensure the Arduino IDE/PlatformIO Serial Monitor is CLOSED.")
    sys.exit()

# MAIN LOOP
try:
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
            except UnicodeDecodeError: continue 
            if not line: continue
                
            parts = line.split()
            if len(parts) >= 2:
                osc_address = parts[0]
                osc_values = [float(v) if '.' in v else int(v) for v in parts[1:]]
                
                # --- PPG LOGIC ---
                if osc_address == "/ppg/data" and len(osc_values) >= 4:
                    raw_val = osc_values[0]
                    filtered_val = osc_values[1]
                    
                    if ppg.mode == "CALIBRATING":
                        ppg.buffer.append(filtered_val)
                        if time.time() - ppg.cal_start_time >= CALIBRATION_TIME:
                            finish_ppg_cal()
                            
                    elif ppg.mode == "ACTIVE":
                        if filtered_val > ppg.max_val: ppg.max_val = filtered_val
                        elif filtered_val < ppg.min_val: ppg.min_val = filtered_val
                        norm_val = normalize(filtered_val, ppg.min_val, ppg.max_val)
                        broadcast_osc("/ppg/normalized", norm_val)
                    
                    broadcast_osc("/ppg/raw", raw_val)
                    broadcast_osc("/ppg/filtered", filtered_val)
                    broadcast_osc("/ppg/bpm", osc_values[2])
                    broadcast_osc("/ppg/hrv", osc_values[3])
                
                # --- GSR LOGIC ---
                elif osc_address == "/gsr/data" and len(osc_values) >= 3:
                    filtered_val = osc_values[0]
                    tonic_val = osc_values[1]
                    phasic_val = osc_values[2]
                    current_time = time.time()
                    
                    raw_derivative = 0.0
                    if gsr.prev_filtered is not None and gsr.prev_time is not None:
                        dt = current_time - gsr.prev_time
                        if dt > 0:
                            raw_derivative = (filtered_val - gsr.prev_filtered) / dt
                    
                    gsr.prev_filtered = filtered_val
                    gsr.prev_time = current_time

                    if gsr.mode == "CALIBRATING":
                        gsr.history.append(tonic_val)
                        if raw_derivative > 0:
                            gsr.deriv_buffer.append(raw_derivative)
                            
                        if current_time - gsr.cal_start_time >= CALIBRATION_TIME:
                            finish_gsr_cal()

                    elif gsr.mode == "ACTIVE":
                        gsr.history.append(tonic_val)
                        glitch_intensity = max(0.0, raw_derivative - gsr.noise_floor)
                        
                        z_score = 0.0
                        if len(gsr.history) > 20: 
                            mean = sum(gsr.history) / len(gsr.history)
                            variance = sum((x - mean) ** 2 for x in gsr.history) / len(gsr.history)
                            std_dev = math.sqrt(variance)
                            
                            if std_dev > 0.1: 
                                z_score = (tonic_val - mean) / std_dev
                                
                        broadcast_osc("/gsr/derivative", glitch_intensity)
                        broadcast_osc("/gsr/zscore", z_score)
                        
                    broadcast_osc("/gsr/filtered", filtered_val)
                    broadcast_osc("/gsr/tonic", tonic_val)
                    broadcast_osc("/gsr/phasic", phasic_val)
                
                # --- SINGLE TRIGGERS PASS-THROUGH ---
                else:
                    broadcast_osc(osc_address, osc_values[0] if len(osc_values) == 1 else osc_values)

except KeyboardInterrupt:
    print("\nClosing connection...")
    ser.close()
    print("Bridge terminated cleanly.")