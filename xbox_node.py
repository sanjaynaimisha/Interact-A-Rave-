"""
================================================================================
INTERACT-A-RAVE: STANDALONE MULTI-XBOX DSP NODE
================================================================================
Role: Detects Xbox gamepads (USB or Bluetooth) dynamically, assigns Player IDs, 
normalizes axis data (0.0 to 1.0), and broadcasts formatted OSC.

Usage (Local):  python3 xbox_node.py
Usage (Remote): python3 xbox_node.py --ip 192.168.2.1 --ip_port 8000 --ip2 192.168.2.2 --ip2_port 8001
================================================================================
"""

import time
import sys
import os
import argparse

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from pythonosc.udp_client import SimpleUDPClient

parser = argparse.ArgumentParser(description="Standalone Xbox DSP Node")
parser.add_argument("--port", default="/dev/tty.usbmodem11", help="Myo Dongle Port")
parser.add_argument("--ip", default="127.0.0.1", help="Target IP for OSC (Default: Localhost)")
parser.add_argument("--ip_port", type=int, default=8000, help="Port for OSC output (Default: 8000)")

parser.add_argument("--ip2", default="127.0.0.1", help="Second Target IP for remote")
parser.add_argument("--ip2_port", type=int, default=8001, help="Port for the second IP")


args = parser.parse_args()

a_client = SimpleUDPClient(args.ip, args.ip_port)
b_client = SimpleUDPClient(args.ip2, args.ip2_port)

def broadcast_osc(address, payload):
    """Sends data simultaneously to TouchDesigner, Max, and the Remote IP"""
    a_client.send_message(address, payload)
    b_client.send_message(address, payload)

def map_range(value, in_min, in_max, out_min, out_max):
    """Standard linear interpolation mapping"""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Standardized SDL2 mapping for Xbox controllers on macOS/Pygame-CE
BUTTON_MAP = {
    0: "a", 1: "b", 2: "x", 3: "y", 4: "view", 5: "xbox", 6: "menu", 
    7: "l3", 8: "r3", 9: "lb", 10: "rb", 11: "up", 12: "down", 
    13: "left", 14: "right", 15: "share"
}

AXIS_MAP = {
    0: "lx", 1: "ly", 2: "rx", 3: "ry", 4: "lt", 5: "rt"
}

def run_xbox_node():
    print("==========================================")
    print("MULTI-XBOX DSP NODE: INITIALIZING")
    print(f"Targeting OSC outputs at {args.ip}:{args.ip_port} and {args.ip2}:{args.ip2_port}")
    print("==========================================")

    pygame.init()
    pygame.joystick.init()

    # Dictionary to track controllers by their unique Pygame instance ID
    active_joysticks = {}
    player_assignment_counter = 1

    print("\n>>> NODE ACTIVE: Waiting for Gamepads (USB or Bluetooth) <<<")

    try:
        while True:
            # We must pump the event queue to detect hot-plugging and button presses
            for event in pygame.event.get():
                
                # --- HOT-PLUG: CONTROLLER CONNECTED ---
                if event.type == pygame.JOYDEVICEADDED:
                    joy = pygame.joystick.Joystick(event.device_index)
                    joy.init()
                    instance_id = joy.get_instance_id()
                    
                    active_joysticks[instance_id] = {
                        "joy": joy,
                        "player_id": player_assignment_counter,
                        "prev_buttons": {btn: 0 for btn in range(joy.get_numbuttons())},
                        "prev_hat": (0, 0)
                    }
                    print(f"[CONNECTED] Player {player_assignment_counter} joined via {joy.get_name()} (Wired/BT)")
                    player_assignment_counter += 1

                # --- HOT-PLUG: CONTROLLER DISCONNECTED ---
                elif event.type == pygame.JOYDEVICEREMOVED:
                    instance_id = event.instance_id
                    if instance_id in active_joysticks:
                        p_id = active_joysticks[instance_id]["player_id"]
                        print(f"[DISCONNECTED] Player {p_id} connection lost.")
                        del active_joysticks[instance_id]

            # --- DATA ROUTING LOOP ---
            for instance_id, data in active_joysticks.items():
                joy = data["joy"]
                player_id = data["player_id"]
                
                # 1. ROUTE AXES (Continuous with Math Normalization)
                for i in range(joy.get_numaxes()):
                    raw_val = joy.get_axis(i)
                    axis_name = AXIS_MAP.get(i, f"axis_{i}")
                    
                    norm_val = map_range(raw_val, -1.0, 1.0, 0.0, 1.0)
                    
                    broadcast_osc(f"/xbox/{player_id}/axis/{axis_name}", norm_val)

                # 2. ROUTE BUTTONS (Discrete)
                for i in range(joy.get_numbuttons()):
                    btn_val = joy.get_button(i)
                    if btn_val != data["prev_buttons"][i]:
                        btn_name = BUTTON_MAP.get(i, f"btn_{i}")
                        broadcast_osc(f"/xbox/{player_id}/button/{btn_name}", btn_val)
                        data["prev_buttons"][i] = btn_val

                # 3. ROUTE D-PAD (Hats)
                if joy.get_numhats() > 0:
                    hat_val = joy.get_hat(0)
                    if hat_val != data["prev_hat"]:
                        broadcast_osc(f"/xbox/{player_id}/hat", hat_val)
                        data["prev_hat"] = hat_val

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[STOP] Xbox DSP Node shutting down safely.")
    finally:
        pygame.quit()

if __name__ == '__main__':
    run_xbox_node()
