#!/usr/bin/env python3
import sys
import time
import signal
from pymavlink import mavutil

# Connection settings
DEVICE = '/dev/ttyACM0'  # USB connection to ArduPilot
BAUD_RATE = 115200       # Baudrate for communication

# Global counters for statistics
stats = {
    'messages_received': 0,
    'bad_data_count': 0,
    'heartbeats': 0,
    'last_update_time': time.time()
}

def connect_to_autopilot():
    """
    Connect to the ArduPilot using pymavlink
    Returns the mavlink connection object
    """
    print(f"Connecting to ArduPilot on {DEVICE}...")
    
    # Create connection object
    try:
        # Correct format for serial connection
        master = mavutil.mavlink_connection(DEVICE, baud=BAUD_RATE)
        
        # Wait for a heartbeat to establish connection
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"Heartbeat detected from system {master.target_system} component {master.target_component}")
        
        return master
    
    except Exception as e:
        print(f"Error connecting to ArduPilot: {e}")
        sys.exit(1)

def request_data_streams(master):
    """
    Request data streams from ArduPilot
    """
    print("Requesting data streams...")
    
    # Request all data streams at 4Hz
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Rate in Hz
        1   # Start
    )
    
    # Request parameters
    master.mav.param_request_list_send(
        master.target_system, 
        master.target_component
    )
    
    print("Data streams requested")
    return True

def print_vehicle_state(msg):
    """
    Print readable vehicle state from heartbeat message
    """
    if msg.get_type() == 'HEARTBEAT':
        mode = mavutil.mode_string_v10(msg)
        if hasattr(msg, 'system_status'):
            status = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
            stats['heartbeats'] += 1
            
            # Only print mode info when it changes
            if not hasattr(print_vehicle_state, 'last_mode') or \
               print_vehicle_state.last_mode != mode:
                print(f"MODE: {mode}, STATUS: {status}")
                print_vehicle_state.last_mode = mode

def print_parameter(msg):
    """
    Print parameter information in a readable format
    """
    if msg.get_type() == 'PARAM_VALUE':
        param_id = msg.param_id
        param_value = msg.param_value
        param_type = msg.param_type
        
        # Filter out common parameters that might be useful to see
        important_prefixes = ['BATT_', 'GPS_', 'COMPASS_', 'ARMING_', 'FS_', 'PILOT_']
        
        if any(param_id.startswith(prefix) for prefix in important_prefixes):
            print(f"PARAM: {param_id} = {param_value}")

def print_statustext(msg):
    """
    Print status text messages
    """
    if msg.get_type() == 'STATUSTEXT':
        severity = msg.severity
        text = msg.text
        print(f"STATUS[{severity}]: {text}")

def print_stats():
    """
    Print connection statistics
    """
    now = time.time()
    runtime = now - stats['last_update_time']
    
    if runtime >= 5.0:  # Update every 5 seconds
        bad_data_percent = 0
        if stats['messages_received'] > 0:
            bad_data_percent = (stats['bad_data_count'] / stats['messages_received']) * 100
            
        print(f"\n--- CONNECTION STATS ---")
        print(f"Messages: {stats['messages_received']}, Bad data: {stats['bad_data_count']} ({bad_data_percent:.1f}%)")
        print(f"Heartbeats: {stats['heartbeats']}")
        print(f"-------------------------\n")
        
        # Reset counters
        stats['messages_received'] = 0
        stats['bad_data_count'] = 0
        stats['heartbeats'] = 0
        stats['last_update_time'] = now

def signal_handler(sig, frame):
    """
    Handle Ctrl+C gracefully
    """
    print("\nExiting...")
    sys.exit(0)

def main():
    # Set up signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    
    # Connect to ArduPilot
    master = connect_to_autopilot()
    
    # Request data streams
    request_data_streams(master)
    
    print("Successfully connected to ArduPilot!")
    print("Monitoring system (press Ctrl+C to exit)...")
    
    # Main loop - monitor messages
    try:
        while True:
            # Check for messages
            msg = master.recv_match(blocking=True, timeout=0.5)
            
            if msg:
                msg_type = msg.get_type()
                stats['messages_received'] += 1
                
                if msg_type == 'BAD_DATA':
                    stats['bad_data_count'] += 1
                    # Don't print BAD_DATA, just count it
                    continue
                
                # Process specific message types
                print_vehicle_state(msg)
                print_parameter(msg)
                print_statustext(msg)
                
                # Print other interesting messages
                if msg_type not in ['HEARTBEAT', 'PARAM_VALUE', 'STATUSTEXT', 
                                   'RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3',
                                   'ATTITUDE', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW',
                                   'GLOBAL_POSITION_INT', 'SYS_STATUS',
                                   'VFR_HUD', 'POWER_STATUS']:
                    print(f"MSG: {msg_type}")
            
            # Print stats periodically
            print_stats()
            
            time.sleep(0.01)  # Small delay to reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Close the connection
        if master:
            print("Closing connection to ArduPilot")
            master.close()

if __name__ == "__main__":
    main() 