import time
from dronekit import connect, VehicleMode

def run_gps_denied_flight():
    print("Initializing Durga's Domain: GPS-Denied Autonomous Flight")
    
    # 1. Connect without waiting for GPS Home Location (wait_ready=False is crucial here!)
    connection_string = 'COM3' 
    print(f"Connecting to Pixhawk on: {connection_string}")
    
    try:
        vehicle = connect(connection_string, baud=57600, wait_ready=False)
        vehicle.parameters.wait_ready(timeout=10) # Wait for basic params, ignore GPS
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # 2. Force ALT_HOLD Mode (Relies only on Barometer and IMU)
    print("Switching to ALT_HOLD mode...")
    vehicle.mode = VehicleMode("ALT_HOLD")
    while not vehicle.mode.name == 'ALT_HOLD':
        time.sleep(1)

    # 3. Force Arming
    print("Arming motors without GPS...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming... (Sensors Team: Ensure ARMING_CHECK is set to 0 in Mission Planner!)")
        time.sleep(1)

    print("MOTORS ARMED. Stand clear.")
    time.sleep(2) # Give motors a second to idle

    # 4. RC Override Takeoff (Simulating pushing the Throttle stick up)
    # Channel 3 is Throttle. 1500 is middle, 2000 is max.
    print("Applying 70% Throttle for takeoff...")
    vehicle.channels.overrides['3'] = 1700 
    
    # Climb for exactly 3 seconds
    time.sleep(3) 

    # 5. RC Override Hover (Simulating releasing the stick to the middle)
    print("Centering Throttle. Barometer holding altitude...")
    vehicle.channels.overrides['3'] = 1500 
    
    # Hover for 8 seconds
    time.sleep(8)

    # 6. Barometric Landing
    print("Initiating Barometric Landing Sequence...")
    # Releasing the RC overrides
    vehicle.channels.overrides = {} 
    
    # Switch to LAND mode. Pixhawk uses the barometer to descend safely and auto-disarm.
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print(" Descending...")
        time.sleep(1)
        
    print("Touchdown confirmed. Motors disarmed.")
    vehicle.close()
    print("GPS-Denied Mission Complete.")

if __name__ == "__main__":
    run_gps_denied_flight()