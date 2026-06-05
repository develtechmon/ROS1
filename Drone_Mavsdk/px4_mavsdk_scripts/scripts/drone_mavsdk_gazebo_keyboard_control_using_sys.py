import asyncio
import sys
import select
import tty
import termios
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

# Speeds
SPEED_M_S = 2.0       # Meters per second
YAW_RATE_DEG_S = 30.0 # Degrees per second
TIMEOUT = 0.15        # Seconds before drone stops moving if no key is held

async def run():
    drone = System()
    # Connect to the simulator (adjust if connecting to real hardware)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!\r")
            break

    print("\n--- CONTROLS (Keep this terminal focused) ---")
    print("[ SPACE ] - Arm")
    print("[ t ]     - Takeoff")
    print("[ o ]     - Start Offboard (Required to move!)")
    print("[ w/s ]   - Forward / Backward")
    print("[ a/d ]   - Left / Right")
    print("[ u/j ]   - Up / Down")
    print("[ q/e ]   - Yaw Left / Yaw Right")
    print("[ l ]     - Land")
    print("[ Ctrl+C] - Exit script\n")

    offboard_started = False
    
    # Save current terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to cbreak mode to read keys instantly without pressing Enter
        tty.setcbreak(sys.stdin.fileno())
        
        v_forward = v_right = v_down = v_yaw = 0.0
        last_key_time = time.time()
        
        while True:
            # 1. Read keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                last_key_time = time.time()
                
                # --- One-off Actions ---
                if key == ' ':
                    print("\rArming...                        ")
                    await drone.action.arm()
                elif key == 't':
                    print("\rTaking off...                    ")
                    await drone.action.takeoff()
                elif key == 'l':
                    print("\rLanding...                       ")
                    if offboard_started:
                        await drone.offboard.stop()
                        offboard_started = False
                    await drone.action.land()
                elif key == 'o' and not offboard_started:
                    print("\rStarting Offboard mode...        ")
                    # Must send a setpoint before starting offboard
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    try:
                        await drone.offboard.start()
                        offboard_started = True
                        print("\rOffboard mode active. You can fly now.")
                    except OffboardError as error:
                        print(f"\rOffboard mode failed: {error._result.result}")
                
                # Exit
                elif key == '\x03': # This is Ctrl+C
                    print("\rExiting...                       ")
                    break
                
                # --- Movement Controls ---
                elif key == 'w': v_forward = SPEED_M_S
                elif key == 's': v_forward = -SPEED_M_S
                elif key == 'd': v_right = SPEED_M_S
                elif key == 'a': v_right = -SPEED_M_S
                elif key == 'j': v_down = SPEED_M_S   # Down is positive
                elif key == 'u': v_down = -SPEED_M_S  # Up is negative
                elif key == 'e': v_yaw = YAW_RATE_DEG_S
                elif key == 'q': v_yaw = -YAW_RATE_DEG_S
            
            # 2. Key Release Check: If no key pressed recently, hover in place
            if time.time() - last_key_time > TIMEOUT:
                v_forward = v_right = v_down = v_yaw = 0.0

            # 3. Send continuous movement commands
            if offboard_started:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(v_forward, v_right, v_down, v_yaw)
                )

            # Run loop at 20Hz
            await asyncio.sleep(0.05)
            
    finally:
        # Always restore the terminal settings back to normal when exiting
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    asyncio.run(run())
