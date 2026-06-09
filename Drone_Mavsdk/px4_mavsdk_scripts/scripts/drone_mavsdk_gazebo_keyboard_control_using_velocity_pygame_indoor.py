import asyncio
import sys
import pygame
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

# Speeds - Lowered slightly for safer indoor testing
SPEED_M_S = 1.0       # 1 meter per second max
YAW_RATE_DEG_S = 20.0 # 20 degrees per second max

async def run():
    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("Indoor Optical Flow Controller")
    
    drone = System()
    # Change system_address to match your hardware connection:
    # - Serial telemetry radio: "serial:///dev/ttyUSB0:57600"
    # - Companion computer onboard: "udp://127.0.0.1:14540"
    
    # SITL
    await drone.connect(system_address="udp://:14540")

    # PIXHAWK
    #await drone.connect(system_address="serial:///dev/ttyACM0:2000000")

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to real drone!")
            break

    print("\n--- INDOOR CONTROLS (Keep Pygame Window Focused) ---")
    print("[ SPACE ] - Arm Drone")
    print("[ o ]     - Start Offboard (MUST do this to take off!)")
    print("[ w/s ]   - Forward / Backward")
    print("[ a/d ]   - Left / Right")
    print("[ u/j ]   - Up (Takeoff) / Down (Land)")
    print("[ q/e ]   - Yaw Left / Yaw Right")
    print("[ l ]     - Non-GPS Landing")
    print("[ ESC ]   - Emergency Stop Offboard (Hover)\n")

    offboard_started = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                if offboard_started:
                    await drone.offboard.stop()
                pygame.quit()
                sys.exit()
                
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Arming...")
                    await drone.action.arm()
                    
                elif event.key == pygame.K_o and not offboard_started:
                    print("Initiating Offboard Mode...")
                    # 1. Send zero-velocity setpoint first
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    try:
                        # 2. Start offboard mode
                        await drone.offboard.start()
                        offboard_started = True
                        print("Offboard Mode ACTIVE. Hold 'u' to take off manually!")
                    except OffboardError as error:
                        print(f"Offboard failed: {error._result.result}. Ensure Optical Flow has a good surface visual.")

                elif event.key == pygame.K_ESCAPE:
                    if offboard_started:
                        print("Emergency: Stopping Offboard (Drone will hover/land depending on failsafe)")
                        await drone.offboard.stop()
                        offboard_started = False
                        
                # --- NEW EMERGENCY NON-GPS LANDING ---
                elif event.key == pygame.K_l:
                    print("\rEMERGENCY LANDING INITIATED...")
                    
                    # 1. Stop Offboard mode cleanly so Python stops sending velocity overrides
                    if offboard_started:
                        try:
                            await drone.offboard.stop()
                            offboard_started = False
                        except Exception:
                            pass # Force proceed if offboard is already broken
                    
                    # 2. Force the flight controller to switch to its native non-GPS Land Mode
                    try:
                        await drone.action.land() 
                        print("Drone is landing vertically. Motors will disarm on touchdown.\r")
                    except Exception as e:
                        print(f"Action Land Rejected: {e}. Falling back to downward forced velocity.")
                        # Emergency fallback: If the flight controller rejects the high-level land command,
                        # re-engage a hard downward velocity override until manually stopped.
                        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.5, 0.0))

        keys = pygame.key.get_pressed()
        
        v_forward = 0.0
        v_right = 0.0
        v_down = 0.0
        v_yaw = 0.0

        if keys[pygame.K_w]: v_forward = SPEED_M_S
        elif keys[pygame.K_s]: v_forward = -SPEED_M_S

        if keys[pygame.K_d]: v_right = SPEED_M_S
        elif keys[pygame.K_a]: v_right = -SPEED_M_S

        # 'u' goes up (negative down), 'j' goes down (positive down)
        if keys[pygame.K_u]: v_down = -SPEED_M_S   
        elif keys[pygame.K_j]: v_down = SPEED_M_S  

        if keys[pygame.K_e]: v_yaw = YAW_RATE_DEG_S
        elif keys[pygame.K_q]: v_yaw = -YAW_RATE_DEG_S

        if offboard_started:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v_forward, v_right, v_down, v_yaw)
            )

        screen.fill((40, 50, 60))
        pygame.display.flip()
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(run())
