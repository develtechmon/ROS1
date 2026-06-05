import asyncio
import sys
import pygame
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

# Speeds
SPEED_M_S = 2.0       
YAW_RATE_DEG_S = 30.0 

async def run():
    # Initialize Pygame and open a small interface window
    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("MAVSDK Flight Controller")
    
    drone = System()

    # SITL
    #await drone.connect(system_address="udp://:14540")

    # PIXHAWK
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("\n--- CONTROLS (Keep the Pygame Window Focused) ---")
    print("[ SPACE ] - Arm")
    print("[ t ]     - Takeoff")
    print("[ o ]     - Start Offboard (Required to move!)")
    print("[ w/s ]   - Forward / Backward")
    print("[ a/d ]   - Left / Right")
    print("[ u/j ]   - Up / Down")
    print("[ q/e ]   - Yaw Left / Yaw Right")
    print("[ l ]     - Land\n")

    offboard_started = False
    clock = pygame.time.Clock()

    while True:
        # 1. Handle Window Closing & One-off Key Presses
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
                elif event.key == pygame.K_t:
                    print("Taking off...")
                    await drone.action.takeoff()
                elif event.key == pygame.K_l:
                    print("Landing...")
                    if offboard_started:
                        await drone.offboard.stop()
                        offboard_started = False
                    await drone.action.land()
                elif event.key == pygame.K_o and not offboard_started:
                    print("Starting Offboard mode...")
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    try:
                        await drone.offboard.start()
                        offboard_started = True
                        print("Offboard active! You can fly now.")
                    except OffboardError as error:
                        print(f"Offboard failed: {error._result.result}")

        # 2. Track Continuous Multi-Key States
        keys = pygame.key.get_pressed()
        
        v_forward = 0.0
        v_right = 0.0
        v_down = 0.0
        v_yaw = 0.0

        if keys[pygame.K_w]: v_forward = SPEED_M_S
        elif keys[pygame.K_s]: v_forward = -SPEED_M_S

        if keys[pygame.K_d]: v_right = SPEED_M_S
        elif keys[pygame.K_a]: v_right = -SPEED_M_S

        if keys[pygame.K_j]: v_down = SPEED_M_S   # Down is positive
        elif keys[pygame.K_u]: v_down = -SPEED_M_S  # Up is negative

        if keys[pygame.K_e]: v_yaw = YAW_RATE_DEG_S
        elif keys[pygame.K_q]: v_yaw = -YAW_RATE_DEG_S

        # 3. Stream commands to drone if offboard is running
        if offboard_started:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v_forward, v_right, v_down, v_yaw)
            )

        # Draw a basic instruction on the screen so it isn't a blank black box
        screen.fill((30, 30, 30))
        pygame.display.flip()

        # Run at 20Hz to maintain the offboard connection stability
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(run())
