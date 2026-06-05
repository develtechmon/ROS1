import asyncio
import sys
import pygame
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

SPEED_M_S = 1.0
YAW_RATE_DEG_S = 20.0

# Shared state
velocity_cmd = [0.0, 0.0, 0.0, 0.0]  # [fwd, right, down, yaw]
offboard_active = False

async def setpoint_loop(drone):
    """Runs independently at strict 20Hz — never blocked by pygame"""
    global offboard_active
    while True:
        if offboard_active:
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(*velocity_cmd)
                )
            except Exception:
                pass
        await asyncio.sleep(0.05)

async def run():
    global velocity_cmd, offboard_active

    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("Indoor Optical Flow Controller")

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:2000000")

    print("Connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Launch setpoint loop as independent task
    asyncio.ensure_future(setpoint_loop(drone))

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Arming...")
                    await drone.action.arm()

                elif event.key == pygame.K_o and not offboard_active:
                    print("Starting offboard...")
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                    try:
                        await drone.offboard.start()
                        offboard_active = True
                        print("Offboard ACTIVE")
                    except OffboardError as e:
                        print(f"Offboard failed: {e}")

                elif event.key == pygame.K_ESCAPE:
                    print("Emergency stop...")
                    offboard_active = False
                    try:
                        await drone.offboard.stop()
                    except OffboardError:
                        # Expected in HITL no-GPS — fall back to land
                        await drone.action.land()

                elif event.key == pygame.K_l:
                    print("Landing...")
                    offboard_active = False
                    try:
                        await drone.offboard.stop()
                    except Exception:
                        pass
                    await drone.action.land()

        # Read held keys
        keys = pygame.key.get_pressed()
        velocity_cmd[0] = SPEED_M_S if keys[pygame.K_w] else (-SPEED_M_S if keys[pygame.K_s] else 0.0)
        velocity_cmd[1] = SPEED_M_S if keys[pygame.K_d] else (-SPEED_M_S if keys[pygame.K_a] else 0.0)
        velocity_cmd[2] = -SPEED_M_S if keys[pygame.K_u] else (SPEED_M_S if keys[pygame.K_j] else 0.0)
        velocity_cmd[3] = YAW_RATE_DEG_S if keys[pygame.K_e] else (-YAW_RATE_DEG_S if keys[pygame.K_q] else 0.0)

        screen.fill((40, 50, 60))
        pygame.display.flip()
        await asyncio.sleep(0.02)  # Event loop at 50Hz, setpoint loop independent

if __name__ == "__main__":
    asyncio.run(run())
