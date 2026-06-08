import asyncio
import sys
import argparse
import pygame
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

# Speeds
SPEED_M_S = 0.5
YAW_RATE_DEG_S = 20.0
POS_STEP_M = 0.05
ALT_STEP_M = 0.05

async def run(system_address):
    pygame.init()
    screen = pygame.display.set_mode((480, 240))
    pygame.display.set_caption("Indoor Controller — Velocity + Position Hold")

    drone = System()
    await drone.connect(system_address=system_address)

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to real drone!")
            break

    print("\n--- INDOOR CONTROLS ---")
    print("[ SPACE ] - Arm Drone")
    print("[ o ]     - Auto-takeoff to 1m + position hold")
    print("[ p ]     - Re-snap position hold to current position")
    print("[ w/s ]   - Forward / Backward")
    print("[ a/d ]   - Left / Right")
    print("[ u/j ]   - Up / Down")
    print("[ q/e ]   - Yaw Left / Yaw Right")
    print("[ l ]     - Land")
    print("[ ESC ]   - Emergency Stop Offboard\n")

    offboard_started = False
    position_hold    = False

    pos = {'n': 0.0, 'e': 0.0, 'd': 0.0}

    async def watch_pos():
        async for p in drone.telemetry.position_velocity_ned():
            pos['n'] = p.position.north_m
            pos['e'] = p.position.east_m
            pos['d'] = p.position.down_m

    asyncio.ensure_future(watch_pos())
    await asyncio.sleep(0.3)

    target_n   = 0.0
    target_e   = 0.0
    target_d   = 0.0
    target_yaw = 0.0

    font = pygame.font.SysFont("monospace", 12)

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

                # START OFFBOARD in velocity mode — exactly your original working code
                elif event.key == pygame.K_o and not offboard_started:
                    print("Auto-takeoff to 1m...")
                    target_n   = pos['n']
                    target_e   = pos['e']
                    target_d   = -1.0
                    target_yaw = 0.0
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(target_n, target_e, target_d, target_yaw)
                    )
                    try:
                        await drone.offboard.start()
                        offboard_started = True
                        position_hold    = True
                        print("Offboard ACTIVE — climbing to 1.0m and holding position.")
                    except OffboardError as error:
                        print(f"Offboard failed: {error._result.result}. Ensure Optical Flow has a good surface visual.")

                # SWITCH TO POSITION HOLD — press p after you have taken off
                elif event.key == pygame.K_p and offboard_started:
                    target_n   = pos['n']
                    target_e   = pos['e']
                    target_d   = pos['d']
                    target_yaw = 0.0
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(target_n, target_e, target_d, target_yaw)
                    )
                    position_hold = True
                    print(f"Switched to POSITION HOLD at "
                          f"N={target_n:.2f} E={target_e:.2f} Alt={-target_d:.2f}m")

                elif event.key == pygame.K_ESCAPE:
                    if offboard_started:
                        print("Emergency: Stopping Offboard")
                        await drone.offboard.stop()
                        offboard_started = False
                        position_hold    = False

                elif event.key == pygame.K_l:
                    print("Landing...")
                    if offboard_started:
                        try:
                            await drone.offboard.stop()
                            offboard_started = False
                            position_hold    = False
                        except Exception:
                            pass
                    try:
                        await drone.action.land()
                        print("Drone is landing vertically.")
                    except Exception as e:
                        print(f"Action Land Rejected: {e}. Falling back to downward velocity.")
                        await drone.offboard.set_velocity_body(
                            VelocityBodyYawspeed(0.0, 0.0, 0.5, 0.0)
                        )

        keys = pygame.key.get_pressed()

        if offboard_started:

            if position_hold:
                # POSITION HOLD — nudge target with keys, PX4 holds between presses
                if keys[pygame.K_w]: target_n += POS_STEP_M
                if keys[pygame.K_s]: target_n -= POS_STEP_M
                if keys[pygame.K_d]: target_e += POS_STEP_M
                if keys[pygame.K_a]: target_e -= POS_STEP_M
                if keys[pygame.K_u]: target_d -= ALT_STEP_M
                if keys[pygame.K_j]: target_d += ALT_STEP_M
                if keys[pygame.K_e]: target_yaw = (target_yaw + 5.0) % 360
                if keys[pygame.K_q]: target_yaw = (target_yaw - 5.0) % 360

                # Must send every loop — PX4 exits offboard if silent >500ms
                await drone.offboard.set_position_ned(
                    PositionNedYaw(target_n, target_e, target_d, target_yaw)
                )

            else:
                # VELOCITY MODE — your original working code unchanged
                v_forward = 0.0
                v_right   = 0.0
                v_down    = 0.0
                v_yaw     = 0.0

                if keys[pygame.K_w]:   v_forward =  SPEED_M_S
                elif keys[pygame.K_s]: v_forward = -SPEED_M_S
                if keys[pygame.K_d]:   v_right =   SPEED_M_S
                elif keys[pygame.K_a]: v_right =  -SPEED_M_S
                if keys[pygame.K_u]:   v_down = -SPEED_M_S
                elif keys[pygame.K_j]: v_down =  SPEED_M_S
                if keys[pygame.K_e]:   v_yaw =  YAW_RATE_DEG_S
                elif keys[pygame.K_q]: v_yaw = -YAW_RATE_DEG_S

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(v_forward, v_right, v_down, v_yaw)
                )

        # Display
        screen.fill((40, 50, 60))
        mode_str = "POSITION HOLD" if position_hold else "VELOCITY"
        mode_col = (100, 220, 100) if position_hold else (220, 180, 60)
        ob_col   = (100, 220, 100) if offboard_started else (200, 80, 80)
        ob_str   = "OFFBOARD ON" if offboard_started else "OFFBOARD OFF"

        rows = [
            (f"{ob_str}  |  Mode: {mode_str}", ob_col if not offboard_started else mode_col),
            (f"Pos: N={pos['n']:+.2f}  E={pos['e']:+.2f}  Alt={-pos['d']:.3f}m", (180, 210, 255)),
            (f"Tgt: N={target_n:+.2f}  E={target_e:+.2f}  D={target_d:+.2f}", (150, 190, 230)),
            ("", (130,130,130)),
            ("SPACE=Arm  o=Takeoff1m  p=Resnap  l=Land  ESC=Stop", (120,120,120)),
            ("w/s=Fwd/Bk  a/d=L/R  u/j=Up/Dn  q/e=Yaw", (120,120,120)),
        ]
        for i, (txt, col) in enumerate(rows):
            screen.blit(pygame.font.SysFont("monospace",12).render(txt, True, col), (8, 10+i*22))

        pygame.display.flip()
        await asyncio.sleep(0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', type=str, default='udp://:14550')
    args = parser.parse_args()
    asyncio.run(run(args.address))
