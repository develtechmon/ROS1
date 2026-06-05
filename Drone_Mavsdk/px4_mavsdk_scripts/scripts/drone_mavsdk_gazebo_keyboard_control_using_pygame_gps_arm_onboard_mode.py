import asyncio
import sys
import pygame
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ── SPEED SETTINGS ────────────────────────────────────────────────────────────
SPEED_M_S       = 2.0   # m/s horizontal
YAW_RATE_DEG_S  = 30.0  # deg/s yaw rate
VERT_SPEED_M_S  = 1.0   # m/s vertical

async def setpoint_loop(drone, velocity_cmd, active_flag):
    """
    Independent 50Hz setpoint sender.
    Runs as a background task — never blocked by pygame event processing.
    """
    while True:
        if active_flag[0]:
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(*velocity_cmd)
                )
            except Exception:
                pass
        await asyncio.sleep(0.02)  # 50Hz


async def run():
    # ── PYGAME SETUP ──────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((400, 250))
    pygame.display.set_caption("MAVSDK HITL Keyboard Controller")

    # ── CONNECT ───────────────────────────────────────────────────────────────
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:2000000")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to Pixhawk!")
            break

    print("Waiting for position lock (GPS)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position lock OK — ready to fly\n")
            break

    print("─── CONTROLS ────────────────────────────────")
    print("  [ SPACE ] — Pre-stream + Start Offboard + Arm")
    print("  [ w/s ]   — Forward / Backward")
    print("  [ a/d ]   — Left / Right")
    print("  [ u/j ]   — Up / Down")
    print("  [ q/e ]   — Yaw Left / Yaw Right")
    print("  [ l ]     — Land")
    print("  [ ESC ]   — Stop Offboard")
    print("─────────────────────────────────────────────\n")

    # ── SHARED STATE ──────────────────────────────────────────────────────────
    # velocity_cmd: [fwd, right, down, yaw]
    velocity_cmd  = [0.0, 0.0, 0.0, 0.0]
    active_flag   = [False]   # mutable so setpoint_loop can read it
    offboard_started = False

    # ── LAUNCH INDEPENDENT SETPOINT TASK ──────────────────────────────────────
    asyncio.ensure_future(setpoint_loop(drone, velocity_cmd, active_flag))

    # ── MAIN EVENT LOOP ───────────────────────────────────────────────────────
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                active_flag[0] = False
                try:
                    await drone.offboard.stop()
                except Exception:
                    pass
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYDOWN:

                # ── ARM + OFFBOARD ────────────────────────────────────────────
                if event.key == pygame.K_SPACE and not offboard_started:
                    print("Pre-streaming setpoints for 1.5s...")

                    # Stream zero setpoints for 1.5s so PX4 accepts offboard
                    for _ in range(30):
                        await drone.offboard.set_velocity_body(
                            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                        )
                        await asyncio.sleep(0.05)

                    # Start offboard BEFORE arming
                    try:
                        await drone.offboard.start()
                        offboard_started = True
                        active_flag[0]   = True
                        print("Offboard ACTIVE")
                    except OffboardError as e:
                        print(f"Offboard start failed: {e}")
                        continue

                    # Arm AFTER offboard is confirmed active
                    await drone.action.arm()
                    print("Armed — hold [ u ] to climb\n")

                # ── LAND ──────────────────────────────────────────────────────
                elif event.key == pygame.K_l:
                    print("Landing...")
                    active_flag[0]   = False
                    offboard_started = False
                    try:
                        await drone.offboard.stop()
                    except Exception:
                        pass
                    await drone.action.land()

                # ── EMERGENCY STOP OFFBOARD ───────────────────────────────────
                elif event.key == pygame.K_ESCAPE:
                    print("Stopping offboard...")
                    active_flag[0]   = False
                    offboard_started = False
                    try:
                        await drone.offboard.stop()
                    except OffboardError:
                        # No GPS HOLD mode — fall back to land
                        print("HOLD denied — landing instead")
                        await drone.action.land()

        # ── READ HELD KEYS → UPDATE VELOCITY CMD ──────────────────────────────
        keys = pygame.key.get_pressed()

        velocity_cmd[0] = (
             SPEED_M_S if keys[pygame.K_w] else
            -SPEED_M_S if keys[pygame.K_s] else 0.0
        )
        velocity_cmd[1] = (
             SPEED_M_S if keys[pygame.K_d] else
            -SPEED_M_S if keys[pygame.K_a] else 0.0
        )
        velocity_cmd[2] = (
            -VERT_SPEED_M_S if keys[pygame.K_u] else   # up = negative NED
             VERT_SPEED_M_S if keys[pygame.K_j] else 0.0
        )
        velocity_cmd[3] = (
             YAW_RATE_DEG_S if keys[pygame.K_e] else
            -YAW_RATE_DEG_S if keys[pygame.K_q] else 0.0
        )

        # ── DRAW UI ───────────────────────────────────────────────────────────
        screen.fill((30, 30, 30))
        font = pygame.font.SysFont("monospace", 14)

        status_color = (0, 220, 0) if offboard_started else (200, 80, 80)
        status_text  = "OFFBOARD ACTIVE" if offboard_started else "OFFBOARD INACTIVE"
        screen.blit(font.render(status_text, True, status_color), (10, 10))

        if offboard_started:
            lines = [
                f"fwd:  {velocity_cmd[0]:+.1f} m/s",
                f"right:{velocity_cmd[1]:+.1f} m/s",
                f"down: {velocity_cmd[2]:+.1f} m/s",
                f"yaw:  {velocity_cmd[3]:+.1f} deg/s",
            ]
            for idx, line in enumerate(lines):
                screen.blit(font.render(line, True, (180, 180, 180)), (10, 40 + idx * 20))

        pygame.display.flip()
        await asyncio.sleep(0.02)


if __name__ == "__main__":
    asyncio.run(run())
