from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
import asyncio

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for global position (home position lock)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Home position set, ready to fly.")
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)

asyncio.run(run())
