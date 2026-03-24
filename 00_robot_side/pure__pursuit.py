import asyncio
import math
import websockets
import json
import time
import traceback

from sensors.rtk_reader import RTKReader
from sensors.imu_reader import IMUReader

from navigation.waypoint import Waypoint, WaypointManager
from navigation.nav_engine import NavigationEngine, NavMode

# === IMU and RTK readers ===
imu_reader = IMUReader()
rtk_reader = RTKReader()
rtk_reader.start()

# Assuming TARGETS is a list of (lat, lon) tuples, e.g., TARGETS = [(lat1, lon1), (lat2, lon2), ...]
# If it's from a file, import accordingly. For now, define or import TARGETS here.
TARGETS = [(38.9073, -92.2683), (38.9074, -92.2684)]  # Example hardcoded targets; replace with actual data
print(TARGETS)

# Create waypoints using Waypoint class
WAYPOINT_RADIUS = 2.0  # meters (adjust as needed)
waypoints = [
    Waypoint(id=i, lat=lat, lon=lon, tolerance_m=WAYPOINT_RADIUS, max_speed=0.6)
    for i, (lat, lon) in enumerate(TARGETS)
]

# Generate CSV from waypoints
waypoints_csv = "id,lat,lon,tolerance_m,max_speed\n" + "\n".join(
    f"{wp.id},{wp.lat},{wp.lon},{wp.tolerance_m},{wp.max_speed}" for wp in waypoints
)

# Optional: Use WaypointManager to validate or manage waypoints locally
wp_mgr = WaypointManager()
num_loaded = wp_mgr.load_csv(waypoints_csv)
print(f"Loaded {num_loaded} waypoints into local manager")
# You can now access wp_mgr.current, wp_mgr.waypoints, etc., for debugging or additional logic

from navigation.geo_utils import haversine_distance, bearing_to_target, normalize_angle

# ======================
# Autonomous navigation task using NavigationEngine
# ======================
async def nav_task(ws, nav_engine):
    try:
        while True:
            # Get GPS data
            pos = rtk_reader.get_data()
            if pos is not None:
                lat, lon = pos["lat"], pos["lon"]
                precision = pos.get("precision", 0.0)

                if precision <= 2 and not math.isnan(precision):
                    # Send RTK data to nav_engine
                    rtk_data = {
                        "lat": lat,
                        "lon": lon,
                        "fix_quality": 4 if precision < 0.5 else (5 if precision < 1.0 else 2)
                    }
                    nav_engine.on_rtk(rtk_data)

            # Get IMU data
            yaw = imu_reader.get_yaw()
            yaw_accuracy = imu_reader.get_accuracy()

            if yaw is not None and yaw_accuracy is not None and yaw_accuracy >= 2:
                # Send IMU data to nav_engine
                imu_data = {
                    "compass": {
                        "bearing": yaw,
                        "accuracy": yaw_accuracy,
                        "calibrated": yaw_accuracy >= 2
                    },
                    "accel": {"x": 0.0, "y": 0.0}
                }
                nav_engine.on_imu(imu_data)

            await asyncio.sleep(0.05)  # IMU rate ~20Hz

    except Exception as e:
        print(f"[NAV TASK ERROR] {e}")
        traceback.print_exc()


 



# ======================
# Main entry
# ======================

#ping the webserver to automatically find the correct connection
#necessary due to ip changing (tailscale vs connection via router)
from pythonping import ping
def get_connection():
    uri1 = "100.87.161.11"
    uri2 = "192.168.0.100"
    try:
        response = ping(uri1, count=4, timeout=5)
        if response.success():
            print(f"[CONNECTION CHECK] WebSocket server {uri1} is reachable.")
            return uri1
        else:
            print(f"[CONNECTION CHECK] WebSocket server {uri1} is unreachable. Attempting connection to {uri2}")
            response = ping(uri2, count=4, timeout=5)
            if response.success():
                print(f"[CONNECTION CHECK] WebSocket server {uri2} is reachable.")
                return uri2
            else:
                print(f"[CONNECTION CHECK] WebSocket server {uri2} is unreachable.")
                return None
    except Exception as e:
        print(f"[CONNECTION CHECK ERROR] {e}")
        return None

async def main():
    while True:
        try:
            port = "8555"
            ws_uri = get_connection()
            if ws_uri == None:
                print("[WS URI] URI is none, connection failed")
                return 0
            
            ws_uri = f"ws://{ws_uri}:{port}"
            print(f"[URI CHECK] {ws_uri}")

            print(f"[MAIN] Connecting to {ws_uri}...")
            async with websockets.connect(ws_uri) as ws:
                print("[MAIN] Connected. Starting tasks...")

                # Define send_velocity function
                def send_velocity(linear, angular):
                    command = f"v{linear:.2f}w{angular:.2f}"
                    asyncio.create_task(ws.send(command))
                    print(f"[NAV] Sending: {command}")

                # Define broadcast function
                async def broadcast(status):
                    try:
                        await ws.send(f"status:{json.dumps(status)}")
                    except:
                        pass

                # Create NavigationEngine
                loop = asyncio.get_event_loop()
                nav_engine = NavigationEngine(
                    send_velocity_fn=send_velocity,
                    broadcast_fn=broadcast,
                    loop=loop
                )

                # Load waypoints
                num_wp = nav_engine.load_waypoints(waypoints_csv)
                print(f"[NAV] Loaded {num_wp} waypoints")

                # Set to Pure Pursuit mode
                nav_engine.set_nav_mode(NavMode.PURE_PURSUIT)

                # Start navigation
                nav_engine.start(force=True)

                await asyncio.gather(
                    nav_task(ws, nav_engine),
                )
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"[WEBSOCKET ERROR] Connection lost: {e}. Reconnecting in 3s...")
            traceback.print_exc()
            await asyncio.sleep(3)

        except Exception as e:
            print(f"[CONTROLLER] Unexpected error: {e}")
            traceback.print_exc()
            await asyncio.sleep(3)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"[CONTROLLER] Error: {e}\n{traceback.format_exc()}")

