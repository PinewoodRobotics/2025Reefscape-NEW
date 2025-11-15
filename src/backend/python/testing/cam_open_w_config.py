import json
import sys
import os
from pathlib import Path

# Add src/ directory to Python path so backend imports work
script_dir = Path(__file__).parent
src_dir = script_dir.parent.parent  # src/backend/python -> src/
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import requests
from backend.python.common.config import get_config_raw
from backend.python.common.util.system import ProcessType, load_basic_system_config

config_base64 = get_config_raw()
watchdog_port = 5005
host = "localhost"

response = requests.post(
    f"http://{host}:{watchdog_port}/set/config",
    json={"config": config_base64},
)

print(f"Lidar 3D Setting Config Output: {response.json()}")

stop_response = requests.post(
    f"http://{host}:{watchdog_port}/stop/process",
    json={"process_types": []},
)

print(f"Lidar 3D Stopping Process Output: {stop_response.json()}")

response = requests.post(
    f"http://{host}:{watchdog_port}/start/process",
    json={
        "process_types": [
            "pathfinding",
        ]
    },
)

print(f"Pos Extrapolator Starting Process Output: {response.json()}")
