from typing import Any
from pymavlink import mavutil
from pymavlink.mavutil import mavlink_connection
import time


class MissionPlanerParametersSetter:
    """
    A utility class for setting MAVLink parameters (e.g., in ArduPilot SITL)
    via a TCP/UDP connection.

    This class establishes a MAVLink connection, waits for a heartbeat,
    and allows setting parameters with confirmation via PARAM_VALUE messages.

    Attributes:
        mav_util (mavutil.mavlink_connection): Active MAVLink connection instance.
    """

    def __init__(self, link: str) -> None:
        """
        Initializes MAVLink connection.

        Args:
            link (str): Connection string (e.g., "tcp:127.0.0.1:5762").

        Raises:
            Exception: If connection fails.
        """
        self.mav_util: mavutil.mavfile | Any = None
        try:
            self.mav_util = mavutil.mavlink_connection(link)
            print("Waiting for heartbeat...")
            self.mav_util.wait_heartbeat()
            print("Connected!")
        except Exception as error:
            print(f"Can't connect error: {error}")

    def set_param(self, name: str, value: float, timeout: float = 3) -> bool:
        """
        Sets a MAVLink parameter and waits for confirmation.

        Sends a PARAM_SET message and listens for a corresponding
        PARAM_VALUE response from the vehicle.

        Args:
            name (str): Parameter name (e.g., "SIM_WIND_SPD").
            value (float): Value to set.
            timeout (float, optional): Time to wait for confirmation in seconds. Defaults to 3.

        Returns:
            bool: True if parameter was successfully set and confirmed,
                  False if timeout occurred or connection is not available.

        Notes:
            - The parameter type is sent as MAV_PARAM_TYPE_REAL32.
            - The method relies on receiving a PARAM_VALUE message with matching param_id.
            - Works with SITL (ArduPilot) and real MAVLink-compatible autopilots.

        Example:
            >>> setter.set_param("SIM_WIND_SPD", 5.0)
            True
        """
        if (self.mav_util == None):
            return False
        self.mav_util.mav.param_set_send(
            self.mav_util.target_system,
            self.mav_util.target_component,
            name.encode(),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        start = time.time()
        while time.time() - start < timeout:
            msg = self.mav_util.recv_match(
                type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id.strip() == name:
                print(f"{name} = {msg.param_value}")
                return True
        print(f"{name} — not found or timeout")
        return False


param_setter = MissionPlanerParametersSetter("tcp:127.0.0.1:5762")

param_setter.set_param('SIM_WIND_SPD', 4.0)
param_setter.set_param('SIM_WIND_DIR', 30.0)
param_setter.set_param('SIM_WIND_TURB', 2.0)
param_setter.set_param('SIM_WIND_TURB_FREQ', 0.2)

print("Done!")
