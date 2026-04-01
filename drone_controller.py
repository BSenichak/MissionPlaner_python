from typing import Optional  # noqa: F401
import collections

# Python 3.10+ compatibility: collections.MutableMapping moved to collections.abc
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math


class DroneController:
    """Drone flight controller: takeoff, navigate to target, hold altitude, land."""
    def __init__(self, connection_string: str) -> None:
        self.vehicle = connect(connection_string, wait_ready=True)

        # Target waypoint (hardcoded)
        self.target_lat = 50.443326
        self.target_lon = 30.448078
        self.target_alt = 200

        # XY PID gains (lateral control)
        self.P_GAIN = 6.0
        self.I_GAIN = 0.5
        self.MAX_TILT = 400  # max PWM deviation from neutral
        self.PWM_NEUTRAL = 1500

        # Altitude PI gains (vertical control)
        self.ALT_P_GAIN = 8.0
        self.ALT_I_GAIN = 0.3

        # Integral accumulators (reset on each new target)
        self.err_x_integral = 0.0
        self.err_y_integral = 0.0
        self.err_alt_integral = 0.0

    # ========================
    # UTILS
    # ========================
    def get_distance_metres(self, loc1: LocationGlobalRelative) -> float:
        """Haversine approximation: horizontal distance to target in metres."""
        d_lat = self.target_lat - loc1.lat
        d_lon = self.target_lon - loc1.lon
        return math.sqrt((d_lat**2) + (d_lon**2)) * 1.113195e5

    # --- Internal helpers ---

    def _wait_until_armable(self) -> None:
        """Block until vehicle reports is_armable=True."""
        print("Preparation for takeoff...")
        while not self.vehicle.is_armable:
            print(f"  Waiting... GPS: {self.vehicle.gps_0}")
            time.sleep(1)

    def _set_mode(self, mode: str) -> None:
        """Switch vehicle mode and wait until the mode is confirmed."""
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(0.5)

    # ========================
    # TAKEOFF
    # ========================
    def arm_and_takeoff(self) -> None:
        """Arm vehicle, switch to GUIDED, trigger simple_takeoff, wait for altitude."""
        self._wait_until_armable()

        print("GUIDED mode...")
        self._set_mode("GUIDED")

        print("Arming...")
        self.vehicle.armed = True

        start = time.time()
        while not self.vehicle.armed and (time.time() - start) < 30:
            print(f"  Arming... {self.vehicle.armed}")
            time.sleep(0.5)

        if not self.vehicle.armed:
            raise RuntimeError("Armed failed")

        print(f"Takeoff at {self.target_alt}m...")
        self.vehicle.simple_takeoff(self.target_alt)

        self._wait_for_altitude()

    def _wait_for_altitude(self) -> None:
        """Poll altitude until target is reached (95%) or 60s timeout."""
        start = time.time()

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"  Altitude: {alt}")

            if alt is not None and alt >= self.target_alt * 0.95:
                print("Height reached")
                return

            if time.time() - start > 60:
                print("HEIGHT TIMEOUT")
                return

            time.sleep(0.5)

    # ========================
    # CONTROL
    # ========================
    def _compute_xy_control(self, curr_lat: float, curr_lon: float, dist: float):
        """
        Lateral PID: compute pitch/roll PWM to reduce position error.
        err_y -> pitch (forward/back), err_x -> roll (left/right).
        Integral term only active when dist > 20m (near target no wind-up).
        """
        # Convert lat/lon error to metres
        err_y = (self.target_lat - curr_lat) * 111320
        err_x = (self.target_lon - curr_lon) * 111320 * math.cos(math.radians(curr_lat))

        # Wind-up guard: freeze integral far from target
        if dist > 20:
            self.err_y_integral += err_y
            self.err_x_integral += err_x

            self.err_y_integral = max(-20, min(20, self.err_y_integral))
            self.err_x_integral = max(-20, min(20, self.err_x_integral))
        else:
            self.err_y_integral *= 0.9
            self.err_x_integral *= 0.9

        pitch = self.PWM_NEUTRAL - (err_y * self.P_GAIN + self.err_y_integral * self.I_GAIN)
        roll = self.PWM_NEUTRAL + (err_x * self.P_GAIN + self.err_x_integral * self.I_GAIN)

        return pitch, roll

    def _compute_altitude_control(self, curr_alt: float):
        """
        Vertical PI: compute throttle PWM to hold target altitude.
        Integral only accumulates when error > 2m (near target no wind-up).
        """
        err_alt = self.target_alt - curr_alt

        if abs(err_alt) > 2:
            self.err_alt_integral += err_alt
            self.err_alt_integral = max(-30, min(30, self.err_alt_integral))
        else:
            self.err_alt_integral *= 0.8

        throttle = 1500 + int(self.ALT_P_GAIN * err_alt + self.ALT_I_GAIN * self.err_alt_integral)
        throttle = max(1400, min(1650, throttle))

        return throttle

    def _apply_limits(self, value: float) -> int:
        """Clamp PWM value to safe tilt range around neutral."""
        return int(max(
            self.PWM_NEUTRAL - self.MAX_TILT,
            min(self.PWM_NEUTRAL + self.MAX_TILT, value)
        ))

    # ========================
    # MAIN FLIGHT
    # ========================
    def fly(self) -> None:
        """
        Navigate to target: STABILIZE mode + RC overrides.
        Computed pitch/roll/throttle sent via channel overrides.
        Exit when distance to target < 30m, then switch to LAND.
        """
        print("STABILIZE mode + RC override")
        self._set_mode("STABILIZE")

        target_yaw_pwm = 1500  # no yaw correction

        try:
            while True:
                loc = self.vehicle.location.global_relative_frame
                dist = self.get_distance_metres(loc)

                if dist < 30:
                    print(f"Goal achieved: {dist:.1f}m")
                    break

                curr_lat = loc.lat or 0.0
                curr_lon = loc.lon or 0.0
                curr_alt = loc.alt or self.target_alt

                pitch, roll = self._compute_xy_control(curr_lat, curr_lon, dist)
                throttle = self._compute_altitude_control(curr_alt)

                pitch = self._apply_limits(pitch)
                roll = self._apply_limits(roll)

                # RC channel overrides: 1=roll, 2=pitch, 3=throttle, 4=yaw
                self.vehicle.channels.overrides = {
                    '1': roll,
                    '2': pitch,
                    '3': throttle,
                    '4': target_yaw_pwm
                }

                print(f"Dist: {dist:.1f} | R:{roll} P:{pitch} T:{throttle}")
                time.sleep(0.1)

        finally:
            self.vehicle.channels.overrides = {}
            self._set_mode("LAND")
            print("LAND")

    # ========================
    # RUN
    # ========================
    def run(self) -> None:
        """
        Full mission sequence:
        1. Arm + takeoff
        2. Fly to target
        3. Wait for landing
        4. Close connection
        """
        try:
            self.arm_and_takeoff()
            self.fly()

            while self.vehicle.armed:
                time.sleep(1)

            print("Mission is completed")

        finally:
            self.vehicle.close()