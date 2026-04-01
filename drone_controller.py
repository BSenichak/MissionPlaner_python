from typing import Optional
import collections

# Fix for Python 3.10+
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math


class DroneController:
    def __init__(self, connection_string: str) -> None:
        self.vehicle = connect(connection_string, wait_ready=True)

        # --- TARGET ---
        self.target_lat = 50.443326
        self.target_lon = 30.448078
        self.target_alt = 200

        # --- PID ---
        self.P_GAIN = 6.0
        self.I_GAIN = 0.5
        self.MAX_TILT = 400
        self.PWM_NEUTRAL = 1500

        # Altitude PI
        self.ALT_P_GAIN = 8.0
        self.ALT_I_GAIN = 0.3

        # Integrals
        self.err_x_integral = 0.0
        self.err_y_integral = 0.0
        self.err_alt_integral = 0.0

    # ========================
    # UTILS
    # ========================
    def get_distance_metres(self, loc1: LocationGlobalRelative) -> float:
        dlat = self.target_lat - loc1.lat
        dlong = self.target_lon - loc1.lon
        return math.sqrt((dlat**2) + (dlong**2)) * 1.113195e5

    def _wait_until_armable(self) -> None:
        print("Preparation for takeoff...")
        while not self.vehicle.is_armable:
            print(f"  Waiting... GPS: {self.vehicle.gps_0}")
            time.sleep(1)

    def _set_mode(self, mode: str) -> None:
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(0.5)

    # ========================
    # TAKEOFF
    # ========================
    def arm_and_takeoff(self) -> None:
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
        err_y = (self.target_lat - curr_lat) * 111320
        err_x = (self.target_lon - curr_lon) * 111320 * math.cos(math.radians(curr_lat))

        # Integral
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
        return int(max(
            self.PWM_NEUTRAL - self.MAX_TILT,
            min(self.PWM_NEUTRAL + self.MAX_TILT, value)
        ))

    # ========================
    # MAIN FLIGHT
    # ========================
    def fly(self) -> None:
        print("STABILIZE mode + RC override")
        self._set_mode("STABILIZE")

        target_yaw_pwm = 1500

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
        try:
            self.arm_and_takeoff()
            self.fly()

            while self.vehicle.armed:
                time.sleep(1)

            print("Mission is completed")

        finally:
            self.vehicle.close()