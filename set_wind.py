from pymavlink import mavutil
import time

m = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
time.sleep(2)

def set_param(name, value, timeout=3):
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode(), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    start = time.time()
    while time.time() - start < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip() == name:
            print(f"{name} = {msg.param_value}")
            return True
    print(f"{name} — не знайдено або timeout")
    return False

set_param('SIM_WIND_SPD', 4.0)  # Спочатку без вітру для тесту
set_param('SIM_WIND_DIR', 30.0)
set_param('SIM_WIND_TURB', 2.0)
set_param('SIM_WIND_TURB_FREQ', 0.2)

m.close()
print("Готово!")
