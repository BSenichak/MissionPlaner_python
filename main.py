import collections

# Виправлення сумісності для Python 3.10+
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# --- КОНФІГУРАЦІЯ ТА КООРДИНАТИ ---
# Точка А (Старт): 50.450739, 30.461242 
# Точка Б (Ціль): 50.443326, 30.448078 
TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 200 # Висота: 200 м 

# Налаштування П-регулятора
P_GAIN = 6.0       # Збільшили для кращої реакції
I_GAIN = 0.5       # Для компенсації сталого зміщення (вітер)
MAX_TILT = 400     # Максимальний нахил
PWM_NEUTRAL = 1500 # Нейтральне значення каналів

# Використовуємо порт SERIAL1 (5762), оскільки 5760 вже зайнятий Mission Planner
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

def get_distance_metres(loc1, lat2, lon2):
    dlat = lat2 - loc1.lat
    dlong = lon2 - loc1.lon
    return math.sqrt((dlat**2) + (dlong**2)) * 1.113195e5

def arm_and_takeoff(target_altitude):
    print("Підготовка до зльоту...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Встановлюємо GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(3)  # Чекаємо переключення режиму

    print("Arming...")
    vehicle.armed = True

    attempts = 0
    while not vehicle.armed and attempts < 30:
        time.sleep(1)
        vehicle.armed = True
        attempts += 1

    if not vehicle.armed:
        print("ПОПЕРЕДЖЕННЯ: Не вдалося armed, продовжуємо...")

    print(f"Зліт на {target_altitude}м...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            print("Висоти досягнуто.")
            break
        time.sleep(1)

def fly_with_rc_override():
    print("Перехід в STABILIZE. Початок керування через RC Override.")
    vehicle.mode = VehicleMode("STABILIZE")

    # Фіксуємо Yaw на початку польоту (утримуємо одне значення весь політ)
    target_yaw_pwm = 1500

    # І-складова
    err_y_integral = 0.0
    err_x_integral = 0.0

    try:
        while True:
            curr_loc = vehicle.location.global_relative_frame
            dist = get_distance_metres(curr_loc, TARGET_LAT, TARGET_LON)

            if dist < 30.0:  # Тестовий поріг
                print(f"Точка Б досягнута (dist={dist:.1f}м). Припинення override.")
                break

            # Розрахунок похибки в метрах
            curr_lat = curr_loc.lat if curr_loc.lat is not None else 0.0
            curr_lon = curr_loc.lon if curr_loc.lon is not None else 0.0
            err_y = (TARGET_LAT - curr_lat) * 111320
            err_x = (TARGET_LON - curr_lon) * 111320 * math.cos(math.radians(curr_lat))

            # Накопичуємо І-похибку (анти-windup при dist < 20м)
            if dist > 20.0:
                err_y_integral += err_y
                err_x_integral += err_x
                err_y_integral = max(-20, min(20, err_y_integral))
                err_x_integral = max(-20, min(20, err_x_integral))
            else:
                # Скидаємо інтеграл коли близько
                err_y_integral *= 0.9
                err_x_integral *= 0.9

            # Розрахунок PWM (ПІ-регулятор)
            pitch_pwm = PWM_NEUTRAL - (err_y * P_GAIN + err_y_integral * I_GAIN)
            roll_pwm = PWM_NEUTRAL + (err_x * P_GAIN + err_x_integral * I_GAIN)

            # Утримання висоти (Канал 3) - плавна регуляція
            curr_alt = curr_loc.alt if curr_loc.alt is not None else TARGET_ALT
            alt_error = TARGET_ALT - curr_alt
            throttle_pwm = 1550 + int(alt_error * 5)  # Пропорційне управління
            throttle_pwm = max(1450, min(1650, throttle_pwm))  # Обмеження

            # Обмеження значень
            pitch_pwm = max(PWM_NEUTRAL - MAX_TILT, min(PWM_NEUTRAL + MAX_TILT, pitch_pwm))
            roll_pwm = max(PWM_NEUTRAL - MAX_TILT, min(PWM_NEUTRAL + MAX_TILT, roll_pwm))


            # Застосування RC Override
            vehicle.channels.overrides = {
                '1': int(roll_pwm),
                '2': int(pitch_pwm),
                '3': int(throttle_pwm),
                '4': target_yaw_pwm
            }

            print(f"Дистанція: {dist:.1f}м | Roll: {int(roll_pwm)} | Pitch: {int(pitch_pwm)}")
            time.sleep(0.1)

    finally:
        # Очищаємо overrides
        vehicle.channels.overrides = {}

        # Встановлюємо LAND mode
        vehicle.mode = VehicleMode("LAND")
        print("Режим LAND встановлено")

# --- ОСНОВНИЙ ЦИКЛ ---
try:
    arm_and_takeoff(TARGET_ALT)
    fly_with_rc_override()
    # LAND mode встановлюється у finally блоці fly_with_rc_override()

    while vehicle.armed:
        time.sleep(1)
    print("Місія завершена успішно.")

finally:
    vehicle.close()
