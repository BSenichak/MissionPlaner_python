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
        print(f"  Очікування... GPS: {vehicle.gps_0}")

    print("Встановлюємо GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)

    print("Arming...")
    vehicle.armed = True

    # Чекаємо справжнього armed з таймаутом
    start = time.time()
    while not vehicle.armed and (time.time() - start) < 30:
        time.sleep(0.5)
        print(f"  Arming... attempts: {vehicle.armed}")

    if not vehicle.armed:
        print("КРИТИЧНА ПОМИЛКА: Не вдалося armed!")
        return  # Не продовжуємо якщо не armed

    print(f"Зліт на {target_altitude}м...")
    vehicle.simple_takeoff(target_altitude)

    # Чекаємо досягнення висоти з таймаутом
    start = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Поточна висота: {alt}")
        if alt is not None and alt >= target_altitude * 0.95:
            print("Висоти досягнуто.")
            break
        if (time.time() - start) > 60:
            print(f"ТАЙМАУТ: Не вдалося досягти висоти за 60с")
            break
        time.sleep(0.5)

def fly_with_rc_override():
    print("Перехід в STABILIZE. Початок керування через RC Override.")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)  # Чекаємо переключення режиму

    # Фіксуємо Yaw на початку польоту (утримуємо одне значення весь політ)
    target_yaw_pwm = 1500

    # І-складові для горизонтального руху
    err_y_integral = 0.0
    err_x_integral = 0.0

    # І-складова для висоти
    err_alt_integral = 0.0
    ALT_I_GAIN = 0.3  # Коефіцієнт для інтегральної складової висоти
    ALT_P_GAIN = 8.0  # Коефіцієнт для пропорційної складової висоти

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

            # Утримання висоти (Канал 3) - PI-регулятор
            curr_alt = curr_loc.alt if curr_loc.alt is not None else TARGET_ALT
            err_alt = TARGET_ALT - curr_alt

            # Накопичуємо І-похибку висоти (анти-windup)
            if abs(err_alt) > 2.0:  # Якщо відхилення більше 2м
                err_alt_integral += err_alt
                err_alt_integral = max(-30, min(30, err_alt_integral))
            else:
                # Скидаємо інтеграл коли близько до цільової висоти
                err_alt_integral *= 0.8

            # PI-регулятор висоти: 1500 + P*err + I*integral
            throttle_pwm = 1500 + int(ALT_P_GAIN * err_alt + ALT_I_GAIN * err_alt_integral)
            throttle_pwm = max(1400, min(1650, throttle_pwm))  # Обмеження

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

            print(f"Дистанція: {dist:.1f}м | Roll: {int(roll_pwm)} | Pitch: {int(pitch_pwm)} | Throttle: {int(throttle_pwm)}")
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
