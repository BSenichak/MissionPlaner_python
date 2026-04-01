# MissionPlaner

## Startup

1. Configure MissionPlaner flight plan with MP_mission_config.waypoints file

2. Wind settings
```bash
python setup.py
```

3. Takeoff and navigation
```bash
python main.py
```

## Flight config (drone_controller.py)

```python
target_lat = 50.443326
target_lon = 30.448078
target_alt = 200  # метрів
```

## Dependency

```
dronekit
pymavlink
```
