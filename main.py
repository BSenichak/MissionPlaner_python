from drone_controller import DroneController

controller = DroneController("tcp:127.0.0.1:5762")
controller.run()