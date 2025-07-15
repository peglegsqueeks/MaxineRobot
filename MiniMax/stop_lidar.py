from pyrplidar import PyRPlidar

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=5)
lidar.stop()
lidar.set_motor_pwm(0)
lidar.disconnect()