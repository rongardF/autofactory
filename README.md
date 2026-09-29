# Building

Navigate into devcontainer workspace root and run:

bash
```
colcon build
```

# Launching

Run following command to launch the station launch system in simulation without Gazebo GUI:

bash
```
ros2 launch station station.launch.py simulated:=true gazebo_gui:=false model:=ur10e
```

Run following command to launch the station launch system with real hardware (no Gazebo GUI):

bash
```
ros2 launch station station.launch.py simulated:=false ip_address:=<robot-ip-address> model:=ur10
```

# View camera image

To view camera image (real or simulated) run:

bash
```
ros2 run rqt_image_view rqt_image_view /cameras/basler_camera/image_raw
```

# Laser cross sensor triggers

To read the laser cross sensor outputs (real or simulated):

bash
```
ros2 topic echo /laser_sensors/captron_orl2/x_axis_triggered
ros2 topic echo /laser_sensors/captron_orl2/y_axis_triggered
```

# Example scripts


Following commands must be executed to activate the '/movement_controller' lifecycle node before any movment can be performed:

bash
```
ros2 lifecycle set /movement_controller configure
ros2 lifecycle set /movement_controller activate
```

There are example Python scripts located in `/examples` directory to illustrate how to perform various type of movments. To run them execute in terminal in devcontainer root folder:

bash
```
python3 examples/movement_to_laser_cross_sensor.py
```