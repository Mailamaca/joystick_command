# motors_interface
ROS Node to interact with motors

## Initialize ROS2 package

1. Create ros2 C++ package

    ```bash
    ros2 pkg create --build-type ament_cmake --node-name joystick_command joystick_command
    ```

2. Build package

    ```bash
    colcon build --packages-select joystick_command
    ```

3. Source the setup file

    ```bash
    . install/setup.bash
    ```

4. Use the package

    ```bash
    ros2 run joystick_command joystick_command
    ```

5. Customize package.xml

6. Add launch file, a yaml for parameters and start it

    ```bash
    ros2 launch joystick_command joystick_command.launch.py
    ```

# Copyright and License

This software contains code licensed as described in LICENSE.