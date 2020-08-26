# motors_interface

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/a981b185b9154d91a468ef7b8974ceeb)](https://app.codacy.com/gh/Mailamaca/joystick_command?utm_source=github.com&utm_medium=referral&utm_content=Mailamaca/joystick_command&utm_campaign=Badge_Grade_Settings)

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
    ros2 launch motors_interface motors_interface.launch.py
    ```

# Copyright and License

This software contains code licensed as described in LICENSE.