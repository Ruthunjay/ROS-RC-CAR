# ROS-RC-CAR

Controlling the bot using keyboard of the pc (by creating ROS node).

# ROS RC Car Control via XBee

This project enables remote control of an RC car using ROS (Robot Operating System) and an XBee module. The system consists of an Arduino-based RC car controlled by a Python script that communicates with the car via XBee modules. The project allows users to send velocity commands to the RC car, enabling it to move forward, backward, left, and right.

## Hardware Requirements

- Arduino board (e.g., Arduino Uno)
- XBee module (2 units)
- Motor driver shield (e.g., L298N) for controlling the motors of the RC car
- RC car chassis with motors and wheels
- USB cables
- Jumper wires

## ROS Setup

1. Install ROS on your computer if not already installed.
2. Create a ROS workspace and clone the provided Python script (`rc_car_control.py`) into the `src` directory.
3. Build the ROS package using `catkin_make`.

## Arduino Setup

1. Upload the provided Arduino sketch (`rc_car_control.ino`) to the Arduino board connected to the RC car.

## Usage

1. Connect the XBee modules to the Arduino and your computer.
2. Power up the RC car and the Arduino.
3. Launch the ROS package with the command: `roslaunch your_package_name rc_car_control.launch` (Make sure to replace `your_package_name` with the actual name of your ROS package.)
4. Publish velocity commands to the `/cmd_vel` topic to control the RC car. The Python script will transmit the commands to the Arduino via XBee, and the car will respond accordingly.

## ROS Node: rc_car_control.py

This ROS node subscribes to the `/cmd_vel` topic to receive velocity commands for controlling the RC car. It prepares the data and sends it to the Arduino using the XBee module. The Python script also listens for responses from the XBee module.

## Arduino Sketch: rc_car_control.ino

This Arduino sketch reads the velocity commands sent from the Python script via XBee and controls the motors of the RC car accordingly. It uses a motor driver shield to control the direction and speed of the car's movement.

## Future Enhancements

This project can be enhanced in several ways:

- Implementing obstacle detection and avoidance using additional sensors.
- Adding a camera to provide a live video feed from the RC car to the user interface.
- Implementing more advanced control algorithms for smoother and more accurate movement.
- Designing a custom PCB for the Arduino and XBee integration.

Feel free to customize and expand upon the project to suit your specific requirements and preferences.

## License

This project is licensed under the [MIT License](LICENSE).


