# HOMEWORK_1 PATELLARO EMMANUEL P38000239 #
# BUILDING A ROBOT MANIPULATOR #
This README file will show the instructions on how to build and run the Homework_1 Project 

## Features ##
- Description of an arm manipulator
- Visualization in RViz2
- Spawn and Control in Gazebo
- Camera Sensor
- Publisher adn Subscriber Node

## Available Packages in this Repository ##
- arm_description
- arm_gazebo
- arm_control

## Getting Started
1. Follow the guide to install ROS2 in Docker [here](https://github.com/RoboticsLab2024/ros2_docker_scripts.git)
2. Clone this repo in your `src` folder inside the `ros2_ws`
    ```shell
    cd src
    git clone https://github.com/EmmanuelPat6/Homework_1.git
    ```
3. Build the packages and make them visible to your workspace
    ```shell
    colcon build
    source install/setup.bash
    ```

## Usage
The `arm_description` package contains 1 launch file
- `display.launch.py` which runs RViz2 

The `arm_gazebo` package contains 2 launch files
- `arm_gazebo.launch.py` which spawns the arm in Gazebo 
- `arm_world.launch.py` which launches `arm_gazebo.launch.py` and `arm_control.launch.py`

The `arm_control` package contains 1 launch file
- `arm_control.launch.py` which spawns the controllers

### Implementation
1. Launch `arm_gazebo.launch.py`  in a sourced terminal run
    ```shell
    ros2 launch arm_gazebo arm_gazebo.launch.py
    ```
2. Run Publisher and Subscriber contained in `arm_controller_node.cpp` in another terminal
    ```shell
    ros2 run arm_control arm_controller
    ```
    After a few seconds the robot should move and in the terminal, the various joint variables value will be displayed.

3. To view the robot in RViz2, run in another terminal (only if `arm_gazebo.launch.py` is already launched)
    ```shell
    ros2 launch arm_description display.launch.py
    ```
    The image viewed by the camera will also be displayed automatically.

4. To show the image captured by the camera separately, it is possible to see it through `rqt_image_view`. Run
    ```shell
    rqt
    ```
    and go in `Plugins->Visualization->Image` View and select `/videocamera`.

    To appreciate the camera behavior, it is possible to add some objects in the `Gazebo Environment`.

5. To give other commands to the robot, it is necessary to stop the `arm_controller` node (in which there are the Publisher and the Subscriber). It is both possible to use this command
    ```shell
    ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]" 
    ```
    with the desired joint positions, or run the New Publisher (not required by the project specifications) which allows to give these values directly from the terminal without the entire command but simply by entering the values separated by a space.
   ```shell
    ros2 run arm_control publisher_terminal
    ```
    In this case, to keep track of the joint position values, it is necessary to do
   ```shell
    ros2 topic echo /joint_states
    ```
