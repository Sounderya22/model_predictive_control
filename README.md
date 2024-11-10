# robot_description_project_control_enpm667_1
Contains the robot description and controllers for simulation in ROS2 and Gazebo


### Dependencies

Make sure that ROS 2 Humble or Higher is already installed in your system.
You can follow the [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

### Build instructions

First, source your ROS 2 workspaces with all the required dependencies.
Then, you are ready to clone and build this repository.
You should only have to do this once per install.

```sh
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/Sounderya22/robot_description_project_control_enpm667_1.git --branch main
cd ..
rosdep install --from-path src --ignore-src -yi
colcon build
```

### Initialization instructions

You will have to do this in every new session in which you wish to use these examples:

```sh
source ~/dev_ws/install/local_setup.sh
```

### Run the examples

Refer to the individual examples README.md for instructions on how to run them.

### Potential pitfalls

If you are unable to automatically install dependencies with rosdep (perhaps due to [this issue](https://github.com/ros-infrastructure/rosdep/issues/733)), please do be sure to manually install the dependencies for your particular example of interest, contained in its package.xml file.
