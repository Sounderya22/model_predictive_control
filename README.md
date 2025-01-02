# Vision-based model predictive controller
Contains the robot description and controllers for simulation in ROS2 and Gazebo


### Dependencies

Make sure that ROS 2 Humble or Higher is already installed in your system.
You can follow the [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

Install the following Dependencies to run the code
```sh
pip install opencv-python numpy cvxpy matplotlib osqp
```
For ROS2 CV Bridge:
```sh
sudo apt install ros-<ros2-distro>-cv-bridge
```


### Build instructions

First, source your ROS 2 workspaces with all the required dependencies.
Then, you are ready to clone and build this repository.
You should only have to do this once per install.

```sh
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/Sounderya22/model_predictive_control.git
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

After Building and Sourcing the Workspace, Launch the TurtleBot using:

```sh
ros2 launch turtlebot3_gazebo custom_world.launch.py
```
Open a Separate Terminal Window to run the controllers.

Run the PD Controller:

```sh
ros2 run my_robot pd_control.py
```
Run the MPC Contoller:

```sh
ros2 run my_robot mpc_control.py
```


### Potential pitfalls

If you are unable to automatically install dependencies with rosdep (perhaps due to [this issue](https://github.com/ros-infrastructure/rosdep/issues/733)), please do be sure to manually install the dependencies for your particular example of interest, contained in its package.xml file.
