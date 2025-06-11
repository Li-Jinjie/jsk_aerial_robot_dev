# Beetle-Art-Omni

## Installation

### 1. Install `acados`

The version of `acados` should be aligned with the version of `3rdparty/acados/CMakeLists.txt` -> `GIT_TAG`.

Specifically, clone the branch for **v0.5.0** from the `acados` git:
```bash
git clone https://github.com/acados/acados.git --branch v0.5.0
```
Then, follow the instructions below:
- Install acados itself: Please follow the instructions on the acados website https://docs.acados.org/installation/index.html
- Install Python interface: Please follow the instructions on the acados website https://docs.acados.org/python_interface/index.html, but don't create virtual env in step 2. The virtual env has compatibility problem with ROS env.
- **Pay attention** that you must execute the step 5 in https://docs.acados.org/python_interface/index.html to test the installation. This step should automatically install t_renderer. If something wrong, please follow step 6 to manually install t_renderer.

### 2. Install the code base and the necessary ROS related packages ...

Setup the folder architecture and clone the repo **with the specific branch**:

```bash
mkdir -p ~/[path_to_ws]/src
cd ~/[path_to_ws]/src
git clone https://github.com/Li-Jinjie/jsk_aerial_robot_dev.git -b develop/MPC_tilt_mt    # pay attention to the branch flag
```

### 2.1 ... for Ubuntu 20.04 and ROS Noetic
Install ROS Noetic for Ubuntu 20.04 from https://wiki.ros.org/ROS/Installation and source the setup file:

```bash
source /opt/ros/one/setup.bash
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:
```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot/aerial_robot_noetic.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic   # install the dependencies/packages stated in package.xml
```

### 2.2 ... for Ubuntu 22.04 and ROS-O
Install ROS-O for ubuntu 22.04 from https://ros.packages.techfak.net/ and source the setup file:

```bash
./jsk_aerial_robot/configure.sh   # for configuration especially for ROS-O in jammy
source /opt/ros/one/setup.bash
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:

```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO      # install the dependencies/packages stated in package.xml
```

For convenience, open `~/.bashrc` and add sourcing of the workspace to the end of the file:

```bash
In ~/.bashrc:

source ~/[path_to_ws]/devel/setup.bash
```

### 3. Install python packages and link them to acados
Install required packages:
```bash
pip install -r src/jsk_aerial_robot/aerial_robot_control/scripts/requirements.txt
```

For the first run, **uncomment** these code in `aerial_robot_control/CMakeLists.txt`
```bash
set(ACADOS_PYTHON_SCRIPTS
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_qd/fix_qd_angvel_out.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_qd/fix_qd_thrust_out.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_no_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo_dist.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo_thrust_dist.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_tri/tilt_tri_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_bi/tilt_bi_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_bi/tilt_bi_2ord_servo.py
)
```

### 4. Build the workspace with `catkin`

```bash
cd ~/path_to_ws
catkin build
```

A frequent problem is the handling of the jobservers in the build process. When occuring during the build process - especially in the aerial_robot_control package - please try simply running the build command again.

### 5. If the build is successful, comment the code in step 3 back.

## Simulation

### 1. Start the simulation
Run the simulation with the following command:
```bash
roslaunch beetle_omni bringup_nmpc_omni.launch real_machine:=false simulation:=True headless:=False nmpc_mode:=0
```
### 2. Start the keyboard script
Run the keyboard with the following command:
```bash
rosrun aerial_robot_base keyboard_command.py
```
Then input `r` to arm the motors and input `t` to let the drone take off.

### 3. Send the trajectory
**After the main window printed 'Hovering'**, please run the following command to send a trajectory:
```bash
rosrun aerial_robot_planning mpc_smach_node.py beetle1
```
Then choose the trajectory you want the drone to perform.

# Flying Hand

We need two notebooks, one as ground station and the other for visual feedback.

**MoCap Computer**

1. Run the software for the data glove and calibrate it. Note that the ip should be set to the ros master computer.
2. Check the MoCap is set correctly.

If there is something wrong with glove connection, please refer
to https://stretchsense.my.site.com/defaulthelpcenter26Sep/s/article/Studio-Glove-and-Dongle-Setup?language=en_US

**Onboard Computer**

Four terminals are needed.

Before takeoff, launch necessary files

1. `roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none  # This is the main launch file for the robot.`
2. `roslaunch aerial_robot_planning hand_arm_glove.launch`
3. `rosbag record -a`

After the robot reach the status of hovering - see terminal output of main launch file -,

4. Run `rosrun aerial_robot_planning mpc_smach_node.py beetle1`, and press 'h' to enter the hand control mode. Note that
   we need to wear the hand part, the shoulder part, and gloves to control the robot.
5. To modify the `/hand/control_mode` parameter using `rosparam` in the terminal, run the following command:
    `rosparam set /hand/control_mode <mode_value>`
    Where `<mode_value>` can be set as one of the following:
    - **1**: Operation Mode
    - **2**: Spherical Coordinate Mode
    - **3**: Cartesian Coordinate Mode
    - **4**: Lock Mode
    - **5**: Exit Mode

    To check the current mode, you can run:`rosparam get /operation_mode`
    
**Ground Station**
Three terminals are needed.

Before takeoff:

1. run `sudo ds4drv` to connect the joystick.
2. `roslaunch aerial_robot_base joy_stick.launch robot_name:=beetle1`
3. `rviz -d ~/ros1/jsk_ws/src/jsk_aerial_robot_dev/robots/beetle/config/nmpc.rviz`

**Visual Computer**

1. `rosrun aerial_robot_planning visual_fb_mode.py`
2. Adjust the window to fullfill the screen.