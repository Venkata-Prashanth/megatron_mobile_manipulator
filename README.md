
 Megatron is an indoor mobile manipulation platform that has been developed using a commercially available radio-controlled (RC) robot base. The project aims to transform the platform from manual RC controlled to autonomous operation, enabling it to perform indoor robotic tasks.

The platform is equipped with a UR5e industrial robotic arm, which enables research and development in the areas of autonomous manipulation, navigation, perception and control systems. It is also used actively for the development, testing and validation of robotics algorithms in real-world indoor environments.
   
---
##  Architecture Overview

The system is split between an onboard industrial computer and a Teensy 4.1 microcontroller.

- **High-Level (ROS 2):** Handles perception, localization, mapping, sensor fusion, and autonomous navigation (Nav2).
- **Low-Level (micro-ROS):** Handles motor control, encoder processing, IMU acquisition, odometry, and safety-critical functions.
- ----

##  Requirements

* **OS:** Ubuntu 22.04 LTS 
* **ROS 2:** Humble

---
## Hardware Configuration  
  
- Mobile Base: [Cu-Chassis-XT(4WD)(AU))](https://www.ulrichc.de/product/robotic/cu-chassis-xt/4wd-au/) 
	- Motor controller : RoboClaw 2x60A
- Manipulator: UR5e (Universal Robots)  
- Sensors:  
	- Ouster OS0 Rev7 - 128 Channels Lidar 
	- ZED2i RGB-D Camera
	- ICM-20498 Low cost IMU   
- Onboard Computer: Nuvo 9166GC
----
## Current Project Status  

At the current stage, all algorithms and control frameworks are being developed and validated on the mobile base only. Integration with the manipulator is planned for future development.

The current robot control stack includes:

- Low-level PI velocity controllers for motor control
- Extended Kalman Filter (EKF) for sensor fusion of wheel encoder odometry and IMU measurements
- Nonlinear Model Predictive Control (NMPC) based motion controller using the robot's dynamic model
---
##  Getting Started

### 1. High-Level ROS 2 Setup

Dependencies

Bash 

```
sudo apt update 
sudo apt install ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-imu-tools ros-$ROS_DISTRO-rviz2
```

Clone the repository into your ROS 2 workspace `src` directory:

Bash

```
cd ~/ros2_ws/src
git clone https://github.com/HSM-Mobile-Robotics/megatron-mobile-manipulator.git
cd ~/ros2_ws
```

Install dependencies and build the workspace:

Bash

```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Low-Level micro-ROS Setup

#### A. Arduino IDE & Teensy Board Configuration
1. **Install Arduino IDE:** Ensure you have the Arduino IDE installed on your system.
2. **Setup Teensy Board Manager:**
   * Open Arduino IDE and go to `File` -> `Preferences`.
   * Add the Teensy URL to the **Additional Boards Manager URLs**: `https://www.pjrc.com/teensy/td_download.html`
   * Open the **Boards Manager** (`Tools` -> `Board` -> `Boards Manager`), search for **Teensy**, and install **Version 1.57.3**.
3. **Configure Linux udev Rules (Critical):** To allow your system to communicate with the Teensy board without root permissions, install the udev rules by running:
	```bash
	sudo cp megatron_firmware/00-teensy.rules /etc/udev/rules.d/
	```


#### B. Install Required Arduino Libraries

Open the Arduino IDE **Library Manager** (`Tools` -> `Manage Libraries`) or manually clone and install the following dependencies:

- **Sensor & Motor Libraries:**
    
    - [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary.git) Version : 1.32
        
    - [Basicmicro RoboClaw Arduino Library](https://github.com/basicmicro/roboclaw_arduino_library.git) 
        
- **micro-ROS Library:**
    
    - Search for `micro_ros_arduino` in the Library Manager and install **Version v2.0.7-humble**.
        

#### C. Install micro-ROS Agent on Ubuntu

The high-level system requires a micro-ROS agent to communicate with the Teensy microcontroller. Build the agent in a dedicated workspace:

Bash

```
# Source your ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a new workspace
mkdir -p ~/uros_ws/src && cd ~/uros_ws

# Clone the setup repository
git clone -b $ROS_DISTRO [https://github.com/micro-ROS/micro_ros_setup.git](https://github.com/micro-ROS/micro_ros_setup.git) src/micro_ros_setup

# Install dependencies and build
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# Create and build the specific agent package
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

#### D. Flashing the Firmware

1. Open the Arduino IDE and navigate to the project directory: `megatron_firmware/megatron_robot`.
    
2. Select your target board: `Tools` -> `Board` -> `Teensy` -> **Teensy 4.1**.
    
3. Select the appropriate communication port: `Tools` -> `Port`.
    
4. Click the **Upload** button to compile and flash the firmware onto the microcontroller.

## Running the Robot

To bring the entire system to life, follow these steps:

Bash

```
cd ~/uros_ws
source install/setup.bash

cd ..
cd ~/megartron_ws
source install/setup.bash

ros2 launch megatron_bringup bringup.launch.py
```


## NMPC Implementation Path following

### Acados Installation for Linux (Ubuntu)

For more details, visit the [Acados Installation Guide](https://docs.acados.org/installation/).

```bash

cd ~/acados_ws/

git clone [https://github.com/acados/acados.git](https://github.com/acados/acados.git)

cd acados

git submodule update --recursive --init

```

#### Installation via CMake

Build and install `acados` using the following commands:

```bash

mkdir -p build

cd build

cmake -DACADOS_WITH_QPOASES=ON ..

# Note: You can add more optional arguments, e.g., -DACADOS_WITH_DAQP=ON.

# A full list of CMake options is provided in the official documentation.

make install -j4  
```

  

> **Note:** We are using the Python interface. The control problem is formulated in Python, and the corresponding C files are automatically generated when the code is executed.

  

#### Python Interface Installation

1. **Prerequisite:** Ensure `acados` is successfully installed using the CMake steps above.

2. **Virtual Environment (Recommended):** Create and activate a Python virtual environment.

```bash
python3 -m venv megatron_pyenv

source megatron_pyenv/bin/activate
```

3. **Install the `acados_template` Python package:**

```bash

pip install -e /path/to/acados/interfaces/acados_template

```



4. **Configure Environment Variables:** Add the paths of the compiled shared libraries (`libacados.so`, `libblasfeo.so`, `libhpipm.so`) and the source directory to your system path.

  

```bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path/to/acados/lib"

export ACADOS_SOURCE_DIR="/path/to/acados"

```

*Tip: You can append these `export` lines to your `~/.bashrc` file so they load automatically whenever you open a new terminal.*

#### Installing CasADi for Problem Formulation

Ensure your virtual environment is active, then install CasADi:


```bash

source megatron_pyenv/bin/activate

pip install casadi

```

Running the robot NMPC (Temporary setup)

Step: 1 Generate NMPC C-Code

```
    cd ~/megatron_mpc/acados_mpc

    source megatron_pyenv/bin/activate

    python3 megatron_generate_c_code_3states.py
```

Step: 2 launch the robot 

Open a new terminal, source your core workspace, and launch the robot 

```
source ~/megatron_ws/install/setup.bash

ros2 launch megatron_bringup bringup.launch.py

```

Tuning & Simulation

To tune parameters or run the robot model simulator than the bringup 


ros2 run megatron_mpc robot_model_sim

In another terminal run the controller node

```
ros2 run megatron_mpc mpc_controller_node

```

Open new terminal 

```
ros2 run megatron_mpc mpc_lifecycle_client

```


## Running the Robot NMPC (Temporary Setup)

### Step 1: Launch the Robot Bringup

Open a new terminal, source your core workspace, and launch the hardware drivers:

```bash
source ~/megatron_ws/install/setup.bash
ros2 launch megatron_bringup bringup.launch.py

```

**Tuning & Simulation Alternative:** If you want to tune parameters or run the robot model simulator **instead** of the physical hardware bringup, run this node instead of the launch file above:
```bash
source ~/megatron_ws/install/setup.bash
ros2 run megatron_mpc robot_model_sim

```

### Step 2: Generate NMPC C-Code

Open a new terminal, activate your Python virtual environment, and generate the required C-code files:

```bash
cd ~/megatron_ws/src/megatron-mobile-manipulator/megatron_mpc/acados_mpc
source ~/megatron_pyenv/bin/activate
python3 megatron_generate_c_code_3states.py

```

### Step 3: Run the Controller Node

Open a new terminal, source the workspace, and start the core NMPC controller node:

```bash
source ~/megatron_ws/install/setup.bash
ros2 run megatron_mpc mpc_controller_node

```

### Step 4: Activate the Lifecycle Client

Open one final terminal to trigger and manage the controller's lifecycle state:

```bash
source ~/megatron_ws/install/setup.bash
ros2 run megatron_mpc mpc_lifecycle_client

```



