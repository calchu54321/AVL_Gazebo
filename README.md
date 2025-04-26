# 🚗 Autonomous Parallel Parking Simulation Project

> **Note**: This project uses **ROS 2 Humble** and **Gazebo Fortress**.

---

## 📦 Required Dependencies:

Make sure to install the following dependencies **before running the code**:

```bash
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-topic-tools
sudo apt install ros-humble-ackermann-steering-controller
sudo apt install ros-humble-ign-ros2-control
sudo apt install ros-humble-ros-gz-bridge
```
## To Run code: 

Open new terminal,

```bash
cd AGV_ws #change name to your path
colcon build
source install/setup.bash
ros2 launch robot_description launch_sim.launch.py
```
Open another terminal,

```bash
source install/setup.bash
ros2 launch robot_description parallel_parking.launch.py
```

## Common Issues:
Cloning repository to your device: 

```bash
#change path to your path in launch_sim.launch.py code
os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join('/home/ubuntu/AGV_ws/src')
```

If you encounter multiple vehicle spawns in Gazebo or Rviz doesn't display properly, open new terminal

```bash
#Control+C each open terminal
ps aux | grep gazebo
kill -9 {first numbers}
```