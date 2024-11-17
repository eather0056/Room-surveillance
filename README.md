# Autonomous TurtleBot3 Navigation in Maze Environment

This repository contains a ROS2-based simulation for multi-robot navigation in a maze environment using TurtleBot3 robots. The project includes mapping, navigation, and synchronous robot behavior triggered by external signals.

---

## **Project Overview**
This project aims to simulate and implement the following functionalities:
1. **SLAM (Simultaneous Localization and Mapping):** Creating a 2D map of the maze environment.
2. **Multi-Robot Navigation:** Enabling multiple robots to navigate to target positions in the maze.
3. **Synchronous Behavior:** Robots start moving simultaneously to new rooms upon receiving a specific signal.
4. **Simulation Environment:** Built using Gazebo with TurtleBot3 robots and a custom maze world.

---

## **Folder Structure**
```plaintext
autonomous_tb3/
├── autonomous_tb3/
│   ├── __init__.py
│   ├── goal.py              # Script for multi-robot goal publishing
│   ├── maze_solver.py       # Solver for maze navigation (if implemented)
│   ├── send_goals.py        # Custom script to send goals to robots
├── config/                  # Configuration files for SLAM, navigation, and RViz
├── launch/                  # Launch files for SLAM, navigation, and robot spawning
│   ├── centralized_slam.launch.py
│   ├── empty_world_no_robot.launch.py
│   ├── multi_mapping.launch.py
│   ├── multi_robot_navigation.launch.py
│   ├── new_navigation.launch.py
│   ├── slam_toolbox.launch.py
├── maps/                    # Contains YAML and PGM map files
├── models/                  # Custom models for the maze and robots
├── urdf/                    # TurtleBot3 URDF and xacro files
├── world/                   # Gazebo world files for the maze environment
├── worlds/
├── rviz2_multi.rviz         # RViz configuration file for multi-robot navigation
├── package.xml              # ROS2 package configuration
├── setup.cfg                # Python setup configuration
├── setup.py                 # Python setup script
```

---

## **How to Use**

### **1. Clone the Repository**
```bash
git clone https://github.com/yourusername/autonomous_tb3.git
cd autonomous_tb3
```

### **2. Build the Workspace**
Ensure you have a ROS2 workspace set up. Place this repository in the `src` folder and build the workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

### **3. Run SLAM for Mapping**
To create a map of the maze environment:
```bash
ros2 launch autonomous_tb3 slam_toolbox.launch.py
```

### **4. Save the Map**
After completing the mapping in RViz:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### **5. Run Multi-Robot Navigation**
To start navigation with multiple robots:
```bash
ros2 launch autonomous_tb3 multi_robot_navigation.launch.py
```

### **6. Trigger Synchronous Movement**
Run the custom script to send goals to the robots:
```bash
ros2 run autonomous_tb3 goal
```

---

## **Features**
- **Gazebo Simulation:** Simulates a real-world maze environment.
- **Multi-Robot Coordination:** Allows multiple robots to navigate simultaneously.
- **SLAM Integration:** Uses `slam_toolbox` for mapping.
- **Navigation Stack:** Leverages ROS2 Navigation Stack for planning and execution.
- **Custom Goal Management:** Includes a script to dynamically publish goals.

---

## **Future Enhancements**
1. Implement real-world testing with hardware.
2. Improve robot behavior for obstacle avoidance in narrow spaces.
3. Add support for advanced communication protocols between robots.

---

## **Acknowledgments**
This project uses:
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [Gazebo](http://gazebosim.org/)
- [ROS2 Navigation](https://navigation.ros.org/)

