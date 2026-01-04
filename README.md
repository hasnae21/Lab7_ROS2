# 🤖 ROS 2 Integrated Lab 07: Modular Robot Design & Simulation

This repository implements **Lab 7** of the ROS 2 curriculum, focusing on **robot modeling**, **modular design with Xacro**, and **simulation in Gazebo**. The lab demonstrates the full pipeline from URDF definition to physics-aware simulation in custom environments.

> ⏱️ **Duration**: 6+ Hours  
> 🎯 **Objective**: Master URDF, Xacro, Gazebo, and robot-state publishing in ROS 2

---

## 📁 Package Structure

```
robotics_sim_project/
├── worlds/                     # Custom Gazebo world files
│   ├── simple_test_world.world
│   └── group_project_world.world
├── urdf/                       # Robot descriptions
│   ├── simple_arm.urdf                 # Plain URDF model
│   └── simple_arm.urdf.xacro           # Modular Xacro version
├── urdf/common/                # Reusable Xacro macros
│   ├── properties.xacro        # Geometric & mass parameters
│   ├── inertias.xacro          # Inertia calculation macros
│   ├── materials.xacro         # RViz visual materials
│   └── gazebo_materials.xacro  # Gazebo-specific materials
├── launch/                     # ROS 2 launch files
│   ├── rviz_display.launch.py  # Visualize in RViz with joint GUI
│   └── gazebo_simple_arm.launch.py  # Spawn robot in Gazebo
├── meshes/                     # (Optional) 3D mesh files
├── CMakeLists.txt
└── package.xml
```

---

## 🧩 Key Concepts Demonstrated

### Part 1: URDF-Based Robot Modeling
- Created a **2-link robotic arm** using native URDF
- Defined **visual, collision, and inertial** properties
- Added **Gazebo-specific tags** for realistic simulation
- Visualized in **RViz** and spawned in **Gazebo**

### Part 2: Modular Design with Xacro
- Converted URDF to **Xacro** for reusability
- Created **parameterized macros** for:
  - Geometry (length, width, height)
  - Mass and inertia (auto-calculated)
  - Materials (for RViz & Gazebo)
- Used **`xacro` command substitution** in launch files

### Part 3: Custom Simulation World
- Built a **custom Gazebo world** with:
  - Static obstacles (walls, ramps, platforms)
  - Dynamic objects (boxes, spheres)
  - Custom lighting and physics settings
- Designed for **robot interaction and navigation challenges**

---

## 🛠️ How to Build & Run

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
                  ros-humble-robot-state-publisher \
                  ros-humble-joint-state-publisher-gui \
                  ros-humble-xacro \
                  liburdfdom-tools
```

### Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select robotics_sim_project
source install/setup.bash
```

### 🖥️ Visualize in RViz (URDF or Xacro)
```bash
ros2 launch robotics_sim_project rviz_display.launch.py
```
> Use the **Joint State Publisher GUI** to move the arm.

### 🌍 Simulate in Gazebo
```bash
ros2 launch robotics_sim_project gazebo_simple_arm.launch.py
```
> The robot spawns at `(0, 0, 0.5)` in the custom world. Click ▶ to start physics.

---

## ✅ Expected Behavior

| Environment | Behavior |
|------------|--------|
| **RViz** | Robot appears with blue base + red arm; joint slider controls rotation |
| **Gazebo** | Robot falls under gravity, lands on ground plane, and stabilizes |

> 💡 **Tip**: The Xacro and URDF versions should behave identically — but Xacro is **easier to modify** (change one parameter in `properties.xacro` to update the whole robot!).

---

## 📚 Learning Outcomes

By completing this lab, you will have mastered:
- URDF syntax for robot description
- Gazebo integration (spawn, physics, materials)
- Xacro for **modular, maintainable robot models**
- Launch file development with `robot_state_publisher` and `gazebo_ros`
- Custom world creation in SDF format

---
