# Task 2 – Dual Arm Architecture & Coordination

This repository demonstrates a **modular ROS 2 software architecture** for a **dual-arm robotic system**, emphasizing clear separation between the **user interface**, **coordination logic**, and **robot description**.

The design focuses on decoupling, extensibility, and clean responsibility boundaries between system components.

---

## 📦 Repository Structure

```
src/
├── dexsent_control
│   ├── config/
│   ├── include/
│   │   └── dexsent_control/
│   ├── launch/
│   ├── scripts/
│   │   ├── cartesian_controller.py
│   │   └── dual_arm_gui.py
│   ├── src/
│   ├── CMakeLists.txt
│   ├── dexsent_control.xml
│   └── package.xml
│
├── dexsent_description
│   ├── urdf/
│   │   ├── archive/
│   │   ├── dual_box.urdf
│   │   └── dual_system.urdf
│   ├── CMakeLists.txt
│   └── package.xml
│
├── .gitignore
└── README.md
```

### Package Overview

- **`dexsent_description`**  
  Contains URDF descriptions for the dual-arm system, including:
  - Shared base
  - Dual-arm mounting configuration
  - Supporting structural elements

- **`dexsent_control`**  
  Contains all control-side logic, including:
  - Centralized Cartesian coordination controller
  - Graphical User Interface (GUI)
  - Launch files and configuration for simulation and visualization

---

## 🚀 Setup & Execution

### Prerequisites

- **Ubuntu 22.04**  
- **ROS 2 Humble**  
- **Python 3**  

---

### Installation

1. **Clone the repository** into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

2. **Build the packages**:

```bash
cd ~/ros2_ws
colcon build --packages-select dexsent_description dexsent_control
```

3. **Source the workspace**:

```bash
source install/setup.bash
```

---

## ▶️ Running the System

The system is executed using **three terminals**, each corresponding to a distinct architectural layer.

### Terminal 1 – Simulation & Visualization

Launch the dual-arm robot description and RViz environment:

```bash
ros2 launch dexsent_control dual_arm_control.launch.py
```

This starts:
- Dual-arm URDF model
- RViz visualization
- Required ROS 2 infrastructure

---

### Terminal 2 – Controller Logic

Run the centralized Cartesian coordination controller:

```bash
python3 src/dexsent_control/scripts/cartesian_controller.py
```

**Responsibilities:**
- Receives commands from the UI
- Computes Cartesian targets
- Coordinates both arms
- Publishes synchronized control commands

---

### Terminal 3 – User Interface

Launch the graphical user interface:

```bash
python3 src/dexsent_control/scripts/dual_arm_gui.py
```

**Responsibilities:**
- Captures user input for Cartesian motion
- Publishes high-level commands via ROS topics
- Remains independent of robot implementation details

---

## 💡 Key Design Decisions

### Decoupled Control Architecture

- The **GUI** communicates with the system exclusively via **ROS topics**.
- The **controller node** acts as the single authority for motion logic.
- This enables:
  - Easy replacement of the UI (CLI, web UI, scripts)
  - Reuse of control logic across simulation and real hardware

### Centralized Dual-Arm Coordination

- Both arms are coordinated by a **single controller node**.
- Prevents conflicting or unsynchronized commands.
- Provides a foundation for:
  - Bimanual manipulation
  - Constraint-based coordination
  - Task-level planning

### State Initialization Strategy

- A **Home state** is explicitly triggered during startup.
- Ensures synchronization between:
  - RViz visualization
  - Controller internal kinematic state
  - GUI input fields
- This avoids discrepancies between displayed and actual robot states.

---

## 🎮 System Architecture

```
┌─────────────────┐
│   GUI Layer     │  ← User interaction via Tkinter
│  (dual_arm_gui) │
└────────┬────────┘
         │ ROS Topics
         ▼
┌─────────────────────────┐
│   Control Layer         │  ← Coordination logic
│ (cartesian_controller)  │
└────────┬────────────────┘
         │ Joint States
         ▼
┌─────────────────────────┐
│  Simulation Layer       │  ← Visualization
│  (RViz + URDF)          │
└─────────────────────────┘
```

---

## ✅ Key Features

- ✨ Modular dual-arm architecture
- 🔌 ROS 2 topic-based communication
- 🎯 GUI-independent backend logic
- 🤝 Centralized coordination controller
- 🧩 Clean separation of concerns
- 🔄 State synchronization mechanism

---

## 🛠️ Topics and Communication

| Topic | Type | Purpose |
|-------|------|---------|
| `/dual_arm/cartesian_command` | `std_msgs/String` | High-level commands from GUI |
| `/joint_states` | `sensor_msgs/JointState` | Robot state visualization |

---

## 📌 Notes

- This implementation is intended for **architecture and coordination demonstration**.
- No hardware drivers are included.
- The system can be extended to real robots by replacing the simulation layer.
- The controller uses a simplified Cartesian-to-joint mapping suitable for demonstration purposes.

---

## 🔧 Extending the System

### Adding New Commands

1. Update `dual_arm_gui.py` to add new buttons or input fields
2. Publish commands to `/dual_arm/cartesian_command`
3. Implement command handling in `cartesian_controller.py`

### Hardware Integration

1. Replace the simulation launch file with hardware drivers
2. Update controller to interface with real robot controllers
3. Maintain the same topic-based communication interface

---

## 📄 License

This project is provided for technical evaluation purposes. All rights reserved by the author.
