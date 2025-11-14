# 🚁 Drone Waypoint Simulation (Phase 1: Kinematics)

A comprehensive MATLAB/Simulink project that simulates a quadcopter UAV performing autonomous waypoint tracking. This project utilizes the **UAV Toolbox** for guidance and a custom **PID controller** for flight stability.

## 🌟 Key Features
* **Guidance Model:** Uses a kinematic approximation for efficient path planning validation.
* **Custom URDF Support:** Imports custom drone designs (SolidWorks/CAD exports) for visualization.
* **Waypoint Tracking:** Algorithms to follow specific 3D paths (Square, Zig-Zag, or imported).
* **Automated Setup:** Includes a `setup_drone_complete.m` script that auto-generates strict Simulink Bus definitions to prevent common data-type errors.
* **NED Coordinate System:** Implements aviation-standard North-East-Down logic.

## 🛠️ Prerequisites
You need **MATLAB R2021b or newer** (Tested on R2025a) with the following toolboxes:
* Simulink
* UAV Toolbox
* Robotics System Toolbox

## 📂 Project Structure
* `Drone_UAVScenario.slx` - The main Simulink simulation model.
* `setup_drone_complete.m` - **Run this first!** Initializes variables, gains, and bus objects.
* `Drone_Assembly_MATLAB.urdf` - The robot description file (Visuals & Physical properties).
* `meshes/` - Folder containing STL files for the drone 3D model.

## 🚀 How to Run
1.  **Clone the repo** to your local machine.
2.  Open MATLAB and navigate to the project folder.
3.  Open `setup_drone_complete.m` and click **Run**.
    * *Check the Command Window for "✅ READY TO RUN".*
4.  Open `Drone_UAVScenario.slx`.
5.  Click **Run** in Simulink.
6.  The **UAV Animation** window will pop up to show the flight.

## ⚙️ Customization
### Changing the Path
To change the flight path, open `setup_drone_complete.m` and modify the `waypoints` matrix:
```matlab
% Format: [North, East, Altitude(Negative)]
waypoints = [0 0 -5; 10 5 -5; ...];
```
## Tuning Flight Behavior
Modify the PID gains in the setup script to change how aggressive the drone flies:

```
Kp_pos: Increases speed towards target.

Kp_yaw: Adjusts turning speed (Keep positive for standard frames).

Kp_z: Adjusts altitude hold strength.
```
## 🐛 Troubleshooting
"Bus Mismatch" / "Input bus type mismatch":

This usually happens if variables are cleared. Re-run setup_drone_complete.m to regenerate the strict Bus Definitions.

# Drone Spinning Uncontrollably:

This is often a yaw coordinate conflict. Try inverting the Kp_yaw sign in the setup script.

# Drone Invisible in Animation:

The drone might be too small relative to the flight height. Use the "Home" button in the animation window to fit the view.
