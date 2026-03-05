#  Team #include: GPS-Denied Autonomous UAV

**Core Stack:** ROS Humble, C++, Python, VINS-Fusion, PX4 Autopilot

##  The Mission
To develop a fully autonomous drone capable of indoor navigation without GPS, utilizing Visual-Inertial Odometry (VIO) via an Intel RealSense D435i and Pixhawk Flight Controller.

##  Members of Team
* **Phani :** VIO Integration & System Architect
* **Radha Madhav:** Embedded Systems & Hardware Integration
* **Revanth:** Simulation & State Machine
* **Durga Prasad:** Flight Control & MAVROS Bridge
* **Raj Reddy:** Computer Vision & Feature Tracking

##  Branching Rule
**DO NOT PUSH TO `main`.** 1. Create a branch: `git checkout -b feature/your-name-task`
2. Push your code: `git push origin feature/your-name-task`
3. Open a **Pull Request** and tag Phani for review.



## Project Structure
```text
Team_include/
├── CMakeLists.txt                
├── package.xml                   
│
├── launch/
│   └── autonomous_flight.launch
│
├── planning/
│   ├── path_planner.py
│   └── occupancy_mapper.py
│
├── flight_control/
│   └── gps_denied_flight.py
│
├── perception/
│   └── yolo_detector.py
│
├── scripts/
│   └── vins_to_mavros.py     
│
├── legacy_prototypes/           
│   ├── dataset_player.py
│   ├── slam_ekf.py
│   ├── optical_flow.py
│   ├── run_sensor_fusion.py
│   └── ... (all other old files)
│
└── tests/
    ├── vision_test.py
    ├── yolo_test.py
    └── test_mapper_windows.py
