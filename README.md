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
DRONE_SLAM/
├── dataset/           # Physical data
│   └── mav0/          # EuRoC extracted machine hall data
├── data_loaders/      # Scripts for data loading and syncing
│   ├── dataset_player.py
│   └── sync_test.py
├── perception/        # Camera modules
│   ├── orb_tracker.py
│   ├── stereo_depth.py
│   └── vision_test.py
├── imu/               # Inertial Measurement Unit
│   ├── imu_integrator.py
│   └── imu_plotter.py
├── planning/          # Map & Path planning
│   ├── occupancy_mapper.py
│   └── path_planner.py
├── localization/      # SLAM algorithms
│   └── slam_ekf.py
├── flight_control/    # Physical Drone Control
│   └── gps_denied_flight.py
├── run_sensor_fusion.py # main file
├── README.md
├── .gitignore
└── LICENSE
