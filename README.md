# 🚗 EVA - Electric Vehicle Assistant

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Qt](https://img.shields.io/badge/Qt-6.10-green)](https://www.qt.io/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-17-orange)](https://en.cppreference.com/)

**EVA** (Electric Vehicle Assistant) is a complete autonomous navigation system combining a ROS2-based trajectory planning module with a professional Qt6/QML HMI interface.

> 🔄 **This project is an enhanced version of** [eva_trajectory_planning](https://github.com/Zakariajouhari1/eva_trajectory_planning) - A trajectory planning system for autonomous vehicles using A* and Cubic Splines. EVA v2.0 adds a professional HMI interface and complete system integration.

---

## 🎥 Demo Video

Watch EVA in action! Click below to see the complete navigation system working in real-time:
https://github.com/Zakariajouhari1/eva-autonomous-vehicle/blob/main/Demo.mkv

<!-- Alternative si la vidéo est dans le repo -->
<div align="center">
  <video src="Demo.mkv" width="800" controls>
    Your browser does not support the video tag.
  </video>
</div>

<!-- Ou avec un GIF si vous en créez un -->
![EVA Demo](Docs/Demo.mp4)

---

## 🏗️ System Architecture

<div align="center">
  <img src="Docs/Images/Architecture.png" alt="EVA System Architecture" width="300"/>
  <p><i>Complete system architecture showing all components and communication flow</i></p>
</div>



### Component Breakdown

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Planning Node** | ROS2 + C++17 | Global path planning with OSRM, cubic spline smoothing |
| **Vehicle HMI** | Qt6 + QML | Professional dashboard with real-time visualization |
| **Simulator** | Python + ROS2 | Physics-based vehicle simulation for testing |

<div align="center">
  <img src="Docs/Images/HMI.png" alt="EVA HMI Interface" width="400"/>
  <p><i>Professional Qt6/QML interface with real-time navigation</i></p>
</div>

---

## ✨ Features

### 🗺️ Trajectory Planning (ROS2)
- **OSRM Integration**: Real-world road routing using OpenStreetMap
- **A* Global Planning**: Optimal path finding on road networks
- **Cubic Spline Smoothing**: Local trajectory refinement for comfort
- **GPS ↔ Local Conversion**: Accurate coordinate transformation
- **Real-time Planning**: ~100-300ms computation time



### 🖥️ Professional HMI (Qt6/QML)
- **Interactive Map**: OpenStreetMap with route visualization
- **Real-time Dashboard**: Speed, battery, vehicle status
- **Responsive Design**: Adapts to any screen size
- **Live Navigation**: Distance, ETA, progress tracking
- **Modern UI/UX**: Professional automotive-grade interface



### 🚙 Vehicle Simulator
- **Physics-based**: Realistic acceleration and movement
- **Path Following**: Automatic trajectory tracking
- **Odometry Publishing**: Real-time position updates
- **Configurable Parameters**: Speed, acceleration, vehicle dynamics



## 🚀 Quick Start

### Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Qt 6.10+
# Download from https://www.qt.io/download-qt-installer

# Dependencies
sudo apt install \
    libcurl4-openssl-dev \
    nlohmann-json3-dev \
    python3-numpy
```

### Installation

```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/eva-autonomous-vehicle.git
cd eva-autonomous-vehicle

# Build ROS2 packages
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Build HMI
cd ../vehicle_hmi
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Launch System

**Terminal 1 - Planning Node:**
```bash
cd ros2_ws
source install/setup.bash
ros2 run eva_planning eva_planning_node
```

**Terminal 2 - Vehicle Simulator:**
```bash
ros2 run vehicle_simulator vehicle_simulator
```

**Terminal 3 - HMI:**
```bash
cd vehicle_hmi/build
./appVehicle_HMI
```

---

## 📖 Usage

1. **Launch all 3 terminals** as shown above
2. **Click on the map** to set a destination
3. **Watch the vehicle** follow the calculated route
4. **Monitor metrics**: Speed, distance, ETA in real-time

![Usage Flow](docs/images/usage_flow.png)

---

## 🗂️ Project Structure

```
eva-autonomous-vehicle/
├── ros2_ws/
│   └── src/
│       ├── eva_planning/          # Trajectory planning
│       │   ├── include/
│       │   │   └── eva_planning/
│       │   │       ├── osrm_client.hpp
│       │   │       ├── coordinate_converter.hpp
│       │   │       └── cubic_spline_planner.hpp
│       │   ├── src/
│       │   │   ├── eva_planning_node.cpp
│       │   │   └── cubic_spline_planner.cpp
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       └── vehicle_simulator/     # Vehicle simulation
│           ├── vehicle_simulator/
│           │   └── vehicle_simulator.py
│           ├── setup.py
│           └── package.xml
│
├── vehicle_hmi/                   # Qt6/QML Interface
│   ├── main.cpp
│   ├── RouteService.{h,cpp}
│   ├── Main.qml
│   ├── VehicleStatusPanel.qml
│   ├── RouteInfoPanel.qml
│   ├── CMakeLists.txt
│   └── resources.qrc
│
├── docs/                          # Documentation
│   ├── images/
│   │   ├── architecture.png
│   │   ├── demo.gif
│   │   └── screenshots/
│   ├── architecture.md
│   └── api.md
│
├── .github/
│   └── workflows/
│       └── ci.yml
│
├── README.md
├── LICENSE
└── .gitignore
```

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Planning Time | 100-300 ms |
| Update Rate | 10 Hz |
| Map Rendering | 60 FPS |
| Memory Usage | ~150 MB |
| CPU Usage | <10% |

<div align="center">
  <img src="docs/images/performance_chart.png" alt="Performance Metrics" width="600"/>
</div>

---

## 🔧 Configuration

### Planning Node Parameters

Edit `ros2_ws/src/eva_planning/config/params.yaml`:

```yaml
eva_planning_node:
  ros__parameters:
    origin_latitude: 33.5731      # Casablanca
    origin_longitude: -7.5898
    osrm_server: "http://router.project-osrm.org"
    max_speed: 15.0               # m/s (~54 km/h)
    lookahead_distance: 10.0      # meters
```

### HMI Configuration

Edit coordinates in `vehicle_hmi/Main.qml`:

```qml
property double originLat: 33.5731
property double originLon: -7.5898
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 Changelog

### v2.0.0 (2025-01-15) - Current
- ✅ Complete HMI integration with Qt6/QML
- ✅ Responsive design
- ✅ Real-time navigation metrics
- ✅ Vehicle simulator with physics
- ✅ Improved OSRM integration
- ✅ Professional automotive-grade interface

### v1.0.0 (2024)
- Initial trajectory planning module
- Basic OSRM integration
- A* global path planning
- Cubic spline local planning

---

## 🐛 Known Issues

- [ ] Offline OSRM server not included (uses public API)
- [ ] No obstacle detection from sensors yet
- [ ] Single vehicle only (no multi-agent)

---

## 🛣️ Roadmap

- [ ] **Local OSRM Server**: Embedded routing engine
- [ ] **Sensor Integration**: LiDAR, Camera obstacle detection
- [ ] **Advanced Planning**: Dynamic obstacles avoidance
- [ ] **Cloud Sync**: Multi-device synchronization
- [ ] **Voice Commands**: Natural language interface
- [ ] **Fleet Management**: Multi-vehicle coordination
- [ ] **Machine Learning**: Predictive route optimization

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Authors

- **Zakaria Jouhari** - *Planning & Integration* - [@Zakariajouhari1](https://github.com/Zakariajouhari1)
- **Mohamed Elhaloua** - *HMI Development* - [@MOHAMEDELHALOUA](https://github.com/MOHAMEDELHALOUA)

---

## 🙏 Acknowledgments

- [ROS2](https://docs.ros.org/) - Robot Operating System
- [OSRM](http://project-osrm.org/) - Open Source Routing Machine
- [Qt](https://www.qt.io/) - Cross-platform framework
- [OpenStreetMap](https://www.openstreetmap.org/) - Map data
- [Previous version](https://github.com/Zakariajouhari1/eva_trajectory_planning) - Foundation for this project

---

<div align="center">

### ⭐ Star this repo if you find it useful!

**Built with ❤️ for autonomous vehicles research**

[Report Bug](https://github.com/YOUR_USERNAME/eva-autonomous-vehicle/issues) · [Request Feature](https://github.com/YOUR_USERNAME/eva-autonomous-vehicle/issues)

</div>

---

## 📚 Related Projects

- [eva_trajectory_planning v1.0](https://github.com/Zakariajouhari1/eva_trajectory_planning) - Original trajectory planning system
- [Vehicle_HMI_QT_QML](https://github.com/MOHAMEDELHALOUA/Vehicle_HMI_QT_QML) - HMI component repository
