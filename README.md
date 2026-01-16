# 🚗 EVA - Electric Vehicle Assistant

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Qt](https://img.shields.io/badge/Qt-6.10-green)](https://www.qt.io/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-17-orange)](https://en.cppreference.com/)

**EVA** (Electric Vehicle Assistant) is a complete autonomous navigation system combining a ROS2-based trajectory planning module with a professional Qt6/QML HMI interface.

![EVA Demo](docs/images/eva_demo.png)

## ✨ Features

### 🗺️ Trajectory Planning (ROS2)
- **OSRM Integration**: Real-world road routing using OpenStreetMap
- **Cubic Spline Smoothing**: Local trajectory refinement
- **GPS ↔ Local Conversion**: Accurate coordinate transformation
- **Real-time Planning**: ~100-300ms computation time

### 🖥️ Professional HMI (Qt6/QML)
- **Interactive Map**: OpenStreetMap with route visualization
- **Real-time Dashboard**: Speed, battery, vehicle status
- **Responsive Design**: Adapts to any screen size
- **Live Navigation**: Distance, ETA, progress tracking

### 🚙 Vehicle Simulator
- **Physics-based**: Realistic acceleration and movement
- **Path Following**: Automatic trajectory tracking
- **Odometry Publishing**: Real-time position updates

## 📦 Architecture

```
┌─────────────────────┐
│   EVA Planning      │
│   (ROS2 + OSRM)     │
└──────────┬──────────┘
           │ ROS2 Topics
           ├──────────────────────┐
           │                      │
┌──────────▼──────────┐  ┌───────▼────────┐
│  Vehicle HMI        │  │   Simulator    │
│  (Qt6 + QML)        │  │   (Python)     │
└─────────────────────┘  └────────────────┘
```

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

## 📖 Usage

1. **Launch all 3 terminals** as shown above
2. **Click on the map** to set a destination
3. **Watch the vehicle** follow the calculated route
4. **Monitor metrics**: Speed, distance, ETA in real-time

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

## 📊 Performance

| Metric | Value |
|--------|-------|
| Planning Time | 100-300 ms |
| Update Rate | 10 Hz |
| Map Rendering | 60 FPS |
| Memory Usage | ~150 MB |
| CPU Usage | <10% |

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 Changelog

### v2.0.0 (2025-01-15) - Current
- ✅ Complete HMI integration with Qt6/QML
- ✅ Responsive design
- ✅ Real-time navigation metrics
- ✅ Vehicle simulator with physics
- ✅ Improved OSRM integration

### v1.0.0 (2024)
- Initial trajectory planning module
- Basic OSRM integration

## 🐛 Known Issues

- [ ] Offline OSRM server not included (uses public API)
- [ ] No obstacle detection from sensors yet
- [ ] Single vehicle only (no multi-agent)

## 🛣️ Roadmap

- [ ] **Local OSRM Server**: Embedded routing engine
- [ ] **Sensor Integration**: LiDAR, Camera obstacle detection
- [ ] **Advanced Planning**: Dynamic obstacles avoidance
- [ ] **Cloud Sync**: Multi-device synchronization
- [ ] **Voice Commands**: Natural language interface
- [ ] **Fleet Management**: Multi-vehicle coordination

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- Zakaria JOuhari
- Mohamed Elhaloua

## 🙏 Acknowledgments

- [ROS2](https://docs.ros.org/) - Robot Operating System
- [OSRM](http://project-osrm.org/) - Open Source Routing Machine
- [Qt](https://www.qt.io/) - Cross-platform framework
- [OpenStreetMap](https://www.openstreetmap.org/) - Map data



---

⭐ **Star this repo** if you find it useful!

**Previous Version**: [eva_trajectory_planning v1.0](https://github.com/Zakariajouhari1/eva_trajectory_planning)
