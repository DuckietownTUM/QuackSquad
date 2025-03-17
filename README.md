# 🦆 Welcome to our Duckietown!

> _Because ducks deserve smart traffic, too._

Welcome to our **Duckietown**, a university project where we teach our robotic friends how to navigate urban traffic without causing any quacking chaos. 🚗🤖🦆

---

## 🐥 Who are we?

We're an enthusiastic team of students and robotics lovers committed to bringing order (and a bit of humor) to the busy streets of Duckietown. Our goal is to implement innovative solutions for autonomous navigation.

**Meet the team:**

| Role                      | Name                               | GitHub                                                   |
|---------------------------|------------------------------------|----------------------------------------------------------|
| ⚙️ **Robotics Developer**  | Julien-Alexandre Bertin Klein 🇫🇷   | [@Zen-Lex](https://github.com/Zen-Lex)                   |
| 🔗 **Systems Integrator**  | Andrea Pellegrin 🇮🇹                | [@andreapellegrin](https://github.com/andreapellegrin)   |
| 🧠 **Algorithm Engineer**  | Fathia Ismail 🇹🇳🇪🇬                 | [@fathia156](https://github.com/fathia156)               |

---

## 🚦 What are we working on?

Our project focuses on:

- 🗺️ **Autonomous Navigation**: Implementing algorithms to help Duckiebots move autonomously and safely through the streets.
- 📷 **Computer Vision**: Enhancing the visual perception of our robots.

---

## 📂 Repository Structure

Here's how we've organized our resources:
```
duckietown/
├── calibrations/                      # Calibration files for camera and wheels
├── containers/                        # Docker-based containers for running Duckietown software
│   ├── dt-car-interface/                # Container for managing car-related modules
│   │   └── packages/                    # Software packages for car control and interaction
│   │       ├── dagu_car/                   # Low-level control of the Duckiebot's motors
│   │       └── joy_mapper/                 # Mapping joystick inputs to robot actions
│   └── dt-core/                       # Core Duckietown software components
│       ├── launchers/                   # Launch files to start robot behaviors
│       └── packages/                    # Core modules managing robot behavior and perception
│           ├── apriltag/                   # Detection and decoding of AprilTags
│           ├── deadreckoning/              # Estimation of the robot's position using odometry
│           ├── dijkstra/                   # Custom implementation of Dijkstra's path-planning algorithm
│           ├── duckietown_demos/           # Launch files for core Duckietown functionalities
│           ├── fsm/                        # Finite State Machine logic for managing robot states
│           ├── lane_control/               # Lane-following and intersection navigation control algorithms
│           ├── led_emitter/                # Control of the Duckiebot's LED signals
│           ├── stop_line_filter/           # Detection of stop lines in the environment
│           ├── test_package/               # Testing utilities and experimental code
│           └── vehicle_detection/          # Algorithms for detecting other vehicles in the environment
├── duckie-web/                        # Custom web-based interfaces for monitoring and controlling Duckiebots
└── resources/                         # Additional assets such as images
```

---

## 🚀 What’s Next?

This project has reached a solid and functional state. While no further updates are planned at the moment, improvements may come in the future if time and opportunity allow.

---

🌟 **Happy Navigation! Quack!** 🦆🚗✨