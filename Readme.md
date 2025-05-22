# 🚘🤖 Unity Multi-Agent Navigation — Real-Time Path Planning & Collision Avoidance

This is a project from the course DD2438 at KTH. It focuses on **multi-agent navigation** in complex environments using both **cars and drones**, each assigned a unique goal. The objective is to minimize the time until **all agents reach their goals** without collisions. The maps simulate real-world traffic scenarios involving shared workspaces, bottlenecks, and high-density movement.

This simulation was built in Unity and implements custom real-time path planning and collision avoidance strategies for both ground and aerial vehicles.

---

## 🌐 Problem Overview

Each agent (car or drone) must independently reach a goal while avoiding collisions with:
- Static obstacles (depending on the map),
- Other dynamic agents (cars/drones) with their own goals.

Maps simulate various urban traffic scenarios:
- `terrain_open`: no obstacles, agents can collide from any direction
- `terrain_open_blocks`: open terrain with static obstacles
- `terrain_intersection`: classic four-way intersection
- `terrain_highway`: agents must merge through a bottleneck
- `terrain_onramp`: like a highway with diagonal entries

---

## 🔧 Implementation Details

### 🗺 Path Planning

- **Base Algorithm**: A* search (from Assignment 1), adapted to:
  - Use cell-center coordinates as path nodes, ensuring uniform reference points
  - Penalize proximity to obstacles
  - Penalize having obstacles on the **left**, encouraging "keep-right" behavior

- **Traffic Optimization**:
  - Introduced **directional bonuses**:  
    - Bonus for following the same path in the same direction  
    - Penalty for using the same path in the opposite direction  
  - Result: organically enforced right-hand traffic and reduced head-on collisions

- **Path Storage**:
  - All agent paths are stored in a global dictionary
  - Enables real-time decision-making and path influence among agents

---

### 🤝 Collision Avoidance

- **Cars**:
  - Constantly analyze their next few path segments
  - If a collision is predicted:
    - If following another vehicle in the same direction, the rear vehicle slows/stops
    - If paths intersect, the vehicle with the higher index (assigned at planning time) yields
  - Uses raycasts in the driving direction to detect other agents and react accordingly

- **Drones**:
  - 360° raycasting system detects agents and obstacles in all directions
  - If something is within a close radius, the drone steers away automatically
  - Forward raycasts help maintain distance by stopping the drone when necessary
  - Critical since drone collisions reduce their acceleration in simulation

---

## 🧪 How to Run

1. Open the project in **Unity**.
2. Use the `MapManager` object to load a map from `/Assets/StreamingAssets/Text/Maps/`.
3. Press Play — each agent will plan and execute a collision-free path to its goal.
4. Goals turn blue when successfully reached.

---

## 🛠 Technologies Used

- Unity3D (C#)
- A* Pathfinding with dynamic penalties
- Raycasting for real-time sensing
- Global coordination through path dictionaries
- Custom traffic-like agent behavior

---

## 🧠 Learning Highlights

- Real-time multi-agent decision-making
- Autonomous vehicle behavior modeling
- Practical tradeoffs in pathfinding vs. collision detection
- Adaptive control logic based on simulation constraints

---


## 👤 Authors

- Chloé Leyssens  
- Josef Afreim
