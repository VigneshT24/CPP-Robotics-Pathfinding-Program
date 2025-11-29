![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

# Robotics Pathfinding Program

A C++ simulation program that demonstrates autonomous robotic navigation through obstacle-laden environments using heuristic-enhanced Depth-First Search (DFS) algorithms.

## Overview

This program simulates a robot navigating from a starting position (0,0) to a goal position (4,4) on a 5×5 grid containing randomly generated obstacles. The robot employs intelligent pathfinding strategies to avoid obstacles and reach its destination.

## Key Features
1. Heuristic-Enhanced DFS: Combines Depth-First Search with 4-Way Directional Vision (4DV) for improved pathfinding
2. Obstacle Detection System: Active Robot Modern Scan (ARMS) pre-validates grid viability before navigation
3. Adaptive Strategy Switching: Automatically attempts alternative paths when primary route fails
4. Real-time Visualization: Color-coded terminal display with animated scanning and pathfinding
5. Performance Analytics: Detailed statistics including moves, sensor usage, and path efficiency

## Technical Implementations

* Core Components
    * Robot Class (robotObject.hpp): Encapsulates robot properties and identification
    * Pathfinding Algorithm: Two-pass DFS with configurable movement priorities
    * Obstacle Avoidance: Multi-directional sensor system (up, down, left, right)
    * Path Validation: Pre-simulation scanning to eliminate unsolvable configurations

* Navigation Strategies
    1. Primary Strategy: DOWN → RIGHT priority
    2. Secondary Strategy: RIGHT → DOWN priority
    3. 4DV Lookahead: Checks diagonal and adjacent cells for optimal routing
 
## Requirements

* C++11 or higher
* ANSI terminal support for colored output
* Standard libraries: iostream, vector, thread, chrono
 
## Installation/Usage

> Clone the Repository

```bash
git clone https://github.com/VigneshT24/CPP-Robotics-Pathfinding-Program.git
cd CPP-Robotics-Pathfinding-Program
```

> Compile & Run

```bash
g++ -std=c++11 main.cpp -o robot_sim
./robot_sim
# or click the run button if shown
```

> User Input

| Parameter | Description | Options/Format |
|-----------|-------------|----------------|
| **Robot Name** | Single character identifier | Any character except `0` or `O` |
| **Difficulty Level** | Controls obstacle density | `5` - Fewer obstacles (super high success rate)<br>`3` - Moderate obstacles (good success rate)<br>`2` - More obstacles (slightly lower success rate) |
| **Robot Type** | Descriptive classification | Any string value |

> Program Output

The program provides:

* Animated grid visualization with color-coded elements
* Real-time pathfinding progression
* Comprehensive statistics including:
  * Total moves counter
  * Sensor usage frequency
  * Path efficiency metrics
  * Visual robot footprint trace
 
**This simulation is developed as a demonstration of algorithmic pathfinding and robotic simulation techniques.**
