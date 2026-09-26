![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

# Robotics Pathfinding Program

A C++ terminal simulation that shows a robot navigating through a grid filled with randomly generated obstacles.

This repository contains **two versions** of the project:

- **Modern version** - uses the **A\*** pathfinding algorithm and an FTXUI-based terminal interface.
- **Legacy version** - keeps the original DFS-style pathfinding approach for comparison and historical interest.

The legacy version is intentionally kept in the repository so you can run both versions and see how the project changed over time.

---

## Modern vs. Legacy

| Feature | Modern Version | Legacy Version |
|---|---|---|
| Pathfinding | A* | DFS-style heuristic search |
| Finds shortest path | Yes, for the current four-direction grid with equal movement cost | Not guaranteed |
| Grid size | Configurable | Fixed 5×5 design |
| Goal location | Configurable | Fixed bottom right |
| Interface | FTXUI terminal UI | ANSI terminal output |
| Path trail | Yes | Yes |
| Live statistics | `g(n)`, `h(n)`, `f(n)`, direction counts | Move/sensor statistics |
| Code structure | Pathfinding and display logic are more clearly separated | Navigation, display, and simulation logic are more tightly connected |
| Build system | CMake | Direct `g++` compile |

### Why the modern version is better

The modern version uses A* to choose the most promising next position while still keeping track of how far the robot has already traveled.

It uses:

- **g(n)** - how many steps the robot has taken from the start
- **h(n)** - an estimate of how far the robot is from the goal
- **f(n)** - the total score: `g(n) + h(n)`

Because movement is limited to up, down, left, and right and every move has the same cost, the modern version can find a shortest valid path when one exists.

The newer code is also easier to read and extend because creating the grid, finding a path, rendering the grid, and animating the robot are handled separately.

The legacy version is still useful if you want to compare the older approach, see how the project originally worked, or study how the code changed during the refactor.

---

## Modern Version

### Features

- A* pathfinding
- Four-direction movement: up, down, left, right
- Custom grid creation
- Random obstacle generation
- Configurable grid size
- Configurable goal location
- Configurable difficulty
- FTXUI terminal interface
- Animated robot movement
- Visible path trail
- Goal displayed as `X`
- Live simulation statistics:
  - Steps from start
  - Estimated distance to goal
  - Total A* cost
  - Up/down/left/right move counts
- `Q` to quit the simulation

### Difficulty

The modern version uses a difficulty value based on obstacle probability.

A higher difficulty means **more obstacles**.

For example:

- `difficulty = 2` → about 20% obstacle chance
- `difficulty = 5` → about 50% obstacle chance
- `difficulty = 8` → about 80% obstacle chance

Very high difficulty values can easily create a grid with no valid path.

---

## Legacy Version

The legacy version preserves the original project behavior.

It uses a DFS-style navigation system with fixed movement priorities and additional obstacle checks. It may successfully find a route, but it is not designed to guarantee the shortest path.

The legacy version also keeps the original 5×5 simulation style, directional sensor statistics, and terminal animations.

---

## Requirements

### Modern version

You will need:

- A C++ compiler
- CMake
- Git
- A terminal that supports ANSI colors

The modern version uses **FTXUI** for its terminal interface. The project CMake configuration handles the FTXUI dependency during configuration.

### Legacy version

You will need:

- A C++ compiler such as `g++`
- A terminal that supports ANSI colors

---

## Clone the Repository

```bash
git clone https://github.com/VigneshT24/CPP-Robotics-Pathfinding-Program.git
cd CPP-Robotics-Pathfinding-Program
```

---

# Running the Modern Version

The modern version uses CMake.

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
./modern_pathfinder_main
```

If the `build` folder already exists, you can simply enter it:

```bash
cd build
cmake ..
cmake --build .
./modern_pathfinder_main
```

### After changing modern code

If you edit the C++ source, **compile again before running**:

```bash
cmake --build .
./modern_pathfinder_main
```

If you also change `CMakeLists.txt`, run the configure step again:

```bash
cmake ..
cmake --build .
./modern_pathfinder_main
```

Do not expect source-code changes to appear if you only run the old executable without rebuilding it.

---

# Running the Legacy Version

The legacy version does not require CMake.

From the repository root:

```bash
g++ legacy_pathfinder_main.cpp -o legacy_pathfinder
./legacy_pathfinder
```

You can choose a different executable name if you want:

```bash
g++ legacy_pathfinder_main.cpp -o my_pathfinder
./my_pathfinder
```

### After changing legacy code

You must compile it again before running it:

```bash
g++ legacy_pathfinder_main.cpp -o legacy_pathfinder
./legacy_pathfinder
```

Running the previous executable without recompiling will run the previous compiled version of the program.

---

## Suggested Project Layout

```text
CPP-Robotics-Pathfinding-Program/
├── modern_pathfinder_main.cpp
├── legacy_pathfinder_main.cpp
├── robotObject.hpp
├── CMakeLists.txt
├── README.md
└── build/                  # generated locally, normally ignored by Git
```

The `build/` directory contains generated CMake files and compiled output, so it should not normally be committed to Git.

---

## How A* Works in This Project

The modern version stores positions that it may visit and gives each one a cost:

```text
f(n) = g(n) + h(n)
```

Where:

- `g(n)` is the number of steps already taken
- `h(n)` is the Manhattan-distance estimate to the goal
- `f(n)` is the combined score

The algorithm repeatedly checks the position with the lowest total cost, looks at its valid neighbors, and remembers the best route found to each cell.

After reaching the goal, it retraces the saved positions to build the final path. The simulation then animates the robot along that path.

---

## Grid Symbols (Modern Version)

| Symbol | Meaning |
|---|---|
| `R` | Robot |
| `0` | Obstacle |
| `X` | Goal |
| `*` | Path already traveled |
| blank cell | Open space |

---

## Notes

- Grids can be custom created or randomly generated, depending on which option is chosen at the start of the simulation.
- Some generated grids may have no valid path.
- Increasing difficulty increases obstacle density.
- Larger grids may require a larger terminal window for the best display.
- The modern simulation uses `Q` to exit.
- The legacy and modern versions are separate programs, so changes to one do not automatically affect the other.

---

## Purpose

This project started as a small robotic pathfinding simulation and was later refactored to use A* and a cleaner terminal interface.

Keeping both versions makes it possible to compare the original design with the newer implementation and see how the pathfinding logic, code structure, and visualization improved over a span of two years.

## Notice

This project is "as-is" and the author is not responsible for any damages or issues causes by the use of any content in this repository.
