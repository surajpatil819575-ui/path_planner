# Grid-Based Path Planner with No-Fly Zones

A* and Dijkstra on a 2D grid (obstacles + no-fly zones), common `IPlanner` interface,
Manhattan/Euclidean heuristics, optional 8-connected moves.

## Features
- A* and Dijkstra planners behind a common `IPlanner` interface
- Obstacles and no-fly zones on a 2D grid
- Manhattan and Euclidean heuristics
- Optional 8-connected movement
- Unit tests via CTest

## Algorithms
| Algorithm | Optimal | Heuristic | Notes |
|-----------|---------|-----------|-------|
| Dijkstra | Yes | None | Explores uniformly |
| A* | Yes (admissible heuristic) | Manhattan / Euclidean | Fewer nodes expanded |

## Project Structure
```
path_planner/
├── include/        # headers (Planner.hpp, AStar.hpp, Dijkstra.hpp)
├── src/            # AStar.cpp, Dijkstra.cpp, main.cpp
├── tests/          # unit tests
├── CMakeLists.txt
└── README.md
```

## Requirements
- CMake 3.15+
- C++17 compiler (GCC, Clang, or MSVC)

## Build & run (Linux / macOS, single-config generators)
```bash
cmake -S . -B build
cmake --build build
./build/planner_demo
ctest --test-dir build --output-on-failure
```

## Build & run (Windows / Visual Studio)
```bash
cmake -S . -B build
cmake --build build --config Release
.\build\Release\planner_demo.exe
ctest --test-dir build -C Release --output-on-failure
```

## Usage
```cpp
// Example: swap planners through the common interface
std::unique_ptr<IPlanner> planner = std::make_unique<AStar>();
auto path = planner->plan(grid, start, goal);
```

