#pragma once
#include "Grid.hpp"
#include <vector>
#include <string>

struct PlanResult {
    bool found = false;
    std::vector<Point> path;   // start se goal tak
    double cost = 0.0;
    long nodesExpanded = 0;
    double runtimeMs = 0.0;
};

enum class Heuristic { Manhattan, Euclidean };

struct PlannerConfig {
    bool allowDiagonal = false;          // false => 4-connected, true => 8-connected
    Heuristic heuristic = Heuristic::Manhattan;
};

// Common interface: A* aur Dijkstra dono isko implement karte hain
class IPlanner {
public:
    virtual ~IPlanner() = default;
    virtual PlanResult plan(const Grid& grid, Point start, Point goal) const = 0;
    virtual std::string name() const = 0;
};
