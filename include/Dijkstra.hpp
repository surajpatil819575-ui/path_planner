#pragma once
#include "Planner.hpp"

class DijkstraPlanner : public IPlanner {
public:
    explicit DijkstraPlanner(PlannerConfig cfg = {}) : cfg_(cfg) {}
    PlanResult plan(const Grid& grid, Point start, Point goal) const override;
    std::string name() const override { return "Dijkstra"; }
private:
    PlannerConfig cfg_;
};
