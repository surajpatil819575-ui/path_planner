#pragma once
#include "Planner.hpp"

class AStarPlanner : public IPlanner {
public:
    explicit AStarPlanner(PlannerConfig cfg = {}) : cfg_(cfg) {}
    PlanResult plan(const Grid& grid, Point start, Point goal) const override;
    std::string name() const override { return "A*"; }
private:
    PlannerConfig cfg_;
};
