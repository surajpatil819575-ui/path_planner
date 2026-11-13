#include <gtest/gtest.h>
#include "AStar.hpp"
#include "Dijkstra.hpp"
#include <cmath>
#include <memory>

namespace {
Grid openGrid(int n) { return Grid(n, n); }
}

TEST(Planner, StraightLine) {
    Grid g = openGrid(5);
    std::unique_ptr<IPlanner> planners[] = {
        std::make_unique<AStarPlanner>(), std::make_unique<DijkstraPlanner>()};
    for (auto& p : planners) {
        auto r = p->plan(g, {0, 0}, {4, 0});
        EXPECT_TRUE(r.found);
        EXPECT_DOUBLE_EQ(r.cost, 4.0);
        EXPECT_EQ(r.path.size(), 5u);
    }
}

TEST(Planner, StartEqualsGoal) {
    Grid g = openGrid(3);
    auto r = AStarPlanner{}.plan(g, {1, 1}, {1, 1});
    EXPECT_TRUE(r.found);
    EXPECT_DOUBLE_EQ(r.cost, 0.0);
    EXPECT_EQ(r.path.size(), 1u);
}

TEST(Planner, BlockedGoal) {
    Grid g = openGrid(4);
    g.set(3, 3, Cell::Obstacle);
    EXPECT_FALSE(AStarPlanner{}.plan(g, {0, 0}, {3, 3}).found);
    EXPECT_FALSE(DijkstraPlanner{}.plan(g, {0, 0}, {3, 3}).found);
}

TEST(Planner, UnreachableGoalWalledOff) {
    Grid g = Grid::fromStrings({"..#..", "..#..", "..#.."});
    EXPECT_FALSE(AStarPlanner{}.plan(g, {0, 0}, {4, 0}).found);
    EXPECT_FALSE(DijkstraPlanner{}.plan(g, {0, 0}, {4, 0}).found);
}

TEST(Planner, NoFlyZoneIsAvoided) {
    Grid g = openGrid(5);
    g.addNoFlyZone(2, 0, 2, 3);          // sirf y=4 pe gap hai
    auto r = AStarPlanner{}.plan(g, {0, 0}, {4, 0});
    ASSERT_TRUE(r.found);
    for (auto& p : r.path) EXPECT_NE(g.at(p.x, p.y), Cell::NoFly);
    EXPECT_DOUBLE_EQ(r.cost, 12.0);
}

TEST(Planner, AStarMatchesDijkstraCost) {
    Grid g = Grid::fromStrings({
        "........",
        ".####.#.",
        ".#..#.#.",
        ".#.##.#.",
        ".#....#.",
        ".######.",
        "........",
    });
    auto a = AStarPlanner{}.plan(g, {0, 0}, {7, 6});
    auto d = DijkstraPlanner{}.plan(g, {0, 0}, {7, 6});
    ASSERT_TRUE(a.found && d.found);
    EXPECT_DOUBLE_EQ(a.cost, d.cost);       // dono optimal hone chahiye
    EXPECT_LE(a.nodesExpanded, d.nodesExpanded);
}

TEST(Planner, DiagonalNoCornerCutting) {
    Grid g = Grid::fromStrings({".#", "#."});
    PlannerConfig cfg{true, Heuristic::Euclidean};
    EXPECT_FALSE(AStarPlanner{cfg}.plan(g, {0, 0}, {1, 1}).found);
}

TEST(Planner, EuclideanDiagonalCost) {
    Grid g = openGrid(4);
    PlannerConfig cfg{true, Heuristic::Euclidean};
    auto r = AStarPlanner{cfg}.plan(g, {0, 0}, {3, 3});
    ASSERT_TRUE(r.found);
    EXPECT_NEAR(r.cost, 3 * std::sqrt(2.0), 1e-9);
}
