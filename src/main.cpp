#include "AStar.hpp"
#include "Dijkstra.hpp"
#include <iostream>
#include <iomanip>
#include <memory>
#include <random>

static void printMap(const Grid& g, const std::vector<Point>& path) {
    std::vector<std::string> rows(g.height(), std::string(g.width(), '.'));
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            rows[y][x] = static_cast<char>(g.at(x, y));
    for (const auto& p : path) rows[p.y][p.x] = '*';
    if (!path.empty()) { rows[path.front().y][path.front().x] = 'S'; rows[path.back().y][path.back().x] = 'G'; }
    for (auto& r : rows) std::cout << r << '\n';
}

static Grid randomMap(int n, double density, unsigned seed) {
    Grid g(n, n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> d(0, 1);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x)
            if (d(rng) < density) g.set(x, y, Cell::Obstacle);
    g.addNoFlyZone(n / 3, n / 4, n / 3 + n / 8, 3 * n / 4);   // beech mein no-fly zone
    g.set(0, 0, Cell::Free); g.set(n - 1, n - 1, Cell::Free);
    return g;
}

int main() {
    // --- Demo map ---
    Grid demo = Grid::fromStrings({
        "..........",
        "..####....",
        "..#XX#....",
        "..#XX#....",
        "..####....",
        "..........",
    });
    AStarPlanner astar({false, Heuristic::Manhattan});
    auto r = astar.plan(demo, {0, 3}, {9, 3});
    std::cout << "A* demo path (cost " << r.cost << "):\n";
    printMap(demo, r.path);

    // --- Benchmark: A* vs Dijkstra ---
    std::cout << "\n" << std::left << std::setw(8) << "Size" << std::setw(12) << "Planner"
              << std::setw(10) << "Cost" << std::setw(12) << "Expanded" << "Time(ms)\n";
    for (int n : {50, 100, 200, 400}) {
        // Aisa random map lo jisme path exist kare (warna comparison meaningless hai)
        unsigned seed = 42;
        Grid g = randomMap(n, 0.2, seed);
        while (!AStarPlanner{}.plan(g, {0, 0}, {n - 1, n - 1}).found)
            g = randomMap(n, 0.2, ++seed);
        std::vector<std::unique_ptr<IPlanner>> planners;
        planners.emplace_back(new AStarPlanner({false, Heuristic::Manhattan}));
        planners.emplace_back(new DijkstraPlanner({false, Heuristic::Manhattan}));
        for (auto& p : planners) {
            auto res = p->plan(g, {0, 0}, {n - 1, n - 1});
            std::cout << std::setw(8) << n << std::setw(12) << p->name()
                      << std::setw(10) << (res.found ? res.cost : -1)
                      << std::setw(12) << res.nodesExpanded
                      << std::fixed << std::setprecision(3) << res.runtimeMs << "\n";
            std::cout.unsetf(std::ios::floatfield);
        }
    }
    return 0;
}
