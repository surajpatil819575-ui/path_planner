#pragma once
#include "Planner.hpp"
#include <cmath>
#include <cstdlib>
#include <vector>
#include <algorithm>

namespace detail {

struct Move { int dx, dy; double cost; };

inline std::vector<Move> moves(bool diagonal) {
    std::vector<Move> m = {{1,0,1.0},{-1,0,1.0},{0,1,1.0},{0,-1,1.0}};
    if (diagonal) {
        const double d = std::sqrt(2.0);
        m.push_back({1,1,d}); m.push_back({1,-1,d});
        m.push_back({-1,1,d}); m.push_back({-1,-1,d});
    }
    return m;
}

inline double heuristic(Heuristic h, Point a, Point b) {
    double dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    if (h == Heuristic::Manhattan) return dx + dy;
    return std::sqrt(dx * dx + dy * dy);
}

// Diagonal move tabhi allow jab dono side ke cells free ho (corner cutting nahi)
inline bool diagonalOk(const Grid& g, int x, int y, const Move& m) {
    if (m.dx == 0 || m.dy == 0) return true;
    return g.passable(x + m.dx, y) && g.passable(x, y + m.dy);
}

inline std::vector<Point> reconstruct(const std::vector<int>& parent, int w, int goalIdx) {
    std::vector<Point> path;
    for (int i = goalIdx; i != -1; i = parent[i]) path.push_back({i % w, i / w});
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace detail
