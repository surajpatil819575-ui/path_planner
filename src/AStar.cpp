#include "AStar.hpp"
#include "Common.hpp"
#include <queue>
#include <functional>
#include <limits>
#include <chrono>

PlanResult AStarPlanner::plan(const Grid& g, Point start, Point goal) const {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    PlanResult res;

    const int W = g.width(), H = g.height();
    if (!g.passable(start.x, start.y) || !g.passable(goal.x, goal.y)) return res;

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gCost(W * H, INF);
    std::vector<int> parent(W * H, -1);
    std::vector<bool> closed(W * H, false);

    // (f, index) min-heap
    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

    auto idx = [W](int x, int y) { return y * W + x; };
    const auto mv = detail::moves(cfg_.allowDiagonal);

    gCost[idx(start.x, start.y)] = 0.0;
    open.push({detail::heuristic(cfg_.heuristic, start, goal), idx(start.x, start.y)});

    while (!open.empty()) {
        int cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;       // stale entry (lazy deletion)
        closed[cur] = true;
        ++res.nodesExpanded;

        int cx = cur % W, cy = cur / W;
        if (cx == goal.x && cy == goal.y) {
            res.found = true;
            res.cost = gCost[cur];
            res.path = detail::reconstruct(parent, W, cur);
            break;
        }
        for (const auto& m : mv) {
            int nx = cx + m.dx, ny = cy + m.dy;
            if (!g.passable(nx, ny) || !detail::diagonalOk(g, cx, cy, m)) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;
            double ng = gCost[cur] + m.cost;
            if (ng < gCost[ni]) {
                gCost[ni] = ng;
                parent[ni] = cur;
                open.push({ng + detail::heuristic(cfg_.heuristic, {nx, ny}, goal), ni});
            }
        }
    }
    (void)H;
    res.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    return res;
}
