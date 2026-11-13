#include "Dijkstra.hpp"
#include "Common.hpp"
#include <queue>
#include <functional>
#include <limits>
#include <chrono>

PlanResult DijkstraPlanner::plan(const Grid& g, Point start, Point goal) const {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    PlanResult res;

    const int W = g.width(), H = g.height();
    if (!g.passable(start.x, start.y) || !g.passable(goal.x, goal.y)) return res;

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(W * H, INF);
    std::vector<int> parent(W * H, -1);
    std::vector<bool> closed(W * H, false);

    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

    auto idx = [W](int x, int y) { return y * W + x; };
    const auto mv = detail::moves(cfg_.allowDiagonal);

    dist[idx(start.x, start.y)] = 0.0;
    pq.push({0.0, idx(start.x, start.y)});

    while (!pq.empty()) {
        int cur = pq.top().second;
        pq.pop();
        if (closed[cur]) continue;
        closed[cur] = true;
        ++res.nodesExpanded;

        int cx = cur % W, cy = cur / W;
        if (cx == goal.x && cy == goal.y) {   // goal pe early exit
            res.found = true;
            res.cost = dist[cur];
            res.path = detail::reconstruct(parent, W, cur);
            break;
        }
        for (const auto& m : mv) {
            int nx = cx + m.dx, ny = cy + m.dy;
            if (!g.passable(nx, ny) || !detail::diagonalOk(g, cx, cy, m)) continue;
            int ni = idx(nx, ny);
            double nd = dist[cur] + m.cost;
            if (nd < dist[ni]) {
                dist[ni] = nd;
                parent[ni] = cur;
                pq.push({nd, ni});
            }
        }
    }
    (void)H;
    res.runtimeMs = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    return res;
}
