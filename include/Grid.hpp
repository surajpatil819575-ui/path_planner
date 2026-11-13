#pragma once
#include <vector>
#include <string>

enum class Cell : char { Free = '.', Obstacle = '#', NoFly = 'X' };

struct Point {
    int x = 0, y = 0;
    bool operator==(const Point& o) const { return x == o.x && y == o.y; }
};

// 2D grid map: obstacles aur no-fly zones dono blocked hain
class Grid {
public:
    Grid(int w, int h) : w_(w), h_(h), cells_(w * h, Cell::Free) {}

    int width() const { return w_; }
    int height() const { return h_; }

    bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    Cell at(int x, int y) const { return cells_[y * w_ + x]; }
    void set(int x, int y, Cell c) { cells_[y * w_ + x] = c; }

    // Rectangle ko no-fly zone bana do (inclusive corners)
    void addNoFlyZone(int x1, int y1, int x2, int y2) {
        for (int y = y1; y <= y2; ++y)
            for (int x = x1; x <= x2; ++x)
                if (inBounds(x, y)) set(x, y, Cell::NoFly);
    }

    bool passable(int x, int y) const {
        return inBounds(x, y) && at(x, y) == Cell::Free;
    }

    // ASCII map: '#' obstacle, 'X' no-fly, '.' free
    static Grid fromStrings(const std::vector<std::string>& rows) {
        Grid g((int)rows[0].size(), (int)rows.size());
        for (int y = 0; y < g.h_; ++y)
            for (int x = 0; x < g.w_; ++x)
                g.set(x, y, static_cast<Cell>(rows[y][x]));
        return g;
    }

private:
    int w_, h_;
    std::vector<Cell> cells_;
};
