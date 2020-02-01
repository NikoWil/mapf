//
// Created by khondar on 01.02.20.
//

#include "pathfinding.h"

#include <algorithm>
#include <queue>

SpacePoint::SpacePoint(SpaceTimePoint p) {
    this->x = p.x;
    this->y = p.y;
}

SpacePoint::SpacePoint(int32_t x, int32_t y) {
    this->x = x;
    this->y = y;
}

bool SpacePoint::operator==(const SpacePoint other) const noexcept {
    return this->x == other.x
           && this->y == other.y;
}

SpaceTimePoint::SpaceTimePoint(SpacePoint p, int32_t t) {
    this->x = p.x;
    this->y = p.y;
    this->t = t;
}

SpaceTimePoint::SpaceTimePoint(int32_t x, int32_t y, int32_t t) {
    this->x = x;
    this->y = y;
    this->t = t;
}

std::ostream &operator<<(std::ostream &os, SpaceTimePoint p) {
    os << "x: " << p.x << ", y: " << p.y << ", t: " << p.t;
    return os;
}

bool SpaceTimePoint::operator==(const SpaceTimePoint &other) const {
    return this->x == other.x
           && this->y == other.y
           && this->t == other.t;
}

std::vector<SpaceTimePoint> get_neighbours(SpaceTimePoint p, int32_t width, int32_t height,
                                           const std::unordered_set<SpaceTimePoint> &reservations) {
    std::vector<SpaceTimePoint> neighbours;
    neighbours.emplace_back(p.x, p.y, p.t + 1);
    if (p.x > 0) {
        neighbours.emplace_back(p.x - 1, p.y, p.t + 1);
    }
    if (p.x < width - 1) {
        neighbours.emplace_back(p.x + 1, p.y, p.t + 1);
    }
    if (p.y > 0) {
        neighbours.emplace_back(p.x, p.y - 1, p.t + 1);
    }
    if (p.y < height - 1) {
        neighbours.emplace_back(p.x, p.y + 1, p.t + 1);
    }

    std::vector<SpaceTimePoint> valid_neighbours;
    for (const auto n : neighbours) {
        if (reservations.empty()
            || (reservations.find(n) == reservations.end()
                && reservations.find(SpaceTimePoint(n.x, n.y, n.t - 1)) ==
                   reservations.end()  // we must not train someone else
                && reservations.find(SpaceTimePoint(n.x, n.y, n.t + 1)) ==
                   reservations.end())) // we must not force someone else into training us
        {
            valid_neighbours.push_back(n);
        }
    }
    return valid_neighbours;
}

std::vector<SpaceTimePoint>
reconstruct_path(const std::unordered_map<SpaceTimePoint, SpaceTimePoint> &came_from, SpaceTimePoint goal) {
    std::vector<SpaceTimePoint> path;
    path.reserve(goal.t + 1);
    SpaceTimePoint curr = goal;
    path.push_back(curr);
    while (came_from.find(curr) != came_from.end()) {
        curr = came_from.at(curr);
        path.push_back(curr);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<SpaceTimePoint> a_star(const SpacePoint start, const SpacePoint goal, uint32_t width, uint32_t height,
                                   const std::unordered_set<SpaceTimePoint> &reservations) { // heuristic is always manhatten distance
    const auto cmp = [goal](SpaceTimePoint p1, SpaceTimePoint p2) {
        // in f(p) = g(p) + h(p), manhatten distance is the heuristic and g(p) = p.t
        const auto d1 = manhatten_distance(p1, goal) + p1.t;
        const auto d2 = manhatten_distance(p2, goal) + p2.t;
        return d1 > d2 || (d1 == d2 && p1.x < p2.x); // direct comparison of x, y is used to break ties
    };
    std::priority_queue<SpaceTimePoint, std::vector<SpaceTimePoint>, decltype(cmp)> open_set(cmp);
    open_set.push(SpaceTimePoint(start));

    std::unordered_map<SpaceTimePoint, SpaceTimePoint> came_from{};

    while (!open_set.empty()) {
        const auto curr = open_set.top();
        open_set.pop();

        if (SpacePoint(curr) == goal) {
            const auto path = reconstruct_path(came_from, curr); // use curr to ensure we know the time
            return path;
        }

        const auto valid_neighbours = get_neighbours(curr, width, height, reservations);
        for (const auto n : valid_neighbours) {
            // Normally we check the cost so far, our cost so far is always the same. So we check came_from instead,
            // as any seen (even not explored) node has an entry
            if (came_from.empty() || came_from.find(n) == came_from.end()) {
                came_from.insert(std::make_pair(n, curr));
                open_set.push(n);
            }
        }
    }

    const std::vector<SpaceTimePoint> path;
    return path;
}