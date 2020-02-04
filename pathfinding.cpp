//
// Created by khondar on 01.02.20.
//

#include "pathfinding.h"

#include <algorithm>
#include <queue>
#include <iostream>

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

bool SpacePoint::operator!=(const SpacePoint other) const noexcept {
    return this->x != other.x || this->y != other.y;
}

std::ostream &operator<<(std::ostream &os, const SpacePoint p) {
    os << "(x: " << p.x << ", y: " << p.y << ")";
    return os;
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

std::ostream &operator<<(std::ostream &os, const SpaceTimePoint p) {
    os << "(x: " << p.x << ", y: " << p.y << ", t: " << p.t << ")";
    return os;
}

bool SpaceTimePoint::operator==(const SpaceTimePoint other) const {
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
    SpaceTimePoint curr = goal;
    path.push_back(curr);
    while (came_from.find(curr) != came_from.end()) {
        curr = came_from.at(curr);
        path.push_back(curr);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<SpaceTimePoint>
a_star(const SpaceTimePoint start, const SpacePoint goal, uint32_t rest_after, int32_t charge, uint32_t width,
       uint32_t height,
       const std::unordered_set<SpaceTimePoint> &reservations) { // heuristic is always manhatten distance
    if (charge < 0) {
        return std::vector<SpaceTimePoint>{};
    }

    using ChargePoint = std::pair<SpaceTimePoint, int32_t>;
    const auto cmp = [goal](ChargePoint p1, ChargePoint p2) {
        // in f(p) = g(p) + h(p), manhatten distance is the heuristic and g(p) = p.t
        const auto d1 = manhatten_distance(p1.first, goal) + p1.first.t;
        const auto d2 = manhatten_distance(p2.first, goal) + p2.first.t;
        return d1 > d2 || (d1 == d2 && p1.first.x < p2.first.x); // direct comparison of x, y is used to break ties
    };
    std::priority_queue<ChargePoint, std::vector<ChargePoint>, decltype(cmp)> open_set(cmp);
    open_set.push(std::make_pair(start, charge));

    std::unordered_map<SpaceTimePoint, SpaceTimePoint> came_from{};

    // If we don't manage to move away from the start or spend >= 4/5ths of the time waiting, give up
    const auto heuristic_distance = static_cast<int32_t>(manhatten_distance(start, goal));
    const int32_t heuristic_factor = 20;

    while (!open_set.empty()) {
        const auto curr = open_set.top();
        open_set.pop();

        if (SpacePoint(curr.first) == goal) {
            const auto path = reconstruct_path(came_from, curr.first); // use curr to ensure we know the time
            return path;
        }

        const auto valid_neighbours = get_neighbours(curr.first, width, height, reservations);
        for (const auto n : valid_neighbours) {
            // Normally we check the cost so far, our cost so far is always the same. So we check came_from instead,
            // as any seen (even not explored) node has an entry
            const int32_t new_charge = n.x == curr.first.x && n.y == curr.first.y ? curr.second : curr.second - 1;
            if (new_charge < 0) {
                break;
            }

            if (/*(n.x == start.x && n.y == start.y && n.t - start.t >= heuristic_factor * heuristic_distance) || */
                    (n.t - start.t) >= (heuristic_factor * heuristic_distance)) { // we are staying still...
                std::cout << "Quit for heuristic!\n";
                std::cout << "Heuristic distance: " << heuristic_distance << "\n";
                std::cout << "n: \t" << n << "\n";
                std::cout << "start: \t" << start << "\n";
                return std::vector<SpaceTimePoint>{};
            }

            if (came_from.empty()) { // no reservations exist
                came_from.insert(std::make_pair(n, curr.first));
                open_set.push(std::make_pair(n, new_charge));
            } else if (came_from.find(n) == came_from.end()) { // no reservations that trouble us
                if (SpacePoint(n) == goal) { // check if the goal is free for the additional rest period
                    bool all_available = true;
                    for (uint32_t i{0}; i <= rest_after + 1; ++i) {
                        // n.t is already 1 in the
                        // n.t + k is k + 1 in the future
                        // if k + 1 in the future are okay with us, we can stay for k and be fine?
                        if (came_from.find(SpaceTimePoint(n.x, n.y, n.t + i)) != came_from.end()) {
                            all_available = false;
                            break;
                        }
                    }
                    if (all_available) {
                        came_from.insert(std::make_pair(n, curr.first));
                        open_set.push(std::make_pair(n, new_charge));
                    }
                } else {
                    came_from.insert(std::make_pair(n, curr.first));
                    open_set.push(std::make_pair(n, new_charge));
                }
            }
        }
    }

    const std::vector<SpaceTimePoint> path;
    return path;
}

std::pair<bool, int32_t>
find_path_and_update(SpaceTimePoint start, SpacePoint goal, uint32_t rest_after, int32_t charge, uint32_t width,
                     uint32_t height, std::unordered_set<SpaceTimePoint> &reservations) {
    std::vector<SpaceTimePoint> path;
    if (start.x != goal.x && start.y != goal.y) {
        path = a_star(start, goal, rest_after, charge, width, height, reservations);
        if (path.empty()) {
            return std::make_pair(false, -1);
        }
    }
    for (const auto p : path) {
        reservations.insert(p);
    }
    int32_t used_charge = get_used_charge(path);
    return std::make_pair(true, used_charge);
}

int32_t get_used_charge(const std::vector<SpaceTimePoint> &path) {
    if (path.empty()) {
        return 0;
    }

    int32_t charge{0};
    SpacePoint last{path.at(0)};
    for (const auto p : path) {
        SpacePoint curr{p};
        if (last != curr) {
            ++charge;
        }
        last = curr;
    }
    return charge;
}
