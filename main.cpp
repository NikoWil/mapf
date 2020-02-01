#include <iostream>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

/* Used to implement custom hash functions for own datatypes (SpaceTimePoint specifically)
 * Taken from
 * https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
 * and thus from boost::hash_combine
 *
 * @tparam T Type of the element to be added to the hash
 * @param seed The hash up to this point
 * @param v The value of which the hash is combined with the current hash
 */
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6u) + (seed>>2u);
}

enum class Move {
    Up, Down, Left, Right, Rest
};

struct SpaceTimePoint;

struct SpacePoint {
    explicit SpacePoint(SpaceTimePoint p);

    SpacePoint(int32_t x, int32_t y) {
        this->x = x;
        this->y = y;
    }

    bool operator==(const SpacePoint other) const noexcept {
        return this->x == other.x
               && this->y == other.y;
    }

    int32_t x;
    int32_t y;
};

struct SpaceTimePoint {
    explicit SpaceTimePoint(SpacePoint p, int32_t t=0) {
        this->x = p.x;
        this->y = p.y;
        this->t = t;
    }

    explicit SpaceTimePoint(int32_t x, int32_t y, int32_t t) {
        this->x = x;
        this->y = y;
        this->t = t;
    }

    friend std::ostream& operator<<(std::ostream& os, SpaceTimePoint p) {
        os << "x: " << p.x << ", y: " << p.y << ", t: " << p.t;
        return os;
    }

    bool operator==(const SpaceTimePoint& other) const {
        return this->x == other.x
               && this->y == other.y
               && this->t == other.t;
    }

    int32_t x;
    int32_t y;
    int32_t t;
};

SpacePoint::SpacePoint(SpaceTimePoint p) {
    this->x = p.x;
    this->y = p.y;
}


namespace std {
    template<>
    struct hash<SpaceTimePoint> {
        size_t operator()(SpaceTimePoint const& p) const noexcept {
            std::hash<int32_t> hasher;
            size_t hash = hasher(p.x);
            hash_combine(hash, p.y);
            hash_combine(hash, p.t);
            return hash;
        }
    };
}

template<typename P1, typename P2>
uint32_t manhatten_distance(P1 p1, P2 p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

std::vector<SpaceTimePoint> get_neighbours(SpaceTimePoint p, int32_t width, int32_t height, const std::unordered_set<SpaceTimePoint>& reservations);

std::vector<SpaceTimePoint> a_star(SpacePoint start, SpacePoint goal, uint32_t width, uint32_t height, const std::unordered_set<SpaceTimePoint>& reservations);

int main() {
    // 1. read the input, parse the instance
    // 2. check if the instance is valid
    // 3. distribute the work & solve the pathing
    const SpacePoint start{0, 0};
    const SpacePoint goal{4, 5};
    const int32_t width{10};
    const int32_t height{10};
    std::unordered_set<SpaceTimePoint> reservations{};
    // TODO: fill the reservations with all the starting positions?
    // TODO: if there are multiple reservations on the same field (i.e. resting happens), add reservations such that we don't reserve longer than needed (we don't worry about being trained when resting, only crashing)

    // TODO: If a robot has no new tour, make sure it does not block another delivery by sitting at a shelf

    std::vector<std::vector<SpaceTimePoint>> paths;

    const auto path = a_star(start, goal, width, height, reservations);
    paths.push_back(path);

    for (const auto p : path) {
        reservations.insert(p);
    }

    const SpacePoint start_2{5, 0};
    const SpacePoint goal_2{2, 0};
    const auto path_2 = a_star(start_2, goal_2, width, height, reservations);
    paths.push_back(path_2);

    // TODO: check if path was empty -> means no path found!

    for (const auto curr : paths) {
        std::cout << "New path:\n";
        for (const auto p : curr) {
            std::cout << p << "\n";
        }
        std::cout << "\n";
    }

    return 0;
}

std::vector<SpaceTimePoint> get_neighbours(SpaceTimePoint p, int32_t width, int32_t height, const std::unordered_set<SpaceTimePoint>& reservations) {
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
            && reservations.find(SpaceTimePoint(n.x, n.y, n.t - 1)) == reservations.end()  // we must not train someone else
            && reservations.find(SpaceTimePoint(n.x, n.y, n.t + 1)) == reservations.end())) // we must not force someone else into training us
        {
            valid_neighbours.push_back(n);
        }
    }
    return valid_neighbours;
}

std::vector<SpaceTimePoint> reconstruct_path(const std::unordered_map<SpaceTimePoint, SpaceTimePoint>& came_from, SpaceTimePoint goal) {
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

std::vector<SpaceTimePoint> a_star(const SpacePoint start, const SpacePoint goal, uint32_t width, uint32_t height, const std::unordered_set<SpaceTimePoint>& reservations) { // heuristic is always manhatten distance
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