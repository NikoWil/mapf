//
// Created by khondar on 01.02.20.
//

#ifndef MAPF_PATHFINDING_H
#define MAPF_PATHFINDING_H

#include <ostream>
//#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>

/* Used to implement custom hash functions for own datatypes (SpaceTimePoint specifically)
 * Taken from
 * https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
 * and thus from boost::hash_combine
 *
 * @tparam T Type of the element to be added to the hash
 * @param seed The hash up to this point
 * @param v The value of which the hash is combined with the current hash
 */
template<class T>
inline void hash_combine(std::size_t &seed, const T &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6u) + (seed >> 2u);
}

enum class Move {
    Up, Down, Left, Right, Rest
};

struct SpaceTimePoint;

struct SpacePoint {
    explicit SpacePoint(SpaceTimePoint p);

    SpacePoint(int32_t x, int32_t y);

    bool operator==(const SpacePoint other) const noexcept;

    bool operator!=(const SpacePoint other) const noexcept;

    friend std::ostream &operator<<(std::ostream& os, const SpacePoint p);

    int32_t x;
    int32_t y;
};

struct SpaceTimePoint {
    explicit SpaceTimePoint(SpacePoint p, int32_t t = 0);

    explicit SpaceTimePoint(int32_t x, int32_t y, int32_t t);

    friend std::ostream &operator<<(std::ostream &os, const SpaceTimePoint p);

    bool operator==(const SpaceTimePoint other) const;

    int32_t x;
    int32_t y;
    int32_t t;
};

namespace std {
    template<>
    struct hash<SpaceTimePoint> {
        size_t operator()(SpaceTimePoint const &p) const noexcept {
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

std::vector<SpaceTimePoint>
get_neighbours(SpaceTimePoint p, int32_t width, int32_t height, const std::unordered_set<SpaceTimePoint> &reservations);

std::vector<SpaceTimePoint>
reconstruct_path(const std::unordered_map<SpaceTimePoint, SpaceTimePoint> &came_from, SpaceTimePoint goal);

/**
 * start: the start node
 * goal: the goal node, time to reach doesn't matter
 * 
 * rest_after: number of time units the field needs to stay free after arrival, e.g. for loading, unloading, charging
 * max_charge: the max. number of move actions that are legal to be executed, resting does not take charge
 */
std::vector<SpaceTimePoint>
a_star(SpaceTimePoint start, SpacePoint goal, uint32_t rest_after, int32_t charge, uint32_t width, uint32_t height,
       const std::unordered_set<SpaceTimePoint> &reservations);

std::pair<bool, int32_t>
find_path_and_update(SpaceTimePoint start, SpacePoint goal, uint32_t rest_after, int32_t charge, uint32_t width,
                     uint32_t height, std::unordered_set<SpaceTimePoint> &reservations);

int32_t get_used_charge(const std::vector<SpaceTimePoint> &path);

#endif //MAPF_PATHFINDING_H
