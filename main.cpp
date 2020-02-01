#include <iostream>
#include <queue>
#include <unordered_set>

#include "pathfinding.h"
#include "input_parsing.h"

int main() {
    // 0. get parameters from the command line

    // 1. read the input, parse the instance
    const std::string filename{"test.txt"};
    const Instance inst = parse_instance(filename);
    print_instance(inst);

    // 2. check if the instance is solvable
    // 3. distribute the work & solve the pathing
    const SpacePoint test_start{0, 0};
    const SpacePoint test_goal{4, 5};
    const int32_t test_width{10};
    const int32_t test_height{10};
    std::unordered_set<SpaceTimePoint> reservations{};
    // TODO: fill the reservations with all the starting positions?
    // TODO: if there are multiple reservations on the same field (i.e. resting happens), add reservations such that we don't reserve longer than needed (we don't worry about being trained when resting, only crashing)

    // TODO: If a robot has no new tour, make sure it does not block another delivery by sitting at a shelf

    std::vector<std::vector<SpaceTimePoint>> paths;

    const auto path = a_star(test_start, test_goal, test_width, test_height, reservations);
    paths.push_back(path);

    for (const auto p : path) {
        reservations.insert(p);
    }

    const SpacePoint test_start_2{4, 0};
    const SpacePoint test_goal_2{2, 0};
    const auto path_2 = a_star(test_start_2, test_goal_2, test_width, test_height, reservations);
    paths.push_back(path_2);

    // TODO: check if path was empty -> means no path found!

    /*
    for (const auto curr : paths) {
        std::cout << "New path:\n";
        for (const auto p : curr) {
            std::cout << p << "\n";
        }
        std::cout << "\n";
    }*/

    return 0;
}
