#include <iostream>
#include <queue>
#include <unordered_set>
#include <algorithm>

#include "pathfinding.h"
#include "input_parsing.h"

void print_path(const std::vector<SpaceTimePoint>& path, const std::string& name) {
    std::cout << name << "\n";
    for (const auto p : path) {
        std::cout << "\t(x: " << p.x << ", y: " << p.y << ", t: " << p.t << ")\n";
    }
}

int main() {
    // 0. get parameters from the command line

    // 1. read the input, parse the instance
    const std::string filename{"test.txt"};
    Instance inst = parse_instance(filename);
    // print_instance(inst);

    // 2. check if the instance is solvable
    // TODO: no

    // 3. distribute the work

    // 4. solve the pathfinding
    std::unordered_set<SpaceTimePoint> reservations{};

    std::vector<std::tuple<int32_t, int32_t, SpaceTimePoint>> robot_endpoints;
    for (const auto p : inst.robot_positions) {
        robot_endpoints.push_back(std::make_tuple(p.first, inst.charge, SpaceTimePoint(p.second)));
    }

    const auto time_comp = [](const std::tuple<int32_t, int32_t, SpaceTimePoint> p1,
                              const std::tuple<int32_t, int32_t, SpaceTimePoint> p2) {
        return std::get<2>(p1).t < std::get<2>(p2).t;
    };
    std::sort(robot_endpoints.begin(), robot_endpoints.end(), time_comp);
    std::cout << "Robot endpoint times\n";
    for (const auto r : robot_endpoints) {
        std::cout << std::get<2>(r).t << " ";
    }
    std::cout << "\n";

    // Keep delivering
    for (const auto d : inst.deliveries) {
        bool delivery_handled = false;

        const auto delivery_start = std::find_if(inst.shelf_positions.begin(), inst.shelf_positions.end(),
                                                 [=](std::pair<char, SpacePoint> shelf) {
                                                     return shelf.first == d.start;
                                                 })->second;
        const auto delivery_goal = std::find_if(inst.shelf_positions.begin(), inst.shelf_positions.end(),
                                                [=](std::pair<char, SpacePoint> shelf) {
                                                    return shelf.first == d.goal;
                                                })->second;
        // Order robots by who is out of work first
        std::sort(robot_endpoints.begin(), robot_endpoints.end(), time_comp);

        std::cout << "New delivery.\n\tstart: " << delivery_start << "\n\tgoal: " << delivery_goal << "\n";

        for (auto& robot : robot_endpoints) {
            const auto robot_start = std::get<2>(robot);
            auto charge = std::get<1>(robot);
            const auto robot_id = std::get<0>(robot);

            // path robot_start - start
            const auto to_start = a_star(robot_start, delivery_start, 1, charge, inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_start);
            if (charge < 0) { continue; }

            const SpaceTimePoint delivery_start_timed(delivery_start, to_start.back().t + 1);
            // rest for a moment to load the package

            // Go to a charger to recharge the robot
            std::sort(inst.charger_positions.begin(), inst.charger_positions.end(),
                      [=](const SpacePoint p1, const SpacePoint p2) {
                          const auto d1 =
                                  manhatten_distance(delivery_start, p1) + manhatten_distance(delivery_goal, p1);
                          const auto d2 =
                                  manhatten_distance(delivery_start, p2) + manhatten_distance(delivery_goal, p2);
                          return d1 < d2;
                      });

            const auto to_charge = a_star(delivery_start_timed, inst.charger_positions.at(0), inst.charge, charge,
                                          inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_charge);
            if (charge < 0) { continue; }

            // check how long we want to stay at the charger
            const auto rest_period = inst.charge - charge;
            const SpaceTimePoint charger_after_recharge(to_charge.back().x, to_charge.back().y,
                                                        to_charge.back().t + rest_period);
            charge = inst.charge; // recharge happened

            // path start - goal
            const auto to_goal = a_star(charger_after_recharge, delivery_goal, 1, charge, inst.width, inst.height,
                                        reservations);
            charge = charge - get_used_charge(to_goal);
            if (charge < 0) { continue; }

            // rest for a moment to unload
            const SpaceTimePoint after_delivery(to_goal.back().x, to_goal.back().y, to_goal.back().t + 1);

            // find the closest charger to our goal and move there to recharge
            std::sort(inst.charger_positions.begin(), inst.charger_positions.end(),
                      [=](const SpacePoint p1, const SpacePoint p2) {
                          const auto d1 = manhatten_distance(delivery_goal, p1);
                          const auto d2 = manhatten_distance(delivery_goal, p2);
                          return d1 < d2;
                      });
            const auto to_2_charge = a_star(after_delivery, inst.charger_positions.at(0), inst.charge, charge,
                                            inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_2_charge);
            if (charge < 0) { continue; }
            const auto rest_period_2 = inst.charge - charge;
            charge = inst.charge;

            const SpaceTimePoint robot_endpoint(to_2_charge.back().x, to_2_charge.back().y, to_2_charge.back().t + rest_period_2 + 1);

            // Assign a new position + charge to our robot
            robot = std::make_tuple(robot_id, charge, robot_endpoint);

            // TODO: insert rests for the first rest period into reservations
            for (int32_t k{1}; k <= rest_period; ++k) {
                const auto x = charger_after_recharge.x;
                const auto y = charger_after_recharge.y;
                const auto t = charger_after_recharge.t - k;
                reservations.insert(SpaceTimePoint(x, y, t));
            }
            // TODO: insert rests for the second rest period into reservations
            for (int32_t k{1}; k <= rest_period_2 + 1; ++k) {
                const auto x = to_2_charge.back().x;
                const auto y = to_2_charge.back().y;
                const auto t = to_2_charge.back().t + k;
                reservations.insert(SpaceTimePoint(x, y, t));
            }
            // TODO: insert reservations for all paths
            for (const auto p : to_start) {
                reservations.insert(p);
            }
            for (const auto p : to_charge) {
                reservations.insert(p);
            }
            for (const auto p : to_goal) {
                reservations.insert(p);
            }
            for (const auto p : to_2_charge) {
                reservations.insert(p);
            }

            std::string move_string;
            move_string.reserve(to_start.size() + to_charge.size() + rest_period + to_goal.size() + to_2_charge.size() + rest_period_2);
            const auto move_gen = [](const SpaceTimePoint p1, const SpaceTimePoint p2) {
                if (p1.x < p2.x) {
                    return 'R';
                } else if (p1.x > p2.x) {
                    return 'L';
                } else if (p1.y < p2.y) {
                    return 'D';
                } else if (p1.y > p2.y) {
                    return 'U';
                } else {
                    return 'S';
                }
            };
            for (size_t k{0}; k < to_start.size() - 1; ++k) {
                move_string.push_back(move_gen(to_start.at(k), to_start.at(k+1)));
            }
            move_string.push_back(d.id); // staying for loading
            for (size_t k{0}; k < to_charge.size() - 1; ++k) {
                move_string.push_back(move_gen(to_charge.at(k), to_charge.at(k + 1)));
            }
            for (int32_t k{0}; k < rest_period; ++k) {
                move_string.push_back('S');
            }
            for (size_t k{0}; k < to_goal.size() - 1; ++k) {
                move_string.push_back(move_gen(to_goal.at(k), to_goal.at(k + 1)));
            }
            move_string.push_back(d.id); // staying for unloading
            for (size_t k{0}; k < to_2_charge.size() - 1; ++k) {
                move_string.push_back(move_gen(to_2_charge.at(k), to_2_charge.at(k + 1)));
            }
            for (int32_t k{0}; k < rest_period_2; ++k) {
                move_string.push_back('S');
            }
            std::cout << "robot: " << robot_id << ", move_string: " << move_string << "\n";
            // If we found a valid path, yay!
            delivery_handled = true;
            break;
        }

        // We didn't find any good robot. Nooo!
        if (!delivery_handled) {
            std::cout << "No solution\n";
            break;
        }
    }

    return 0;
}
