#include <iostream>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <random>
#include <fstream>

#include "pathfinding.h"
#include "input_parsing.h"

void print_path(const std::vector<SpaceTimePoint> &path, const std::string &name) {
    std::cout << name << "\n";
    for (const auto p : path) {
        std::cout << "\t(x: " << p.x << ", y: " << p.y << ", t: " << p.t << ")\n";
    }
}

std::string path_to_string(const std::vector<SpaceTimePoint> &path) {
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

    std::string move_string;
    for (size_t k{0}; k < path.size() - 1; ++k) {
        move_string.push_back(move_gen(path.at(k), path.at(k + 1)));
    }
    return move_string;
}

bool find_actions(SpaceTimePoint start, int32_t charge, int32_t needed_steps, int32_t width, int32_t height,
                  const std::unordered_set<SpaceTimePoint> &reservations, std::mt19937 &rng,
                  std::vector<SpaceTimePoint> &path);

void print_output(std::vector<std::pair<int32_t, std::string>> &paths, const std::string &out_filename) {
    std::ofstream fs(out_filename);
    if (!fs) {
        std::cout << "Could not open output filename\n";
        std::exit(1);
    }

    std::sort(paths.begin(), paths.end(), [](const auto p1, const auto p2) {
        return p1.first < p2.first;
    });

    std::string out_string;
    const size_t s{paths.size()};
    // FIXME: assumption, that all path strings are of same length
    for (size_t k{0}; k < paths.at(0).second.size(); ++k) {
        for (size_t l{0}; l < s; ++l) {
            out_string.push_back(paths.at(l).second.at(k));
        }
        out_string.push_back('\n');
    }
    fs.write(out_string.data(), out_string.size());

    fs.close();
}

int main(int argc, char *argv[]) {
    // 0. get parameters from the command line
    if (argc != 3) {
        std::cout << "Invalid program call. Call as './mapf <input file> <output file>\n";
        std::exit(1);
    }

    std::string input_file{argv[1]};
    // 1. read the input, parse the instance
    Instance inst = parse_instance(input_file);
    print_instance(inst);

    // 2. check if the instance is solvable
    // TODO: no

    // 4. solve the pathfinding


    std::vector<std::tuple<int32_t, int32_t, SpaceTimePoint>> robot_endpoints;
    for (const auto p : inst.robot_positions) {
        robot_endpoints.emplace_back(p.first, inst.charge, SpaceTimePoint(p.second));
    }

    const auto time_comp = [](const std::tuple<int32_t, int32_t, SpaceTimePoint> p1,
                              const std::tuple<int32_t, int32_t, SpaceTimePoint> p2) {
        return std::get<2>(p1).t < std::get<2>(p2).t;
    };

    std::vector<std::pair<int32_t, std::string>> move_strings;
    for (const auto &p : inst.robot_positions) {
        move_strings.emplace_back(p.first, std::string{});
    }

    std::unordered_set<SpaceTimePoint> reservations{}; // set of points in time which are occupied
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

        for (auto &robot : robot_endpoints) {
            const auto robot_start = std::get<2>(robot);
            auto charge = std::get<1>(robot);
            const auto robot_id = std::get<0>(robot);

            // path robot_start - start
            const auto to_start = a_star(robot_start, delivery_start, 1, charge, inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_start);
            if (charge < 0 || to_start.empty()) { continue; }

            // rest for a moment to load the package
            const SpaceTimePoint delivery_start_timed(to_start.back().x, to_start.back().y, to_start.back().t + 1);

            // Go to a charger to recharge the robot
            std::sort(inst.charger_positions.begin(), inst.charger_positions.end(),
                      [=](const SpacePoint p1, const SpacePoint p2) {
                          const auto d1 =
                                  manhatten_distance(delivery_start, p1) + manhatten_distance(delivery_goal, p1);
                          const auto d2 =
                                  manhatten_distance(delivery_start, p2) + manhatten_distance(delivery_goal, p2);
                          return d1 < d2;
                      });

            const auto to_charge_1 = a_star(delivery_start_timed, inst.charger_positions.at(0), inst.charge, charge,
                                            inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_charge_1);

            if (charge < 0 || to_charge_1.empty()) { continue; }

            // check how long we want to stay at the charger
            const auto rest_period_1 = inst.charge - charge;
            const SpaceTimePoint charger_after_recharge(to_charge_1.back().x, to_charge_1.back().y,
                                                        to_charge_1.back().t + rest_period_1);
            charge = inst.charge; // recharge happened

            // path start - goal
            const auto to_goal = a_star(charger_after_recharge, delivery_goal, 1, charge, inst.width, inst.height,
                                        reservations);
            charge = charge - get_used_charge(to_goal);
            if (charge < 0 || to_goal.empty()) { continue; }

            // rest for a moment to unload
            const SpaceTimePoint after_delivery(to_goal.back().x, to_goal.back().y, to_goal.back().t + 1);

            // find the closest charger to our goal and move there to recharge
            std::sort(inst.charger_positions.begin(), inst.charger_positions.end(),
                      [=](const SpacePoint p1, const SpacePoint p2) {
                          const auto d1 = manhatten_distance(delivery_goal, p1);
                          const auto d2 = manhatten_distance(delivery_goal, p2);
                          return d1 < d2;
                      });
            const auto to_charge_2 = a_star(after_delivery, inst.charger_positions.at(0), inst.charge, charge,
                                            inst.width, inst.height, reservations);
            charge = charge - get_used_charge(to_charge_2);
            if (charge < 0) { continue; }

            auto rest_period_2{0};
            SpaceTimePoint robot_endpoint(0, 0, 0);
            if (!to_charge_2.empty()) {
                rest_period_2 = inst.charge - charge;
                robot_endpoint = SpaceTimePoint(to_charge_2.back().x, to_charge_2.back().y,
                                                to_charge_2.back().t + rest_period_2);
                charge = inst.charge;
            } else {
                // No path to a charger, no recharging
                robot_endpoint = SpaceTimePoint(to_goal.back().x, to_goal.back().y, to_goal.back().t + 1);
            }

            // Assign a new position + charge to our robot
            robot = std::make_tuple(robot_id, charge, robot_endpoint);

            // insert rests for the first rest period into reservations
            for (int32_t k{1}; k <= rest_period_1; ++k) {
                const auto x = to_charge_1.back().x;
                const auto y = to_charge_1.back().y;
                const auto t = to_charge_1.back().t + k;
                reservations.insert(SpaceTimePoint(x, y, t));
            }
            // insert rests for the second rest period into reservations
            if (!to_charge_2.empty()) {
                for (int32_t k{1}; k <= rest_period_2; ++k) {
                    const auto x = to_charge_2.back().x;
                    const auto y = to_charge_2.back().y;
                    const auto t = to_charge_2.back().t + k;
                    reservations.insert(SpaceTimePoint(x, y, t));
                }
            }
            for (const auto p : to_start) {
                reservations.insert(p);
            }
            for (const auto p : to_charge_1) {
                reservations.insert(p);
            }
            for (const auto p : to_goal) {
                reservations.insert(p);
            }
            for (const auto p : to_charge_2) {
                reservations.insert(p);
            }
            reservations.insert(robot_endpoint);

            std::string move_string;
            /*move_string.reserve(
                    to_start.size() + to_charge_1.size() + rest_period_1 + to_goal.size() + to_charge_2.size() +
                    rest_period_2);*/
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
                move_string.push_back(move_gen(to_start.at(k), to_start.at(k + 1)));
            }
            move_string.push_back(d.id); // staying for loading
            for (size_t k{0}; k < to_charge_1.size() - 1; ++k) {
                move_string.push_back(move_gen(to_charge_1.at(k), to_charge_1.at(k + 1)));
            }
            for (int32_t k{0}; k < rest_period_1; ++k) {
                move_string.push_back('S');
            }
            for (size_t k{0}; k < to_goal.size() - 1; ++k) {
                move_string.push_back(move_gen(to_goal.at(k), to_goal.at(k + 1)));
            }
            move_string.push_back(d.id); // staying for unloadingg
            if (!to_charge_2.empty()) {
                for (size_t k{0}; k < to_charge_2.size() - 1; ++k) {
                    move_string.push_back(move_gen(to_charge_2.at(k), to_charge_2.at(k + 1)));
                }
                for (int32_t k{0}; k < rest_period_2; ++k) {
                    move_string.push_back('S');
                }
            }

            for (auto &m : move_strings) {
                if (m.first == robot_id) {
                    m.second += move_string;
                }
            }

            // If we found a valid path, yay!
            delivery_handled = true;
            break;
        }

        // We didn't find any good robot. Nooo!
        if (!delivery_handled) {
            std::cout << "No solution\n";
            std::exit(0);
            break;
        }
    }

    std::cout << "All packages delivered, now fill 'meaningless' actions for robots to let others deliver.\n";

    size_t max_length{0};
    for (const auto &m : move_strings) {
        max_length = std::max(max_length, m.second.length());
    }

    std::mt19937 rng{std::random_device{}()};
    for (const auto &r : robot_endpoints) {
        const auto end_time = std::get<2>(r).t;
        const auto robot_id = std::get<0>(r);

        if (static_cast<size_t>(end_time) < max_length) {
            const auto needed_steps = max_length - end_time - 1;
            const auto start = std::get<2>(r);
            const auto charge = std::get<1>(r);

            std::vector<SpaceTimePoint> rest_path;
            if (!find_actions(start, charge, needed_steps, inst.width, inst.height, reservations, rng, rest_path)) {
                std::cout << "Not all robots could manage to evade the rest of the pack while no longer needed.\n";
                std::cout
                        << "A solution might be found if we get permission to blow up robots that are past their use\n";
                std::exit(0);
            } else {
                rest_path.push_back(start);
                std::reverse(rest_path.begin(), rest_path.end());
                for (const auto &p : rest_path) {
                    reservations.insert(p);
                }
                const auto move_string = path_to_string(rest_path);
                for (size_t k{0}; k < move_strings.size(); ++k) {
                    if (move_strings.at(k).first == robot_id) {
                        move_strings.at(k).second += move_string;
                        break;
                    }
                }
            }
        }
    }

    std::cout << "Final movements:\n";
    for (const auto &m : move_strings) {
        std::cout << "id: " << m.first << ", str: " << m.second << "\n";
    }

    print_output(move_strings, argv[2]);

    return 0;
}

bool is_avail(SpaceTimePoint p, const std::unordered_set<SpaceTimePoint> &reservations) {
    if (reservations.empty()) {
        return true;
    }

    return reservations.find(p) == reservations.end()
           && reservations.find(SpaceTimePoint(p.x, p.y, p.t - 1)) == reservations.end()
           && reservations.find(SpaceTimePoint(p.x, p.y, p.t + 1)) == reservations.end();
}

bool find_actions(SpaceTimePoint start, int32_t charge, int32_t needed_steps, int32_t width, int32_t height,
                  const std::unordered_set<SpaceTimePoint> &reservations, std::mt19937 &rng,
                  std::vector<SpaceTimePoint> &path) {
    if (charge < 0) {
        return false;
    }
    if (needed_steps < 0) {
        return true;
    }

    std::vector<SpaceTimePoint> neighbours;
    neighbours.emplace_back(start.x, start.y, start.t + 1);
    if (start.x > 0) {
        neighbours.emplace_back(start.x - 1, start.y, start.t + 1);
    }
    if (start.x < width) {
        neighbours.emplace_back(start.x + 1, start.y, start.t + 1);
    }
    if (start.y > 0) {
        neighbours.emplace_back(start.x, start.y - 1, start.t + 1);
    }
    if (start.y < height) {
        neighbours.emplace_back(start.x, start.y + 1, start.t + 1);
    }

    std::vector<SpaceTimePoint> avail_neighbours;
    for (const auto &n : neighbours) {
        if (is_avail(n, reservations)) {
            avail_neighbours.push_back(n);
        }
    }
    // we do a beautiful random walk. Why? Why not!
    std::shuffle(avail_neighbours.begin(), avail_neighbours.end(), rng);
    for (const auto n : avail_neighbours) {
        int32_t new_charge = charge;
        if (start.x != n.x || start.y != n.y) {
            new_charge--;
        }
        if (find_actions(n, new_charge, needed_steps - 1, width, height, reservations, rng, path)) {
            path.push_back(n);
            return true;
        }
    }
    return false;
}
