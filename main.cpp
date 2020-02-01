#include <iostream>
#include <queue>
#include <unordered_set>
#include <fstream>

#include "pathfinding.h"

bool read_lines(const std::string& filename, std::vector<std::string>& lines) {
    std::ifstream input(filename.c_str());
    if (!input) {
        return false;
    }

    for (std::string line; std::getline(input, line);) {
        if (!line.empty() && line.at(line.length() -1) == '\n') {
            line.erase(line.length() - 1);
        }
        lines.push_back(line);
    }
    return true;
}

struct Delivery {
    char id;
    char start;
    char goal;
};

struct Instance {
    std::vector<std::pair<int32_t , SpacePoint>> robot_positions;
    std::vector<std::pair<char, SpacePoint>> shelf_positions;
    std::vector<SpacePoint> charger_positions;
    std::vector<Delivery> deliveries;
    int32_t width;
    int32_t height;
};

int main() {
    // 0. get parameters from the command line

    // 1. read the input, parse the instance
    const std::string filename{"test.txt"};
    std::vector<std::string> lines;

     if (!read_lines(filename, lines)) {
         std::cout << "Cannot open the file: " << filename << "\n";
         std::exit(1);
     }
     if (lines.size() == 0) {
         std::cout << "Empty instance, what the heck?\n";
         std::exit(0);
     }

    Instance inst;
    std::vector<std::pair<int32_t , SpacePoint>> robot_positions;
    std::vector<std::pair<char, SpacePoint>> shelf_positions;
    std::vector<SpacePoint> charger_positions;
    std::vector<Delivery> deliveries;
    int32_t width{static_cast<int32_t >(lines.at(0).length()) - 2};
    int32_t height{0};

    size_t i{1};
    for (; i < lines.size(); ++i) { // parse the grid itself
        const auto l = lines.at(i);
        if (!l.empty() && l.at(0) != '#') {
            break;
        }
        if (l.length() != static_cast<size_t>(width) + 2) {
            std::cout << "Line " << i << " in instance is too short\n";
            std::exit(1);
        }

        for (size_t k{1}; k < l.length() - 1; ++k) { // skip left and right wall
            char char_at = l[k];
            if (char_at >= 'A' && char_at <= 'Z') { // Capital letter
                shelf_positions.push_back(std::make_pair(char_at, SpacePoint(k - 1, i - 1)));
            } else if (char_at == '_') { // _ charging station
                charger_positions.push_back(SpacePoint(k - 1, i - 1));
            } else if (char_at >= '0' && char_at <= '9') { // digit
                int32_t id = char_at - '0';
                robot_positions.push_back(std::make_pair(id, SpacePoint(k - 1, i - 1)));
            }
        }
    }
    height = i - 2;
    if (height < 0) {
        std::cout << "Invalid instance, grid needed\n";
        std::exit(1);
    }

    if (i == lines.size() || lines[i].find("charge") != 0) {
        std::cout << "Invalid instance, charge needed\n";
        std::exit(1);
    }
    int32_t charge = std::stoi(lines[i].substr(7));
    ++i;
    for (; i < lines.size(); ++i) { // parse the needed deliveries
        const auto line = lines.at(i);
        char id = line.at(0);
        char start = line.at(2);
        char goal = line.at(4);
        deliveries.push_back({id, start, goal});
    }

    std::cout << "width: " << width << "\n";
    std::cout << "height: " << height << "\n";
    std::cout << "charge: " << charge << "\n";
    std::cout << "shelf_positions:\n";
    for (const auto s : shelf_positions) {
        std::cout << "\t" <<  s.first << " ,x: " << s.second.x << " ,y: " << s.second.y << "\n";
    }
    std::cout << "robot_position:\n";
    for (const auto r : robot_positions) {
        std::cout << "\t" << r.first << ", x: " << r.second.x << " ,y: " << r.second.y << "\n";
    }
    std::cout << "charger_positions:\n";
    for (const auto c : charger_positions) {
        std::cout << "\tx: " << c.x << ", y: " << c.y << "\n";
    }
    std::cout << "deliveries:\n";
    for (const auto d : deliveries) {
        std::cout << "\tid: " << d.id << ", start: " << d.start << ", goal: " << d.goal << "\n";
    }

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
