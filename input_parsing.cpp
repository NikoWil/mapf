//
// Created by khondar on 01.02.20.
//

#include <fstream>
#include <iostream>

#include "input_parsing.h"

bool read_lines(const std::string &filename, std::vector<std::string> &lines) {
    std::ifstream input(filename.c_str());
    if (!input) {
        return false;
    }

    for (std::string line; std::getline(input, line);) {
        if (!line.empty() && line.at(line.length() - 1) == '\n') {
            line.erase(line.length() - 1);
        }
        lines.push_back(line);
    }
    return true;
}

Instance parse_instance(const std::string &filename) {
    std::vector<std::string> lines;

    if (!read_lines(filename, lines)) {
        std::cout << "Cannot open the file: " << filename << "\n";
        std::exit(1);
    }
    if (lines.empty()) {
        std::cout << "Empty instance, what the heck?\n";
        std::exit(0);
    }

    Instance inst;
    inst.width = static_cast<int32_t >(lines.at(0).length()) - 2;
    inst.height = 0;

    size_t i{1};
    for (; i < lines.size(); ++i) { // parse the grid itself
        const auto l = lines.at(i);
        if (!l.empty() && l.at(0) != '#') {
            break;
        }
        if (l.length() != static_cast<size_t>(inst.width) + 2) {
            std::cout << "Line " << i << " in instance is too short\n";
            std::exit(1);
        }

        for (size_t k{1}; k < l.length() - 1; ++k) { // skip left and right wall
            char char_at = l[k];
            if (char_at >= 'A' && char_at <= 'Z') { // Capital letter
                inst.shelf_positions.emplace_back(char_at, SpacePoint(k - 1, i - 1));
            } else if (char_at == '_') { // _ charging station
                inst.charger_positions.emplace_back(k - 1, i - 1);
            } else if (char_at >= '0' && char_at <= '9') { // digit
                int32_t id = char_at - '0';
                inst.robot_positions.emplace_back(id, SpacePoint(k - 1, i - 1));
            }
        }
    }
    inst.height = i - 2;
    if (inst.height < 0) {
        std::cout << "Invalid instance, grid needed\n";
        std::exit(1);
    }

    if (i == lines.size() || lines.at(i).find("charge") != 0) {
        std::cout << "Invalid instance, charge needed\n";
        std::exit(1);
    }
    inst.charge = std::stoi(lines[i].substr(7));
    i += 1; // skip next line that only says 'packages'
    if (i == lines.size() || lines.at(i).find("packages") != 0) {
        std::cout << "Invalid instance, packages needed\n";
        std::exit(1);
    }
    i += 1;
    for (; i < lines.size(); ++i) { // parse the needed deliveries
        const auto line = lines.at(i);
        char id = line.at(0);
        char start = line.at(2);
        char goal = line.at(4);
        inst.deliveries.push_back({id, start, goal});
    }

    return inst;
}

void print_instance(const Instance &inst) {
    std::cout << "width: " << inst.width << "\n";
    std::cout << "height: " << inst.height << "\n";
    std::cout << "charge: " << inst.charge << "\n";
    std::cout << "shelf_positions:\n";
    for (const auto s : inst.shelf_positions) {
        std::cout << "\t" << s.first << " ,x: " << s.second.x << " ,y: " << s.second.y << "\n";
    }
    std::cout << "robot_positions:\n";
    for (const auto r : inst.robot_positions) {
        std::cout << "\t" << r.first << ", x: " << r.second.x << " ,y: " << r.second.y << "\n";
    }
    std::cout << "charger_positions:\n";
    for (const auto c : inst.charger_positions) {
        std::cout << "\tx: " << c.x << ", y: " << c.y << "\n";
    }
    std::cout << "deliveries:\n";
    for (const auto d : inst.deliveries) {
        std::cout << "\tid: " << d.id << ", start: " << d.start << ", goal: " << d.goal << "\n";
    }
}
