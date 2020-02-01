//
// Created by khondar on 01.02.20.
//

#ifndef MAPF_INPUT_PARSING_H
#define MAPF_INPUT_PARSING_H

#include <vector>

#include "pathfinding.h"

struct Delivery {
    char id;
    char start;
    char goal;
};

struct Instance {
    std::vector<std::pair<int32_t, SpacePoint>> robot_positions;
    std::vector<std::pair<char, SpacePoint>> shelf_positions;
    std::vector<SpacePoint> charger_positions;
    std::vector<Delivery> deliveries;
    int32_t width;
    int32_t height;
    int32_t charge;
};

bool read_lines(const std::string &filename, std::vector<std::string> &lines);

Instance parse_instance(const std::string& filename);

void print_instance(const Instance& inst);

#endif //MAPF_INPUT_PARSING_H
