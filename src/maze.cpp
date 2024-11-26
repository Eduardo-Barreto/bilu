#include "bilu/maze.hpp"

#include <array>
#include <iostream>

namespace bilu {

Maze::Maze(Position start, Position goal) {
    init(start, goal);
}

void Maze::init(Position start, Position goal) {
    start_ = start;
    goal_ = goal;
    initialized = true;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cells.at(j).at(i) = Position(j, i);
        }
    }
}

void Maze::update(const Position& position) {
    cells.at(position.x).at(position.y) = position;

    for (int i = 0; i < static_cast<int>(position.walls.size()); i++) {
        if (!position.walls.at(i)) {
            continue;
        }

        auto direction = static_cast<Direction>(i);

        Position neighbor = position + direction;

        if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) {
            continue;
        }

        cells.at(neighbor.x).at(neighbor.y).wall = true;

        cells.at(neighbor.x).at(neighbor.y).walls.at(static_cast<int>(direction)) = false;
    }
}

void Maze::print() {
    std::cout << "Maze:" << std::endl;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            auto& cell = cells.at(j).at(i);

            std::cout << " ";

            if (cell == start_) {
                std::cout << "S";
            } else if (cell == goal_) {
                std::cout << "G";
            } else if (cell.visited) {
                std::cout << "V";
            } else if (cell.wall) {
                std::cout << "W";
            } else {
                std::cout << ".";
            }

            std::cout << " ";
        }

        std::cout << std::endl;
    }
}

}  // namespace bilu
