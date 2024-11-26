#ifndef MAZE_HPP
#define MAZE_HPP

#include "bilu/explore_helper.hpp"
#include "bilu/position.hpp"

namespace bilu {

constexpr int width = 20;
constexpr int height = 20;

class Maze {
public:
    Maze() = default;

    Maze(Position start, Position goal);

    void init(Position start, Position goal);

    void update(const Position& position);

    void print();

    bool initialized{};

private:
    Position start_;
    Position goal_;

    std::array<std::array<Position, height>, width> cells{};
};

}  // namespace bilu

#endif  // MAZE_HPP
