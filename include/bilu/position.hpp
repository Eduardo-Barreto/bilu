#ifndef POSITION_HPP
#define POSITION_HPP

#include <array>

namespace bilu {

/**
 * @brief Representa as direções possíveis para movimentação.
 */
enum Direction {
    UP = 0,
    DOWN,
    LEFT,
    RIGHT
};

struct Position {
    Position();
    Position(int x, int y);
    int                 x;
    int                 y;
    std::array<bool, 4> walls;

    bool visited = false;
    bool wall = false;
    int  cost;

    bool     operator==(const Position& other) const;
    Position operator-(const Position& other) const;
    Position operator +(const Direction& direction) const;
};

}  // namespace bilu

#endif  // POSITION_HPP
