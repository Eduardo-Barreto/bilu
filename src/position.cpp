#include "bilu/position.hpp"

#include <array>

namespace bilu {

Position::Position() : x(0), y(0) { }

Position::Position(int x, int y) : x(x), y(y) { }

bool Position::operator==(const Position& other) const {
    return x == other.x && y == other.y;
}

Position Position::operator-(const Position& other) const {
    return {x - other.x, y - other.y};
}

Position Position::operator+(const Direction& direction) const {
    switch (direction) {
        case UP:
            return {x, y - 1};
        case DOWN:
            return {x, y + 1};
        case LEFT:
            return {x - 1, y};
        case RIGHT:
            return {x + 1, y};
        default:
            return {x, y};
    }

    return {x, y};
}

}  // namespace bilu
