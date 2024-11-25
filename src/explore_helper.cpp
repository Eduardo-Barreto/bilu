#include "bilu/explore_helper.hpp"
#include <array>
#include <cstdlib>

namespace bilu {
Direction calculate_preferred_direction(
    const std::array<int8_t, 2>& robot_pos, const std::array<int8_t, 2>& target_pos,
    const std::array<bool, 4>& blocked_directions
) {
    int dx = target_pos[1] - robot_pos[1];
    int dy = target_pos[0] - robot_pos[0];

    if (std::abs(dx) > std::abs(dy)) {
        if (dx > 0 && !blocked_directions[static_cast<int>(RIGHT)]) {
            return RIGHT;
        }
        if (dx < 0 && !blocked_directions[static_cast<int>(LEFT)]) {
            return LEFT;
        }
    } else {
        if (dy > 0 && !blocked_directions[static_cast<int>(DOWN)]) {
            return DOWN;
        }
        if (dy < 0 && !blocked_directions[static_cast<int>(UP)]) {
            return UP;
        }
    }

    return get_next_direction(blocked_directions);
}

std::string direction_to_string(Direction direction) {
    switch (direction) {
        case UP:
            return "up";
        case DOWN:
            return "down";
        case LEFT:
            return "left";
        case RIGHT:
            return "right";
        default:
            return "";
    }
}

Direction get_next_direction(const std::array<bool, 4>& blocked_directions) {
    for (int i = 0; i < 4; i++) {
        if (!blocked_directions.at(i)) {
            return static_cast<Direction>(i);
        }
    }

    return RIGHT;
}

}  // namespace bilu
