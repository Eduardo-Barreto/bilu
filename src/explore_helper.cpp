#include "bilu/explore_helper.hpp"
#include <array>
#include <cstdlib>
#include <limits>

namespace bilu {
Direction calculate_preferred_direction(
    const std::array<int8_t, 2>& robot_pos, const std::array<int8_t, 2>& target_pos,
    const std::array<bool, 4>& blocked_directions
) {
    int dx = target_pos[0] - robot_pos[0];
    int dy = target_pos[1] - robot_pos[1];

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

bilu::Direction calculate_next_direction(
    const std::array<int8_t, 2>& robot_pos,
    const std::array<std::array<int, GRID_SIZE>, GRID_SIZE>& flood_map,
    const std::array<bool, 4>& blocked_directions
) {
    int min_value = std::numeric_limits<int>::max();
    bilu::Direction best_direction = bilu::Direction::UP;

    // Avalia todas as direções possíveis
    for (int dir = 0; dir < 4; ++dir) {
        if (blocked_directions[dir]) {
            continue; // Ignora direções bloqueadas
        }

        std::array<int8_t, 2> next_pos = robot_pos;

        // Calcula a próxima posição com base na direção
        switch (static_cast<bilu::Direction>(dir)) {
            case bilu::Direction::UP:
                next_pos[1]--;
                break;
            case bilu::Direction::DOWN:
                next_pos[1]++;
                break;
            case bilu::Direction::LEFT:
                next_pos[0]--;
                break;
            case bilu::Direction::RIGHT:
                next_pos[0]++;
                break;
        }

        // Verifica se a próxima posição está dentro dos limites
        if (next_pos[0] >= 0 && next_pos[0] < GRID_SIZE && next_pos[1] >= 0 && next_pos[1] < GRID_SIZE) {
            int flood_value = flood_map[next_pos[0]][next_pos[1]];
            if (flood_value < min_value) {
                min_value = flood_value;
                best_direction = static_cast<bilu::Direction>(dir);
            }
        }
    }

    return best_direction;
}

}  // namespace bilu
