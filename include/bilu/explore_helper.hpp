#ifndef BILU_EXPLORE_HELPER_HPP
#define BILU_EXPLORE_HELPER_HPP

#include "bilu/position.hpp"
#include <array>
#include <string>

namespace bilu {

/**
 * @brief Calcula a direção preferencial para o robô se mover em direção ao alvo.
 *
 * @param robot_pos Posição atual do robô [x, y].
 * @param target_pos Posição do alvo [x, y].
 * @param blocked_directions Direções bloqueadas.
 *
 * @return Direção preferencial.
 */
Direction calculate_preferred_direction(
    const Position& robot_pos, const Position& target_pos, const std::array<bool, 4>& blocked_directions
);

/**
 * @brief Converte uma direção enum para uma string compatível com o serviço.
 *
 * @param direction A direção a ser convertida.
 * @return String representando a direção.
 */
std::string direction_to_string(Direction direction);

/**
 * @brief Determina a próxima direção alternativa em caso de bloqueio.
 *
 * @param blocked_directions Direções bloqueadas.
 * @return Próxima direção alternativa.
 */
Direction get_next_direction(const std::array<bool, 4>& blocked_directions);

}  // namespace bilu

#endif  // BILU_EXPLORE_HELPER_HPP
