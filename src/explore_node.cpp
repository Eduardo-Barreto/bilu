#include "bilu/explore_node.hpp"

ExploreNode::ExploreNode() : Node("explore_node"), robot_pos_{0, 0}, target_pos_{5, 5} {
    client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    for (auto& row : grid_) {
        row.fill(0);  // Inicializa o mapa como não visitado
    }
    grid_[robot_pos_[0]][robot_pos_[1]] = 1;  // Marca a posição inicial como visitada

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    start_exploration();
}

void ExploreNode::start_exploration() {
    send_move_command(bilu::Direction::RIGHT);
    RCLCPP_INFO(this->get_logger(), "Starting Flood Fill exploration...");
    flood_fill(grid_, robot_pos_[0], robot_pos_[1]);
}

void ExploreNode::send_move_command(bilu::Direction direction) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = bilu::direction_to_string(direction);

    auto result =
        client_->async_send_request(request, [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
            service_callback(future);
        });
}

void ExploreNode::flood_fill(std::array<std::array<int, GRID_SIZE>, GRID_SIZE>& grid, int start_x, int start_y) {
    std::queue<std::pair<int, int>> to_visit;

    // Adiciona a célula inicial
    to_visit.emplace(start_x, start_y);

    while (!to_visit.empty()) {
        auto [x, y] = to_visit.front();
        to_visit.pop();

        // Se a célula já foi visitada ou é bloqueada, pula
        if (map_[x][y] != 0) {
            continue;
        }

        // Marca como visitada
        map_[x][y] = 1;

        // Adiciona as células vizinhas acessíveis
        if (x > 0 && map_[x - 1][y] == 0) {
            to_visit.emplace(x - 1, y);
        }
        if (x < static_cast<int>(map_.size()) - 1 && map_[x + 1][y] == 0) {
            to_visit.emplace(x + 1, y);
        }
        if (y > 0 && map_[x][y - 1] == 0) {
            to_visit.emplace(x, y - 1);
        }
        if (y < static_cast<int>(map_[0].size()) - 1 && map_[x][y + 1] == 0) {
            to_visit.emplace(x, y + 1);
        }
    }
}

void ExploreNode::service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future) {
    const auto& response = future.get();

    // Atualiza as informações do robô e do alvo
    robot_pos_ = {response->robot_pos[0], response->robot_pos[1]};
    target_pos_ = {response->target_pos[0], response->target_pos[1]};
    update_blocked_directions(response);

    // Marca a posição atual como visitada
    grid_[robot_pos_[0]][robot_pos_[1]] = 1;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Robot position: (%d, %d)", robot_pos_[0], robot_pos_[1]);

        // Recalcula o Flood Fill com base no estado atual
        std::array<std::array<int, GRID_SIZE>, GRID_SIZE> flood_map{};
        flood_fill(flood_map, robot_pos_[0], robot_pos_[1]);

        // Determina a direção preferida com base no Flood Fill
        bilu::Direction next_direction = bilu::calculate_next_direction(robot_pos_, flood_map, blocked_directions);

        RCLCPP_INFO(
            this->get_logger(), "Next preferred direction: %s", bilu::direction_to_string(next_direction).c_str()
        );

        // Envia o próximo comando de movimento
        send_move_command(next_direction);
    } else {
        RCLCPP_WARN(this->get_logger(), "Movement blocked at (%d, %d)", robot_pos_[0], robot_pos_[1]);
        update_blocked_directions(response);

        // Escolhe uma direção alternativa
        bilu::Direction next_direction = bilu::get_next_direction(blocked_directions);
        RCLCPP_WARN(this->get_logger(), "Trying next direction: %s", bilu::direction_to_string(next_direction).c_str());
        send_move_command(next_direction);
    }
}

void ExploreNode::update_blocked_directions(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>& response) {
    blocked_directions = {response->left == "b", response->down == "b", response->up == "b", response->right == "b"};

    int x = robot_pos_[0];
    int y = robot_pos_[1];

    // Marca a célula atual como visitada
    map_[x][y] = 1;

    // Atualiza as células ao redor com base nas direções bloqueadas
    if (x > 0 && response->left == "b") {
        map_[x - 1][y] = -1;
    }
    if (x < static_cast<int>(map_.size()) - 1 && response->right == "b") {
        map_[x + 1][y] = -1;
    }
    if (y > 0 && response->up == "b") {
        map_[x][y - 1] = -1;
    }
    if (y < static_cast<int>(map_[0].size()) - 1 && response->down == "b") {
        map_[x][y + 1] = -1;
    }
}

void ExploreNode::print_map() {
    for (const auto& row : map_) {
        for (const auto& cell : row) {
            if (cell == -1) {
                std::cout << "X ";  // Bloqueado
            } else if (cell == 1) {
                std::cout << ". ";  // Visitado
            } else {
                std::cout << "  ";  // Não visitado
            }
        }
        std::cout << "\n";
    }
}
