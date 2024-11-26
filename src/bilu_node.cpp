#include "bilu/bilu_node.hpp"
#include <chrono>
#include "bilu/grid.hpp"

namespace bilu {

BiluNode::BiluNode() : Node("bilu_node"), grid(20, 20), astar(grid) {
    move_client = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    while (!move_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    send_move_command(Direction::RIGHT);
}

std::string BiluNode::direction_to_string(Direction direction) {
    switch (direction) {
        case Direction::UP:
            return "up";
        case Direction::DOWN:
            return "down";
        case Direction::LEFT:
            return "left";
        case Direction::RIGHT:
            return "right";
    }

    return "";
}

void BiluNode::send_move_command(Direction direction) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction_to_string(direction);

    auto result = move_client->async_send_request(
        request, [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) { service_callback(future); }
    );
}

void BiluNode::service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future) {
    const auto& response = future.get();

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move the robot.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Robot moved successfully.");
    start.x = static_cast<unsigned char>(response->robot_pos[0]);
    start.y = static_cast<unsigned char>(response->robot_pos[1]);

    if (!this->started) {
        started = true;
        end.x = static_cast<unsigned char>(response->target_pos[1]);
        end.y = static_cast<unsigned char>(response->target_pos[0]);

        grid.loadExploredMap("explored_map.csv");
    }

    if (start == end) {
        RCLCPP_INFO(this->get_logger(), "Robot reached the target.");
        grid.displayExploredMap(start.x, start.y);
        grid.saveExploredMap("explored_map.csv");
        return;
    }

    std::array<CellState, 4> neighbors = {
        convertToCellState(response->right), convertToCellState(response->down), convertToCellState(response->left),
        convertToCellState(response->up)
    };

    Direction next_move = astar.exploreAndMove(start, end, neighbors);
    RCLCPP_INFO(this->get_logger(), "Robot position: (%d, %d)", start.x, start.y);
    RCLCPP_INFO(this->get_logger(), "Target position: (%d, %d)", end.x, end.y);
    RCLCPP_INFO(this->get_logger(), "Next move: %s", direction_to_string(next_move).c_str());

    send_move_command(next_move);
}

CellState BiluNode::convertToCellState(const std::string& cell) {
    return cell == "b" ? CellState::OBSTACLE : CellState::FREE;
}

}  // namespace bilu
