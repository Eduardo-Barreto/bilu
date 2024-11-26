#include "bilu/explore_node.hpp"
#include <rclcpp/logging.hpp>
#include "cg_interfaces/srv/move_cmd.hpp"

namespace bilu {

ExploreNode::ExploreNode() : Node("explore_node"), robot_position{2, 2}, target_position{17, 17} {
    // Criando cliente para o serviço MoveCmd
    client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    // Espera até o serviço estar disponível
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Inicia a exploração
    start_exploration();
}

void ExploreNode::start_exploration() {
    send_move_command(bilu::Direction::RIGHT);
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

void ExploreNode::service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future) {
    const auto& response = future.get();

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move the robot.");
        return;
    }

    if (!maze.initialized) {
        maze.init(robot_position, target_position);
    }

    process_response(response);
    maze.update(robot_position);
    maze.print();

    bilu::Direction next_direction =
        bilu::calculate_preferred_direction(robot_position, target_position, blocked_directions);
    RCLCPP_INFO(this->get_logger(), "Next preferred direction: %s", bilu::direction_to_string(next_direction).c_str());
    send_move_command(next_direction);
}

void ExploreNode::process_response(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>& response) {
    robot_position = {response->robot_pos[1], response->robot_pos[0]};
    target_position = {response->target_pos[1], response->target_pos[0]};
    robot_position.walls = {response->up == "b", response->down == "b", response->left == "b", response->right == "b"};
    robot_position.visited = true;

    RCLCPP_INFO(this->get_logger(), "Robot position: [%d, %d]", robot_position.x, robot_position.y);
    RCLCPP_INFO(this->get_logger(), "Target position: [%d, %d]", target_position.x, target_position.y);
    RCLCPP_INFO(
        this->get_logger(), "Walls: [%d, %d, %d, %d]", robot_position.walls[0], robot_position.walls[1],
        robot_position.walls[2], robot_position.walls[3]
    );
}

}  // namespace bilu
