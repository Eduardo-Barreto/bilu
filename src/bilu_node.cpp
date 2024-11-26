#include "bilu/bilu_node.hpp"
#include "bilu/grid.hpp"

namespace bilu {

BiluNode::BiluNode() : Node("bilu_node") {
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
    std::this_thread::sleep_for(std::chrono::seconds(1));
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
}

}  // namespace bilu
