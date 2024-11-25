#include "bilu/explore_node.hpp"

ExploreNode::ExploreNode() : Node("explore_node"), robot_pos_{0, 0}, target_pos_{5, 5} {
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

    if (response->success) {
        robot_pos_ = {response->robot_pos[0], response->robot_pos[1]};
        target_pos_ = {response->target_pos[0], response->target_pos[1]};
        update_blocked_directions(response);

        RCLCPP_INFO(this->get_logger(), "Robot position: (%d, %d)", robot_pos_[0], robot_pos_[1]);

        bilu::Direction next_direction = bilu::calculate_preferred_direction(robot_pos_, target_pos_, blocked_directions);
        RCLCPP_INFO(
            this->get_logger(), "Next preferred direction: %s", bilu::direction_to_string(next_direction).c_str()
        );
        send_move_command(next_direction);
    } else {
        RCLCPP_WARN(this->get_logger(), "Movement blocked at (%d, %d)", robot_pos_[0], robot_pos_[1]);
        update_blocked_directions(response);

        // Escolhe nova direção considerando os bloqueios
        bilu::Direction next_direction = bilu::get_next_direction(blocked_directions);
        RCLCPP_WARN(this->get_logger(), "Trying next direction: %s", bilu::direction_to_string(next_direction).c_str());
        send_move_command(next_direction);
    }
}

void ExploreNode::update_blocked_directions(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>& response) {
    blocked_directions = {response->left == "b", response->down == "b", response->up == "b", response->right == "b"};
}
