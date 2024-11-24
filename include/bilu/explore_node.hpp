#ifndef EXPLORE_NODE_HPP
#define EXPLORE_NODE_HPP

#include "bilu/explore_helper.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <array>
#include <rclcpp/rclcpp.hpp>

class ExploreNode : public rclcpp::Node {
public:
    ExploreNode();

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    std::array<int8_t, 2>                                  robot_pos_;   // Posição do robô
    std::array<int8_t, 2>                                  target_pos_;  // Posição do alvo
    std::array<bool, 4>                                    blocked_directions;

    void start_exploration();
    void send_move_command(bilu::Direction direction);
    void service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future);
    void update_blocked_directions(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>& response);
};

#endif  // EXPLORE_NODE_HPP
