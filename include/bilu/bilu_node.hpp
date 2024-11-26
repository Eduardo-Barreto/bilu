#ifndef BILU_NODE_HPP
#define BILU_NODE_HPP

#include "bilu/grid.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <rclcpp/rclcpp.hpp>

namespace bilu {

class BiluNode : public rclcpp::Node {
public:
    BiluNode();

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client;
    void                                                   send_move_command(Direction direction);
    void service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future);
    static std::string direction_to_string(Direction direction);
};

}  // namespace bilu

#endif  // BILU_NODE_HPP
