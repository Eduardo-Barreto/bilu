#ifndef EXPLORE_NODE_HPP
#define EXPLORE_NODE_HPP

#include "bilu/explore_helper.hpp"
#include "bilu/maze.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <array>
#include <rclcpp/rclcpp.hpp>

namespace bilu {

class ExploreNode : public rclcpp::Node {
public:
    ExploreNode();

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    Position                                               robot_position;
    Position                                               target_position;
    Maze                                                   maze;
    std::array<bool, 4>                                    blocked_directions;

    void start_exploration();
    void send_move_command(Direction direction);
    void service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future);
    void process_response(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>& response);
};

}  // namespace bilu
#endif  // EXPLORE_NODE_HPP
