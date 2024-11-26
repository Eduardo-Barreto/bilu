#ifndef BILU_NODE_HPP
#define BILU_NODE_HPP

#include "bilu/astar.hpp"
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
    void               service_callback(const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture& future);
    static std::string direction_to_string(Direction direction);

    static CellState convertToCellState(const std::string& cell);
    Grid             grid;
    AStar            astar;
    bilu::Node       start, end, current;

    bool started = false;
};

}  // namespace bilu

#endif  // BILU_NODE_HPP
