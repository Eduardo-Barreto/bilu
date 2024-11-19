#ifndef BILU_BILU_NODE_HPP
#define BILU_BILU_NODE_HPP

#include <rclcpp/rclcpp.hpp>

namespace bilu {
class BiluNode : public rclcpp::Node {
public:
    /**
     * @brief Construtor padrão do node
     */
    BiluNode();

private:
    rclcpp::Logger logger_;
};
}  // namespace bilu

#endif  // BILU_BILU_NODE_HPP
