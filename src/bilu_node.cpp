#include "bilu/bilu_node.hpp"

namespace bilu {
BiluNode::BiluNode() : Node("bilu_node"), logger_(this->get_logger()) {
    RCLCPP_INFO(logger_, "Iniciando o node Bilu...");
}
}  // namespace bilu

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bilu::BiluNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
