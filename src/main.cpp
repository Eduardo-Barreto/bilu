#include "bilu/explore_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Criar uma instância do nó
    auto node = std::make_shared<ExploreNode>();

    // Criar um executor e adicionar o nó
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Rodar o executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
