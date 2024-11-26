#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "bilu/grid.hpp"
#include "bilu/node.hpp"

namespace bilu {

class AStar {
public:
    explicit AStar(Grid& grid);

    std::vector<Node> findPath(const Node& start, const Node& end);

    Direction exploreAndMove(Node& current, const Node& end, const std::array<CellState, 4>& neighbors);

    const std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

private:
    Grid& grid;

    static float heuristic(const Node& a, const Node& b);

    static std::vector<Node> reconstructPath(Node* endNode);
};

}  // namespace bilu

#endif  // ASTAR_HPP
