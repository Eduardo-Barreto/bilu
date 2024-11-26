#include "bilu/astar.hpp"

#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>

namespace bilu {

AStar::AStar(Grid& grid) : grid(grid) { }

std::vector<Node> AStar::findPath(const Node& start, const Node& end) {
    auto cmp = [](Node* left, Node* right) { return *left > *right; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> openList(cmp);

    std::unordered_map<int, Node*> allNodes;

    auto hash = [&](int x, int y) { return x * 1000 + y; };

    Node* startNode = new Node(start.x, start.y, 0, heuristic(start, end));
    openList.push(startNode);
    allNodes[hash(start.x, start.y)] = startNode;

    std::unordered_map<int, bool> closedList;

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (*current == end) {
            return reconstructPath(current);
        }

        closedList[hash(current->x, current->y)] = true;

        for (const auto& [dx, dy] : directions) {
            int nx = current->x + dx;
            int ny = current->y + dy;

            if (!grid.isValid(nx, ny) || closedList[hash(nx, ny)] || grid.getExploredCellState(nx, ny) == OBSTACLE) {
                continue;
            }

            float newG = current->cost + 1;

            Node* neighbor = allNodes[hash(nx, ny)];
            if (neighbor == nullptr) {
                neighbor = new Node(nx, ny, newG, heuristic({nx, ny}, end), current);
                allNodes[hash(nx, ny)] = neighbor;
                openList.push(neighbor);
            } else if (newG < neighbor->cost) {
                neighbor->cost = newG;
                neighbor->parent = current;
                openList.push(neighbor);
            }
        }
    }

    return {};
}

void AStar::exploreAndMove(Node& current, const Node& end) {
    for (const auto& [dx, dy] : directions) {
        int nx = current.x + dx;
        int ny = current.y + dy;
        grid.updateExploredCell(nx, ny);
    }

    auto path = findPath(current, end);
    if (!path.empty()) {
        current = path[1];
        grid.displayExploredMap(current.x, current.y);
    } else {
        std::cout << "No path found!\n";
    }
}

float AStar::heuristic(const Node& a, const Node& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<Node> AStar::reconstructPath(Node* endNode) {
    std::vector<Node> path;
    for (Node* node = endNode; node != nullptr; node = node->parent) {
        path.push_back(*node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace bilu
