#include "bilu/node.hpp"

namespace bilu {

Node::Node(int x, int y, float cost, float heuristic, Node* parent) :
    x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) { }

float Node::f() const {
    return cost + heuristic;
}

bool Node::operator>(const Node& other) const {
    return f() > other.f();
}

bool Node::operator==(const Node& other) const {
    return x == other.x && y == other.y;
}

bool Node::operator!=(const Node& other) const {
    return !(*this == other);
}

}  // namespace bilu
