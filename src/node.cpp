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

Direction Node::operator-(const Node& other) const {
    if (y == other.y) {
        return x > other.x ? UP : DOWN;
    }

    return y > other.y ? LEFT : RIGHT;
}

}  // namespace bilu
