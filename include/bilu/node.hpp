#ifndef NODE_HPP
#define NODE_HPP

namespace bilu {

struct Node {
    int   x, y;
    float cost, heuristic;
    Node* parent;

    Node(int x, int y, float cost = 0, float heuristic = 0, Node* parent = nullptr);

    float f() const;

    bool operator>(const Node& other) const;
    bool operator==(const Node& other) const;
    bool operator!=(const Node& other) const;

};

}  // namespace bilu
#endif  // NODE_HPP
