#ifndef NODE_H
#define NODE_H

#include <ras_utils/graph/position.h>
#include <iostream>

class Node
{
public:
    Node();
    Node(const Position &pos, int id);
    Node(const Position& pos, int id, bool is_object);
    Node(double x, double y, int id);

    const Position& getPosition() const;
    int getID()                   const;
    bool isObject()               const;

    bool operator==(const Node &other) const;
private:
    Position pos_;
    int id_;
    bool is_object_;
    friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

std::ostream &operator<<(std::ostream &os, const Node &node);
#endif // NODE_H
