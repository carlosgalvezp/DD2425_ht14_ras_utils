#ifndef NODE_H
#define NODE_H

#include <ras_utils/graph/position.h>

class Node
{
public:
    Node();
    Node(const Position &pos, int id);
    Node(double x, double y, int id);

    Position getPosition();
    int getID();
private:
    Position pos_;
    int id_;
};

#endif // NODE_H
