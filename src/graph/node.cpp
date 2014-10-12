#include <ras_utils/graph/node.h>

Node::Node()
{
    pos_ = Position(0,0);
    id_ = 0;
}

Node::Node(const Position& pos, int id)
{
    pos_ = Position(pos.x_, pos.y_);
    id_ = id;
}

Node::Node(double x, double y, int id)
{
    pos_ = Position(x,y);
    id_ = id;
}

Position Node::getPosition(){   return pos_; }
int Node::getID(){ return id_; }


