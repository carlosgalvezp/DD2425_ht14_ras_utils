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

Node::Node(const Position& pos, int id, bool is_object)
{
    pos_ = Position(pos.x_, pos.y_);
    id_ = id;
    is_object_ = is_object;
}

Node::Node(double x, double y, int id)
{
    pos_ = Position(x,y);
    id_ = id;
}

Position Node::getPosition() const {   return pos_; }
int      Node::getID()       const {   return id_;  }
bool     Node::isObject()    const {   return is_object_;}

bool Node::operator ==(const Node &obj) const
{
    return this->pos_ == obj.pos_;
}


std::ostream &operator<<(std::ostream &os, const Node &node)
{
    os << node.getPosition().x_ << " "
       << node.getPosition().y_ << " "
       << node.getID() << std::endl;
    return os;
}

