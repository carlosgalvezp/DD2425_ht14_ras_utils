#include <ras_utils/graph/edge.h>

Edge::Edge()
{
    p1_ = Node(0,0, ' ');
    p2_ = Node(0,0, ' ');
    cost_ = 0;
}

Edge::Edge(const Node& p1, const Node& p2, double cost)
{
    p1_ = p1;
    p2_ = p2;
    cost_ = cost;
}

const Node& Edge::getP1() const {    return p1_;}
const Node& Edge::getP2() const {    return p2_;}
double Edge::getCost()    const {    return cost_;}
