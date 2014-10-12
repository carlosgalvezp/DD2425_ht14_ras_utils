#include <ras_utils/graph/edge.h>

Edge::Edge()
{
    p1_ = Node(0,0, ' ');
    p2_ = Node(0,0, ' ');
    cost_ = 0;
}

Edge::Edge(Node p1, Node p2, double cost)
{
    p1_ = p1;
    p2_ = p2;
    cost_ = cost;
}

Node Edge::getP1(){    return p1_;}
Node Edge::getP2(){    return p2_;}
double Edge::getCost(){    return cost_;}
