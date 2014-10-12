#ifndef EDGE_H
#define EDGE_H

#include <ras_utils/graph/node.h>

class Edge
{
public:
    Edge();
    Edge(Node p1, Node p2, double cost);

    Node getP1();
    Node getP2();
    double getCost();
private:
    Node p1_, p2_;
    double cost_;
};

#endif // EDGE_H
