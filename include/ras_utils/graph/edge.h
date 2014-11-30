#ifndef EDGE_H
#define EDGE_H

#include <ras_utils/graph/node.h>

class Edge
{
public:
    Edge();
    Edge(const Node &p1, const Node &p2, double cost);

    const Node& getP1() const;
    const Node& getP2() const;
    double getCost()    const;

    // If parameter is P1 then P2 is returned and vice versa
    const Node& getOther(const Node &node) const;
private:
    Node p1_, p2_;
    double cost_;
};

#endif // EDGE_H
