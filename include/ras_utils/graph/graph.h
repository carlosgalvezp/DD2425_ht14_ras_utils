#ifndef GRAPH_H
#define GRAPH_H

#include <ras_utils/graph/edge.h>

#include <vector>

#include <Eigen/Core>
class Graph
{
public:
    Graph();
    Graph(std::vector<Node> nodes, std::vector<Edge> edges, Eigen::MatrixXd costs);

    std::vector<Node> getNodes() const;
    std::vector<Edge> getEdges() const;
    std::size_t getNodeCount()   const;
    std::size_t getEdgeCount()   const;

    void setNodes(std::vector<Node> nodes);
    void setEdges(std::vector<Edge> edges);

    double computePathCost(std::vector<int>& path) const;

private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

    Eigen::MatrixXd cost_matrix_;
};

#endif // GRAPH_H
