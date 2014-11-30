#ifndef GRAPH_H
#define GRAPH_H

#include <ras_utils/graph/edge.h>
#include <algorithm>
#include <vector>

#include <Eigen/Core>
class Graph
{
public:
    Graph();
    Graph(const std::vector<Node> &nodes, const std::vector<Edge> &edges);
    Graph(const std::vector<Node> &nodes, const std::vector<Edge> &edges, const Eigen::MatrixXd &costs);

    void computeCostMatrix(const std::vector<Node> &nodes, const std::vector<Edge> &edges, Eigen::MatrixXd &costs);

    std::vector<Node> getNodes() const;
    std::vector<Edge> getEdges() const;
    std::size_t getNodeCount()   const;
    std::size_t getEdgeCount()   const;

    std::vector<Edge> getEdges(const Node &node) const;

    void setNodes(std::vector<Node> nodes);
    void setEdges(std::vector<Edge> edges);

    double computePathCost(std::vector<int>& path) const;
private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

    Eigen::MatrixXd cost_matrix_;
};

namespace Graph_Utils
{
    void readGraph(const std::string &path, Graph &graph);

}


#endif // GRAPH_H
