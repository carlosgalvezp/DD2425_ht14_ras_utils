#ifndef GRAPH_H
#define GRAPH_H

#include <ras_utils/graph/edge.h>
#include <algorithm>
#include <vector>
#include <fstream>

#include <Eigen/Core>
class Graph
{
public:
    Graph();
    Graph(const Graph& src);
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
//     Reads the objects graph, assuming the following format:
//
//        N               (number of nodes. Should include the starting position twice)
//        n0_x n1_y       (should be starting position, 0,0)
//        n1_x n2_y       (an object position)
//        n2_x n3_y       (an object position)
//        ...
//        nN-1_x nN-1_y   (should be starting position, 0,0)
//        n0 n1 c01           (cost c01 from node 0 to node 1)
//        n0 n2 c02           (cost c02 from node 0 to node 2)
//        ...
//        n1 n2 c12           (cost c12 from node 1 to node 2)
//        n1 n3 c13           (cost c13 from node 1 to node 3)
//        ...
//        nN-2 nN-1 cN-2N-1   (cost cN-2N-1 from node N-2 to node N-1)

    void readGraph(const std::string &path, Graph &graph);

}


#endif // GRAPH_H
