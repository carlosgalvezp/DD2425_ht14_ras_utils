#include <ras_utils/graph/graph.h>

Graph::Graph()
{
}

Graph::Graph(std::vector<Node> nodes, std::vector<Edge> edges, Eigen::MatrixXd cost_matrix)
{
    nodes_ = nodes;
    edges_ = edges;

    cost_matrix_ = cost_matrix;
}

std::vector<Node> Graph::getNodes() const   { return nodes_;       }
std::vector<Edge> Graph::getEdges() const   { return edges_;       }
std::size_t Graph::getNodeCount()   const   { return nodes_.size();}
std::size_t Graph::getEdgeCount()   const   { return edges_.size();}

void Graph::setNodes(std::vector<Node> nodes){ nodes_ = nodes;}
void Graph::setEdges(std::vector<Edge> edges){ edges_ = edges;}

double Graph::computePathCost(std::vector<int>& path) const
{
    double cost = 0.0;

    for(unsigned int i = 0; i < path.size()-1; i++)
    {
        cost += cost_matrix_(path.at(i), path.at(i+1));
    }
    return cost;
}
