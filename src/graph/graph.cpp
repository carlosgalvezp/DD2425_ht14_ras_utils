#include <ras_utils/graph/graph.h>

Graph::Graph()
{
}

Graph::Graph(const Graph& src)
{
    this->nodes_ = src.nodes_;
    this->edges_ = src.edges_;
    this->cost_matrix_ = src.cost_matrix_;
}

Graph::Graph(const std::vector<Node> &nodes, const std::vector<Edge> &edges)
{
    this->nodes_ = nodes;
    this->edges_ = edges;

    // ** Compute cost matrix
    computeCostMatrix(nodes, edges,this->cost_matrix_);
    std::cout << "COST MATRIX\n" << cost_matrix_<<std::endl;
}

Graph::Graph(const std::vector<Node> &nodes, const std::vector<Edge> &edges, const Eigen::MatrixXd &cost_matrix)
{
    this->nodes_ = nodes;
    this->edges_ = edges;
    this->cost_matrix_ = cost_matrix;
}

void Graph::computeCostMatrix(const std::vector<Node> &nodes, const std::vector<Edge> &edges, Eigen::MatrixXd &costs)
{
    // ** Init to infinite cost
    costs = std::numeric_limits<double>::infinity()*Eigen::MatrixXd::Identity(nodes.size(), nodes.size());

    // ** Loop over the edges and update the cost
    for(std::size_t i = 0; i < edges.size(); ++i)
    {
        const Edge &e = edges[i];
        const Node &n1 = e.getP1();
        const Node &n2 = e.getP2();

        costs(n1.getID(),n2.getID()) = e.getCost();
        costs(n2.getID(),n1.getID()) = e.getCost();
    }
}

std::vector<Node> Graph::getNodes() const   { return nodes_;       }
std::vector<Edge> Graph::getEdges() const   { return edges_;       }
std::size_t Graph::getNodeCount()   const   { return nodes_.size();}
std::size_t Graph::getEdgeCount()   const   { return edges_.size();}

std::vector<Edge> Graph::getEdges(const Node &node) const
{
    std::vector<Edge> return_edges;
    for (std::vector<Edge>::const_iterator it = edges_.begin() ; it != edges_.end(); ++it)
    {
        const Edge & edge = *it;
        if(edge.getP1() == node || edge.getP2() == node)
        {
            return_edges.push_back(edge);
        }
    }
    return return_edges;
}

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

namespace Graph_Utils
{
    void readGraph(const std::string &path, Graph &graph)
    {
        std::vector<Node> nodes;
        std::vector<Edge> edges;

        std::ifstream file;
        file.open(path);

        // ** Number of nodes
        int N;
        file >> N;

        // ** Nodes' positions
        for(std::size_t i = 0; i < N; ++i)
        {
            double x,y;
            file >> x >> y;
            Node n(x,y,i);
            nodes.push_back(n);
        }

        // ** Edges
        int n_edges = N*(N-1)/2.0;
        for(std::size_t i = 0; i < n_edges; ++i)
        {
            int n1, n2;
            double cost;
            file >> n1 >> n2 >> cost;
            Edge e(nodes[n1], nodes[n2], cost);
            std::cout << "CREATING EDGE BETWEEN "<<n1 << " and " << n2<<"; Cost: "<<cost<<std::endl;
            edges.push_back(e);
        }

        // ** Create graph
        graph = Graph(nodes, edges);

        std::clog << "Created a graph with " << graph.getNodeCount() << " nodes and "
                  << graph.getEdgeCount() << " edges" << std::endl;
    }

}
