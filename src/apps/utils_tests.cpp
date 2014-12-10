#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include <sys/time.h>

#include <opencv2/opencv.hpp>

#include <ras_utils/graph/graph.h>
#include <ras_utils/genetic/genetic_algorithm.h>
#include <ras_utils/ras_utils.h>
// Graphical params
#define X_MIN 0
#define X_MAX 1000
#define Y_MIN 0
#define Y_MAX 1000

#define N_NODES 3


void createGraph(double x_min, double x_max, double y_min, double y_max,
                 unsigned int n_nodes, Graph& graph);

void visualizeGraph(Graph& graph);
void visualizeSolution(const Graph &graph, const std::vector<Node> &path);

Position createPosition(double x_min, double x_max, double y_min, double y_max);
double randDouble(double x_min, double x_max);

void testTSP();
void testVector();

int main()
{
//    testVector();
    testTSP();
    return 0;
}

void testVector()
{
    std::vector<std::string> v {"cube","cube","ball","cylinder", "ball", "triangle", "ball"};
    std::cout <<RAS_Utils::get_most_repeated<std::string>(v)<<std::endl;
}
void testTSP()
{
    // ** Set random seed
    srand(std::time(NULL));

    // ** Create a graph
    Graph graph;
    createGraph(X_MIN, X_MAX, Y_MIN, Y_MAX, N_NODES, graph);
//    Graph_Utils::readGraph(RAS_Names::OBJECT_GRAPH_PATH, graph);

    // ** Visualize the graph
    visualizeGraph(graph);

    // ** Run the genetic algoritm
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);

    GeneticAlgorithm gm(graph);
    std::vector<Node> solution;
    gm.computeSolution(solution);

    gettimeofday(&t2,NULL);
    std::cout << "Genetic Algorithm: "<< (t2.tv_sec - t1.tv_sec) +
                                         (t2.tv_usec - t1.tv_usec)/1000000.0 << " seconds" << std::endl;

    // ** Visualize solution
    visualizeSolution(graph, solution);
}

void createGraph(double x_min, double x_max, double y_min, double y_max,
                 unsigned int n_nodes, Graph& graph)
{
    std::cout << "Creating graph..." << std::endl;
    // ** Create the nodes
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    for (unsigned int i = 0; i < n_nodes; i++)
    {
        Position p;
        if (i == 0)
            p = Position(0,0);
        else
            p = createPosition(x_min, x_max, y_min, y_max);

        Node n(p, i);
        nodes.push_back(n);
    }

    // ** Create the edges
    Eigen::MatrixXd cost_m(nodes.size(), nodes.size());
    for (unsigned int i = 0; i < n_nodes; i++)
    {
        for(unsigned int j = i+1; j < n_nodes; j++)
        {
            Node n1 = nodes.at(i);
            Node n2 = nodes.at(j);
            double cost = Position::euclideanDistance(n1.getPosition(), n2.getPosition());
            Edge e(n1, n2, cost);
            edges.push_back(e);

            // Update the cost matrix
            cost_m(i,j) = cost;
            cost_m(j,i) = cost;
        }
    }
    graph = Graph(nodes, edges, cost_m);
}

Position createPosition(double x_min, double x_max, double y_min, double y_max)
{
    double x = randDouble(x_min, x_max);
    double y = randDouble(y_min, y_max);
    Position p(x,y);
    return p;
}

double randDouble(double x_min, double x_max)
{
    double x = (double) rand() / RAND_MAX;
    return x_min + x * (x_max - x_min);
}

void visualizeGraph(Graph& graph)
{
    // ** Initialize Mat image
    cv::Mat img(X_MAX, Y_MAX, CV_8UC3);

    // ** Draw lines
    for (std::size_t i = 0; i < graph.getEdgeCount(); i++)
    {
        Edge e = graph.getEdges().at(i);
        cv::line(img, cv::Point(e.getP1().getPosition().x_, e.getP1().getPosition().y_),
                      cv::Point(e.getP2().getPosition().x_, e.getP2().getPosition().y_), cv::Scalar(255,255,255));
    }

    // ** Draw nodes and text
    for (std::size_t i = 0; i < graph.getNodeCount(); i++)
    {
        Node n = graph.getNodes().at(i);
        cv::circle(img, cv::Point(n.getPosition().x_, n.getPosition().y_),
                   5, cv::Scalar(0,0,255), -1);

        std::stringstream ss;
        ss << n.getID();
        cv::putText(img, ss.str(),
                    cv::Point(n.getPosition().x_, n.getPosition().y_),
                    CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
    }

    cv::namedWindow("Graph", CV_WINDOW_AUTOSIZE);
    cv::imshow("Graph", img);
    cv::waitKey();
    cv::destroyWindow("Graph");
}

void visualizeSolution(const Graph& graph, const std::vector<Node> &path)
{
    // ** Visualize path    
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        std::cout << path[i].getID();
        if (i < path.size()-1)
             std::cout<< "->";
        else
            std::cout << std::endl;
    }

    // ** Initialize Mat image
    cv::Mat img(X_MAX, Y_MAX, CV_8UC3, cv::Scalar(0,0,0));

    // ** Draw lines
    for (std::size_t i = 0; i < graph.getEdgeCount(); i++)
    {
        Edge e = graph.getEdges().at(i);
        cv::line(img, cv::Point(e.getP1().getPosition().x_, e.getP1().getPosition().y_),
                      cv::Point(e.getP2().getPosition().x_, e.getP2().getPosition().y_), cv::Scalar(255,255,255));
    }

    // ** Draw solution
    for (std::size_t i = 0; i < path.size()-1; i++)
    {
        Position pos1 = path[i].getPosition();
        Position pos2 = path[i+1].getPosition();

        cv::line(img, cv::Point(pos1.x_, pos1.y_),
                      cv::Point(pos2.x_, pos2.y_), cv::Scalar(255,0,0),3);
    }

    // ** Draw nodes and text
    for (std::size_t i = 0; i < graph.getNodeCount(); i++)
    {
        Node n = graph.getNodes().at(i);
        cv::circle(img, cv::Point(n.getPosition().x_, n.getPosition().y_),
                   5, cv::Scalar(0,0,255), -1);

        std::stringstream ss;
        ss << n.getID();
        cv::putText(img, ss.str(),
                    cv::Point(n.getPosition().x_, n.getPosition().y_),
                    CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
    }


    cv::namedWindow("Solution", CV_WINDOW_AUTOSIZE);
    cv::imshow("Solution", img);
    cv::waitKey();
}
