#ifndef DFS_PLANNER_H
#define DFS_PLANNER_H

#include <ras_utils/graph/graph.h>
#include <vector>
#include <map>


class DFS_Planner
{
public:
    DFS_Planner();
    DFS_Planner(const Graph &graph);

    double computePath(const Node &start, const Node &end, std::vector<Node> &path);
private:
    Graph graph_;

    void dfsSearch(const Node &start, const Node &end, std::vector<Node> &optimal_path, double &optimal_path_cost);
    void dfsSearch(const Graph &graph,
                                const Node & current_node,
                                const Node & end_node,
                                std::vector<Node> &current_path,
                                double current_cost,
                                std::vector<Node> &optimal_path,
                                std::map<int, double> & stored_costs
                                );
};

#endif // DFS_PLANNER_H
