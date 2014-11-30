#ifndef BFS_PLANNER_H
#define BFS_PLANNER_H

#include <ras_utils/graph/graph.h>
#include <queue>

class BFS_Planner
{
public:
    BFS_Planner();
    BFS_Planner(const Graph &graph);

    double computePath(const Node &start, const Node &end, std::queue<Node> &path);
private:
    Graph graph_;
};

#endif // BFS_PLANNER_H
