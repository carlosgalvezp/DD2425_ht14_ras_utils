#include <ras_utils/graph/bfs_planner.h>

BFS_Planner::BFS_Planner()
{
}

BFS_Planner::BFS_Planner(const Graph &graph)
    : graph_(graph){}

double BFS_Planner::computePath(const Node &start, const Node &end, std::queue<Node> &path)
{

    std::cerr <<"BFS_Planner::computePath" <<std::endl;
    return 0;
}
