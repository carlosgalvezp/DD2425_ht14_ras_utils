#include <ras_utils/graph/dfs_planner.h>

#include <ras_utils/graph/edge.h>

DFS_Planner::DFS_Planner() {}
DFS_Planner::DFS_Planner(const Graph &graph) : graph_(graph) {}

double DFS_Planner::computePath(const Node &start, const Node &end, std::vector<Node> &optimal_path)
{
    double optimal_path_cost;
    dfsSearch(start, end, optimal_path, optimal_path_cost);
    return optimal_path_cost;
}

void DFS_Planner::dfsSearch(const Node &start, const Node &end, std::vector<Node> &optimal_path, double &optimal_path_cost)
{

    std::vector<Node> current_path;
    current_path.push_back(start);
    std::vector<Node> optimal_path_temp;
    std::map<int, double> stored_costs;
    stored_costs[start.getID()] = 0;
    dfsSearch(graph_, start, end, current_path, 0, optimal_path_temp, stored_costs);
    optimal_path = optimal_path_temp;
    optimal_path_cost = stored_costs[end.getID()];
}

void DFS_Planner::dfsSearch(const Graph &graph,
                            const Node & current_node,
                            const Node & end_node,
                            std::vector<Node> &current_path,
                            double current_cost,
                            std::vector<Node> &optimal_path,
                            std::map<int, double> & stored_costs
                            )
{
    if(current_node == end_node) {
        // Found a new best route
        optimal_path = current_path;
        return;
    }

    double new_cost;

    std::vector<Edge> current_edges = graph.getEdges(current_node);
    for (std::vector<Edge>::const_iterator it = current_edges.begin() ; it != current_edges.end(); ++it)
    {
        const Edge edge = *it;
        new_cost = current_cost + edge.getCost();
       std::map<int, double>::iterator it = stored_costs.find(end_node.getID());
        if(it == stored_costs.end() || current_cost < it->second)
            {
            // The new score for the next node is better than the optimal score, continue
            const Node & neighbour_node = edge.getOther(current_node);

            it = stored_costs.find(neighbour_node.getID());
            if(it == stored_costs.end() || new_cost < it->second)
            {
                std::cout << "Current ID:" << current_node.getID() << "Neighbour node ID: " << neighbour_node.getID() << " new_cost: " << new_cost << std::endl;
                // The new score for the next node is the best score for this node so far, continue
                stored_costs[neighbour_node.getID()] = new_cost;
                current_path.push_back(neighbour_node);
                dfsSearch(graph, neighbour_node, end_node, current_path, new_cost, optimal_path, stored_costs);
                current_path.pop_back();
            }
        }
    }
}
