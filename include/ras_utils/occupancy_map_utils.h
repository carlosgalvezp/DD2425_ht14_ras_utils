#ifndef RAS_OCCUPANCY_MAP_UTILS_H
#define RAS_OCCUPANCY_MAP_UTILS_H

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int64MultiArray.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <set>

#define OCC_GRID_SIMPLE_UNKNOWN_AREA 50
#define OCC_GRID_SIMPLE_FREE_AREA    0
#define OCC_GRID_SIMPLE_BLOCKED_AREA 100

namespace RAS_Utils
{
namespace occ_grid
{

void convertToMatrixPos(const nav_msgs::OccupancyGrid & grid_data, int & i, int & j, double x, double y)
{
    int height = grid_data.info.height;
    int width = grid_data.info.width;
    double resolution = grid_data.info.resolution;
    double offset_x = grid_data.info.origin.position.x;
    double offset_y = grid_data.info.origin.position.y;

    double real_i = x / resolution;
    double real_j = y / resolution;

    real_i -= offset_x / resolution;
    real_j -= offset_y / resolution;

    i = real_i + 0.5;
    j = real_j + 0.5;
}

int getValue(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    int i, j;
    convertToMatrixPos(grid_data, i, j, x, y);
    return(grid_data.data[i + grid_data.info.height*j]);
}

void convertToRealPos(const nav_msgs::OccupancyGrid & grid_data, double & x, double & y, int i, int j)
{
    int height = grid_data.info.height;
    int width = grid_data.info.width;
    double resolution = grid_data.info.resolution;
    double offset_x = grid_data.info.origin.position.x;
    double offset_y = grid_data.info.origin.position.y;

    double real_i = i + offset_x / resolution;
    double real_j = j + offset_y / resolution;

    x = real_i * resolution;
    y = real_j * resolution;
}

void convertToRealPos(const nav_msgs::OccupancyGrid &grid_data, double &x, double &y, int index)
{

    int i, j;
    i = index % grid_data.info.height;
    j = index / grid_data.info.height;
    convertToRealPos(grid_data, x, y, i, j);
}

bool isUnknown(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) == OCC_GRID_SIMPLE_UNKNOWN_AREA;
}

bool isFree(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) < OCC_GRID_SIMPLE_UNKNOWN_AREA;
}

bool isWall(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) == OCC_GRID_SIMPLE_BLOCKED_AREA;
}

namespace bfs_search
{

    struct Traveler {
      int from_index;
      int index;
      int cost;
      int nr_points;

      Traveler(int from_index, int index, int cost, int nr_points) : from_index(from_index), index(index), cost(cost), nr_points(nr_points){}
    } ;

  struct CostStepInfo {
        int index;
        int step;

        CostStepInfo(int index, int step) : index(index), step(step){}
  };


    class CompareTravelers {
    public:
        bool operator()(Traveler& t1, Traveler& t2)
        {
           return (t1.cost > t2.cost);
        }
    };



    std::vector<int> getClosest4Indexes(const nav_msgs::OccupancyGrid & occ_grid, int index)
    {
        std::vector<int> return_vector;
        return_vector.push_back(index + 1);
        return_vector.push_back(index - 1);
        return_vector.push_back(index + occ_grid.info.height);
        return_vector.push_back(index - occ_grid.info.height);
        return return_vector;
    }

    std::vector<geometry_msgs::Point> run(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, int start_i, int start_j, std::function<bool(int) > & stopFunction)
    {
        int index = start_i + start_j * occ_grid.info.height;

        std::vector<long> visited_cost_map(occ_grid.info.height * occ_grid.info.width, -1);
        std::vector<int> cheapest_next_neighbour(occ_grid.info.height * occ_grid.info.width);
        std::priority_queue<Traveler, std::vector<Traveler>, CompareTravelers > bfs_queue;
        //std::queue<Traveler > bfs_queue;
        long cheapest_route_cost = -1;
        int end_index = -1;
        int cheapest_route_nr_points = 0;
        // Add the first to queue;
        bfs_queue.push(Traveler(-1, index, 0, 1)); // minus one simply indicates its the start index

        long prevCost= -1;
        std::vector<int> neighbours;

        while( !bfs_queue.empty() )
        {


            Traveler current_traveler = bfs_queue.top();
            bfs_queue.pop();
            prevCost = current_traveler.cost;

            long current_cell_cost = cost_grid.data[current_traveler.index];

            long new_traveler_cost = current_traveler.cost + current_cell_cost;

           // std::cout << current_traveler.index << " cell cost " << current_cell_cost << std::endl;

            if((visited_cost_map[current_traveler.index] != -1 && new_traveler_cost >= visited_cost_map[current_traveler.index]) || (cheapest_route_cost != -1 && new_traveler_cost >= cheapest_route_cost))
            {
                // The node we are visiting has been visited with a lower cost or we have found a route and the current cost is greater than that one
                continue;
            }


            visited_cost_map[current_traveler.index] = new_traveler_cost;
            cheapest_next_neighbour[current_traveler.index] = current_traveler.from_index;

            if( stopFunction(current_traveler.index) )
            {
                end_index = current_traveler.index;
                cheapest_route_cost = new_traveler_cost;
                cheapest_route_nr_points = current_traveler.nr_points;
                // we have found what we search for, abort
                continue;
            }

            if(occ_grid.data[current_traveler.index] == OCC_GRID_SIMPLE_UNKNOWN_AREA || occ_grid.data[current_traveler.index] == OCC_GRID_SIMPLE_BLOCKED_AREA )
            {
                // abort, we are hitting unknown or wall
                continue;
            }

            // Not found a goal yet.
            neighbours = getClosest4Indexes(occ_grid, current_traveler.index);
            for(int neighbour : neighbours)
            {
                bfs_queue.push(Traveler(current_traveler.index, neighbour, new_traveler_cost, current_traveler.nr_points + 1));
            }
        }

        std::vector<geometry_msgs::Point> best_path(cheapest_route_nr_points);
        int next_index = end_index;
        int vector_pos = cheapest_route_nr_points - 1;
        while(next_index != -1)
        {
            geometry_msgs::Point point;
            double real_x, real_y;
            convertToRealPos(occ_grid, real_x, real_y, next_index);
            point.x = real_x;
            point.y = real_y;
            best_path[vector_pos] = point;
            next_index = cheapest_next_neighbour[next_index];
            vector_pos--;
        }
        return best_path;
    }



    std::vector<geometry_msgs::Point> getClosestUnknownPath(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, double start_x, double start_y)
    {
        int i, j;
        convertToMatrixPos(occ_grid, i, j, start_x, start_y);
        std::function<bool(int) > stopFunction = [&occ_grid](int index){
            return occ_grid.data[index] == OCC_GRID_SIMPLE_UNKNOWN_AREA;
        };

        return run(occ_grid, cost_grid, i, j, stopFunction);
    }

    std::vector<geometry_msgs::Point> getPathFromTo(const nav_msgs::OccupancyGrid & occ_grid, const std_msgs::Int64MultiArray & cost_grid, double from_x, double from_y, double to_x, double to_y)
    {
        int from_i, from_j, to_i, to_j;
        convertToMatrixPos(occ_grid, from_i, from_j, from_x, from_y);
        convertToMatrixPos(occ_grid, to_i, to_j, to_x, to_y);

        std::function<bool(int) > stopFunction = [&occ_grid, &to_i, &to_j](int index)
        {
            return to_i + to_j * occ_grid.info.height == index;
        };

        return run(occ_grid, cost_grid, from_i, from_j, stopFunction);
    }



}}}




#endif // RAS_OCCUPANCY_MAP_UTILS_H
