#ifndef RAS_OCCUPANCY_MAP_UTILS_H
#define RAS_OCCUPANCY_MAP_UTILS_H

#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <fstream>

#define OCC_GRID_SIMPLE_UNKNOWN_AREA 50
#define OCC_GRID_SIMPLE_FREE_AREA    0
#define OCC_GRID_SIMPLE_BLOCKED_AREA 100

namespace RAS_Utils
{
namespace occ_grid
{

int getValue(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    int height = grid_data.info.height;
    int width = grid_data.info.width;
    double resolution = grid_data.info.resolution;
    double offset_x = grid_data.info.origin.position.x;
    double offset_y = grid_data.info.origin.position.y;

    double real_i = x / resolution;
    double real_j = y / resolution;

    real_i -= offset_x;
    real_j -= offset_y;

    int i = real_i + 0.5;
    int j = real_j + 0.5;

    return(grid_data.data[i + height*j]);
}

bool isUnknown(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) == 50;
}

bool isFree(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) == 0;
}

bool isWall(const nav_msgs::OccupancyGrid & grid_data, double x, double y)
{
    return getValue(grid_data, x, y) == 100;
}

void saveMap(const nav_msgs::OccupancyGrid &grid_data, const std::string &path)
{
    // Format:
    // height width resolution origin_x origin_y
    // data[0]
    // data[1]
    // ...
    // data[N-1]
    std::ofstream file;
    file.open(path);
    file << grid_data.info.height               << " "
         << grid_data.info.width                << " "
         << grid_data.info.resolution           << " "
         << grid_data.info.origin.position.x    << " "
         << grid_data.info.origin.position.y;

    std::size_t N = grid_data.info.width * grid_data.info.height / grid_data.info.resolution;

    for(unsigned int i = 0; i < N; ++i)
    {
        file << grid_data.data[i] << "\n";
    }

    file.close();
}

void loadMap(const std::string &path, nav_msgs::OccupancyGrid &grid_data)
{
    // Format:
    // height width resolution origin_x origin_y
    // data[0]
    // data[1]
    // ...
    // data[N-1]
    std::ifstream file;
    file.open(path);
    file >> grid_data.info.height
         >> grid_data.info.width
         >> grid_data.info.resolution
         >> grid_data.info.origin.position.x
         >> grid_data.info.origin.position.y;

    std::size_t N = grid_data.info.width * grid_data.info.height / grid_data.info.resolution;

    for(unsigned int i = 0; i < N; ++i)
    {
        file >> grid_data.data[i];
    }

    file.close();
}

}}




#endif // RAS_OCCUPANCY_MAP_UTILS_H
