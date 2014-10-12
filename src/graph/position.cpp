#include <ras_utils/graph/position.h>

Position::Position()
{
}

Position::Position(double x, double y)
{
    x_ = x;
    y_ = y;
}

double Position::euclideanDistance(Position p1, Position p2)
{
    return sqrt( (p1.x_ - p2.x_)*(p1.x_ - p2.x_) +
                 (p1.y_ - p2.y_)*(p1.y_ - p2.y_) );
}
