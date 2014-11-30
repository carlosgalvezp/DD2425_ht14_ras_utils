#ifndef POSITION_H
#define POSITION_H

#include <cmath>

class Position
{
public:
    Position();
    Position(double x, double y);

    static double euclideanDistance(Position p1, Position p2);

    bool operator==(const Position &obj) const;

    double x_;
    double y_;
};



#endif // POSITION_H
