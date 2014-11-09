#ifndef NODE_COMMUNICATION_H
#define NODE_COMMUNICATION_H

//Constant variables to define a communication protocol, general form:

//    COMM_NODE-FROM_NODE-TO_ACTION VALUE

//COMM            is compulsory
//NODE_FROM       node sending the command
//NODE_TO         node receiving the command
//ACTION          action that the receiving node will perform
//VALUE           an integer different from the previous ones

struct Node_Communication
{
    static const int COMM_BRAIN_NAVIGATION_WALL = 1;
    static const int COMM_BRAIN_NAVIGATION_STOP = 2;
};



#endif // NODE_COMMUNICATION_H
