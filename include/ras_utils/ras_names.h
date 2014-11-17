#ifndef RAS_NAMES_H
#define RAS_NAMES_H
#include <string>

// In this file we define the name of all the nodes and topics. Convention:
//    NODE_NODE_NAME
//    TOPIC_TOPIC_NAME
//    SRV_SERVICE_NAME


// Node names
#define NODE_BRAIN                      "brain"
#define NODE_MOTOR_CONTROLLER           "motor_controller"
#define NODE_NAVIGATION                 "navigation"

// Topic names
#define TOPIC_ARDUINO_ADC               "/arduino/adc"
#define TOPIC_ODOMETRY                  "/robot/odometry"
#define TOPIC_ARDUINO_PWM               "/arduino/pwm"
#define TOPIC_ARDUINO_ENCODERS          "/arduino/encoders"
#define TOPIC_MAP_OCC_GRID              "/map/occ_grid"

#define TOPIC_MOTOR_CONTROLLER_TWIST    "/motor_controller/twist"

// Servers names
#define SRV_BRAIN_IN                    "/brain/comm"
#define SRV_NAVIGATION_IN               "/navigation/comm"

// ===========================================================================
// ===========================================================================

// Paths where we store or read data
namespace RAS_Names
{
const std::string models_3D_path = (std::string(getenv("HOME")) + std::string("/3d_data/"));

}

#endif
