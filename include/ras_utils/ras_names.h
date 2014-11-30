#ifndef RAS_NAMES_H
#define RAS_NAMES_H
#include <string>

#define GROUP_NUMBER 6
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
#define TOPIC_MAP_OCC_GRID              "/map/occ_grid_rviz"
#define TOPIC_MAP_OCC_GRID_THICK        "/map/occ_grid_thick"

#define TOPIC_SPEAKER                   "/espeak/string"
#define TOPIC_EVIDENCE                  "/evidence"
#define TOPIC_MOTOR_CONTROLLER_TWIST    "/motor_controller/twist"
#define TOPIC_CAMERA_RGB                "/camera/rgb/image_rect_color"
#define TOPIC_CAMERA_DEPTH              "/camera/depth_registered/hw_registered/image_rect_raw"
#define TOPIC_OBSTACLE                  "/obstacle"
#define TOPIC_IMU                       "/imu/data"
#define TOPIC_MARKERS                   "/map_markers"

// Servers names
#define SRV_BRAIN_IN                    "/brain/comm"
#define SRV_NAVIGATION_IN               "/navigation/comm"

// 3D Shapes
#define SHAPE_3D_CUBE      0
#define SHAPE_3D_BALL      1
#define SHAPE_3D_OTHER     2

// Colors
#define COLOR_RED       0
#define COLOR_GREEN     1
#define COLOR_BLUE      2
#define COLOR_YELLOW    3
#define COLOR_PURPLE    4

// Object IDs
#define OBJECT_NAME_RED_CUBE         "Red Cube"
#define OBJECT_NAME_BLUE_CUBE        "Blue Cube"
#define OBJECT_NAME_GREEN_CUBE       "Green Cube"
#define OBJECT_NAME_YELLOW_CUBE      "Yellow Cube"
#define OBJECT_NAME_YELLOW_BALL      "Yellow Ball"
#define OBJECT_NAME_RED_BALL         "Red Ball"
#define OBJECT_NAME_GREEN_CYLINDER   "Green Cylinder"
#define OBJECT_NAME_BLUE_TRIANGLE    "Blue Triangle"
#define OBJECT_NAME_PURPLE_CROSS     "Purple Cross"
#define OBJECT_NAME_PATRIC           "Patric"
#define OBJECT_NAME_UNKNOWN          "An Object"

// Object indices
#define OBJECT_IDX_RED_CUBE         0
#define OBJECT_IDX_BLUE_CUBE        1
#define OBJECT_IDX_GREEN_CUBE       2
#define OBJECT_IDX_YELLOW_CUBE      3
#define OBJECT_IDX_YELLOW_BALL      4
#define OBJECT_IDX_RED_BALL         5
#define OBJECT_IDX_GREEN_CYLINDER   6
#define OBJECT_IDX_BLUE_TRIANGLE    7
#define OBJECT_IDX_PURPLE_CROSS     8
#define OBJECT_IDX_PATRIC           9
#define OBJECT_IDX_UNKNOWN          10

// Coordinate frames
#define COORD_FRAME_WORLD       "world"
#define COORD_FRAME_ROBOT       "robot"







// ===========================================================================
// ===========================================================================

// Paths where we store or read data
namespace RAS_Names
{
const std::string models_3D_path = (std::string(getenv("HOME")) + std::string("/3d_data/"));
const std::string CALIBRATION_PATH = (std::string(getenv("HOME")) + std::string("/camera_extrinsic.txt"));

const std::string MAP_ROOT_PATH = (std::string(getenv("HOME")) + std::string("/map/"));
const std::string RAW_MAP_PATH = (std::string(getenv("HOME")) + std::string("/map/raw_map.txt"));
const std::string THICK_MAP_PATH = (std::string(getenv("HOME")) + std::string("/map/thick_map.txt"));
const std::string TOPOLOGICAL_MAP_PATH = (std::string(getenv("HOME")) + std::string("/map/topological_map.txt"));

    namespace Navigation_Modes
    {
        static const int NAVIGATION_WALL_FOLLOW = 1;
        static const int NAVIGATION_GO_OBJECT   = 2;
        static const int NAVIGATION_STOP        = 3;
    }    
}

#endif
