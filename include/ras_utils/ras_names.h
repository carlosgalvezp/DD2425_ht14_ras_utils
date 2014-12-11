#ifndef RAS_NAMES_H
#define RAS_NAMES_H
#include <string>
#include <vector>

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
#define TOPIC_ARDUINO_ADC_FILTERED      "/arduino/ir_m_filtered"
#define TOPIC_ARDUINO_ADC_NOT_FILTERED  "/arduino/ir_m_not_filtered"

#define TOPIC_ODOMETRY                  "/robot/odometry"
#define TOPIC_ARDUINO_PWM               "/arduino/pwm"
#define TOPIC_ARDUINO_ENCODERS          "/arduino/encoders"

#define TOPIC_MAP_OCC_GRID              "/map/occ_grid_rviz"
#define TOPIC_MAP_OCC_GRID_THICK        "/map/occ_grid_thick"
#define TOPIC_MAP_COST                  "/map/array_cost"

#define TOPIC_MAP_OCC_GRID_BAG          "/map/occ_grid_rviz_bag"
#define TOPIC_MAP_OCC_GRID_THICK_BAG    "/map/occ_grid_thick_bag"
#define TOPIC_MAP_COST_BAG              "/map/array_cost_bag"

#define TOPIC_LOCALIZATION              "/robot/odometry/localization"

#define TOPIC_SPEAKER                   "/espeak/string"
#define TOPIC_EVIDENCE                  "/evidence"
#define TOPIC_MOTOR_CONTROLLER_TWIST    "/motor_controller/twist"
#define TOPIC_CAMERA_RGB                "/camera/rgb/image_rect_color"
#define TOPIC_CAMERA_DEPTH              "/camera/depth_registered/hw_registered/image_rect_raw"
#define TOPIC_OBSTACLE                  "/obstacle"
#define TOPIC_IMU                       "/imu/data"
#define TOPIC_MARKERS                   "/map_markers"
#define TOPIC_OBJECT_MARKERS            "/map_object_markers"
#define TOPIC_OBJECT_AS_OBSTACLE        "/object_detection/object_as_obstacle"

#define TOPIC_ROBOT_OBJECT_POSITION     "/object_detection/robot_position"
#define TOPIC_OBSTACLE_LASER_MAP        "/obstacle_detection/laser"

#define TOPIC_PATH_FINDER_POINT         "/navigation/path_finder_point"


// Servers names
#define SRV_BRAIN_IN                    "/brain/comm"
#define SRV_NAVIGATION_IN               "/navigation/comm"

// Parameter server
#define PARAM_PHASE                     "/Phase"
#define PARAM_ROBOT_VELOCITY            "/Robot_Velocity"
#define PARAM_CONTEST                   "/Contest"

// Rviz namespaces
#define RVIZ_MARKER_NS_OBJECT           "Objects"

// 3D Shapes
#define SHAPE_3D_CUBE      0
#define SHAPE_3D_BALL      1
#define SHAPE_3D_OTHER     2

// Colors
#define COLOR_RED            0
#define COLOR_GREEN          1
#define COLOR_BLUE           2
#define COLOR_YELLOW         3
#define COLOR_PURPLE         4
#define COLOR_LIGHT_GREEN    5
#define COLOR_ORANGE         6

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
#define COORD_FRAME_WORLD               "world"
#define COORD_FRAME_ROBOT               "robot"
#define COORD_FRAME_CAMERA_LINK         "camera_link"
#define COORD_FRAME_CAMERA_RGB_OPTICAL  "camera_rgb_optical_frame"

// IR Sensors
#define IR_SENSOR_FRONT_RIGHT      0
#define IR_SENSOR_FRONT_LEFT       1
#define IR_SENSOR_BACK_RIGHT       2
#define IR_SENSOR_BACK_LEFT        3
#define IR_SENSOR_FRONT            4
#define IR_SENSOR_BACK             5

// ===========================================================================
// ===========================================================================

// Paths where we store or read data
namespace RAS_Names
{
const std::string HOME = std::string(getenv("HOME"));
const std::string PROJECT_DATA_ROOT = HOME + std::string("/DD2425_Data/");

const std::string MODELS_3D_PATH    =   PROJECT_DATA_ROOT + std::string("3d_data_train/");
const std::string MODELS_COLOR_ROOT =   PROJECT_DATA_ROOT + std::string("color_data/");
const std::string MODELS_COLOR_PATH =   MODELS_COLOR_ROOT + std::string("models.txt");


const std::vector<std::string> MODELS_3D_NAMES = {"cube", "ball"};
//const std::vector<std::string> COLOR_NAMES = {"red", "green", "blue", "yellow", "purple"};
const std::vector<std::string> COLOR_NAMES = {"red", "green", "blue", "yellow", "purple", "light_green", "orange"};


const std::string CALIBRATION_PATH = PROJECT_DATA_ROOT + std::string("cam_calibration/tf_robot_to_camera_link.txt");

const std::string MAP_ROOT_PATH = PROJECT_DATA_ROOT + std::string("map/") ;

const std::string OBJECT_GRAPH_PATH = MAP_ROOT_PATH + std::string("object_graph.txt");
const std::string OBJECT_POSITIONS_PATH = MAP_ROOT_PATH + std::string("object_positions.txt");
const std::string OBJECT_BEST_PATH_PATH = MAP_ROOT_PATH + std::string("object_path.txt");

const std::string INITIAL_ODOMETRY_ALIGNER = MAP_ROOT_PATH + std::string("initial_odometry_aligner");

const std::string HSV_PARAMS_ROOT_PATH = PROJECT_DATA_ROOT + std::string("HSV/");
const std::string HSV_PARAMS_LAB       = HSV_PARAMS_ROOT_PATH + std::string("lab.txt");
const std::string HSV_PARAMS_CONTEST   = HSV_PARAMS_ROOT_PATH + std::string("contest.txt");

const std::string FILE_TIME_START_MAP_ROSBAG = "time_start_map_bag.txt";

    namespace Navigation_Modes
    {
        static const int NAVIGATION_WALL_FOLLOW = 1;
        static const int NAVIGATION_GO_OBJECT   = 2;
        static const int NAVIGATION_STOP        = 3;
    }    
}

#endif
