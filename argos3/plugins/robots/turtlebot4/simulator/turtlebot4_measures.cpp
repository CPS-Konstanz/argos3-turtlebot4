/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "turtlebot4_measures.h"

/****************************************/
/****************************************/
// Measures from here:
// https://emanual.robotis.com/docs/en/platform/turtlebot4/features/

UInt8 TURTLEBOT4_POWERON_LASERON = 3;
const Real TURTLEBOT4_MASS = 3.945f;                                                             // mass in kg (see https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)
const Real TURTLEBOT4_BASE_RADIUS = 0.17095;                                                     // meters, value used for collision (see https://iroboteducation.github.io/create3_docs/hw/mechanical/)
const Real TURTLEBOT4_BASE_ELEVATION = 0.0045;                                                   // meters, height of the base from the ground (see https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)
const Real TURTLEBOT4_BASE_HEIGHT = 0.351;                                                       // meters, height of the robot (https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)
const Real TURTLEBOT4_LOWER_BODY_HEIGHT = 0.0934;                                                // meters, height of the lower body part, that is Create3 platform height (see https://iroboteducation.github.io/create3_docs/hw/mechanical/)
const Real TURTLEBOT4_UPPER_BODY_HEIGHT = TURTLEBOT4_BASE_HEIGHT - TURTLEBOT4_LOWER_BODY_HEIGHT; // meters, position of the top plate, approximation
const Real TURTLEBOT4_BASE_TOP = TURTLEBOT4_BASE_ELEVATION + TURTLEBOT4_BASE_HEIGHT;             // meters, height of the top of the robot
const Real TURTLEBOT4_COLUMN_RADIUS = 0.008f;                                                    // tubes ~1cm
const Real TURTLEBOT4_COLUMN_HEIGHT = TURTLEBOT4_UPPER_BODY_HEIGHT;                              // meters, heigh of the sticks connecting the base to the upper body, approximation

const Real TURTLEBOT4_WHEEL_RADIUS = 0.036;                                  // meters, radius of the wheel (see https://iroboteducation.github.io/create3_docs/hw/mechanical/)
const Real TURTLEBOT4_WHEEL_DISTANCE = 0.235;                                // meters, distance between the centers of the two wheels (see https://iroboteducation.github.io/create3_docs/hw/mechanical/)
const Real TURTLEBOT4_HALF_WHEEL_DISTANCE = TURTLEBOT4_WHEEL_DISTANCE * 0.5; // meters, half of the distance between the centers of the two wheels
const Real UPPER_BODY_RADIUS = TURTLEBOT4_BASE_RADIUS * 0.95f;               // meters, radius of the top plate, approximation

const Real TURTLEBOT4_LIDAR_ELEVATION = TURTLEBOT4_LOWER_BODY_HEIGHT + 0.098715f; // meters (see https://github.com/paoloelle/turtlebot4/blob/a50ef806e23b45d13fabad0b99c90013c4fac9a0/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro#L33
const Real TURTLEBOT4_LIDAR_SENSORS_FAN_RADIUS = TURTLEBOT4_BASE_RADIUS;          // meters, here we avoid that the lidar detects the sticks so we set the radius as the base radius
const CRadians TURTLEBOT4_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(360.0)));           // radians, angle span of the LiDAR sensor all around the robot
const CRange<Real> TURTLEBOT4_LIDAR_SENSORS_RING_RANGE(0.01, 12.00);              // minimum and maximum reading ranges of the LiDAR. these values sums to TURTLEBOT4_LIDAR_SENSORS_FAN_RADIUS

const Real TURTLEBOT4_MIN_LINEAR_VELOCITY = 0.0f;   // m/s, minimum linear velocity of the robot, robot can go backwards but we assume that this is not possibile
const Real TURTLEBOT4_MAX_LINEAR_VELOCITY = 0.460f; // m/s, maximum linear velocity of the robot (see https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)

// please note that on the angular velocity the sign of the value determins the direction of rotation, here we define the module of the min and maximum values
const Real TURTLEBOT4_MIN_ANGULAR_VELOCITY = 0;     // rad/s, minimum angular velocity of the robot (see, https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)
const Real TURTLEBOT4_MAX_ANGULAR_VELOCITY = 1.90f; // rad/s, maximum angular velocity of the robot (see, https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)

// values for max force and max torque are copied from the footbot
const Real TURTLEBOT4_MAX_FORCE = 15.0f;
const Real TURTLEBOT4_MAX_TORQUE = 150.0f;

const CRadians TURTLEBOT4_LED_RING_START_ANGLE = CRadians((ARGOS_PI / 8.0f) * 0.5f);
const Real TURTLEBOT4_LED_RING_ELEVATION = TURTLEBOT4_BASE_TOP + 0.02f;
const Real TURTLEBOT4_RAB_ELEVATION = TURTLEBOT4_LED_RING_ELEVATION;

const Real TURTLEBOT4_IR_SENSOR_RING_ELEVATION = 0.0572; // meters (see https://github.com/iRobotEducation/create3_sim/blob/b7c69013d0db241df64199cae9491286635d1bcc/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro)
const Real TURTLEBOT4_IR_SENSOR_RING_RADIUS = TURTLEBOT4_BASE_RADIUS;
const CRadians TURTLEBOT4_IR_SENSOR_RING_START_ANGLE = CRadians((2 * ARGOS_PI / 8.0f) * 0.5f);
const Real TURTLEBOT4_IR_SENSOR_RING_RANGE = 0.1f;
const Real OMNIDIRECTIONAL_CAMERA_ELEVATION = 0.288699733f;

const CVector2 TURTLEBOT4_GROUND_SENSOR_SIDE_LEFT(0.06f, 0.145f);    // meters. XY position of the side left ground sensor, relative to the center of the robot  (see https://github.com/iRobotEducation/create3_sim/blob/b7c69013d0db241df64199cae9491286635d1bcc/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro#L137)
const CVector2 TURTLEBOT4_GROUND_SENSOR_SIDE_RIGHT(0.06f, -0.145f);  // meters. XY position of the side rigth ground sensor, relative to the center of the robot (see https://github.com/iRobotEducation/create3_sim/blob/b7c69013d0db241df64199cae9491286635d1bcc/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro#L137)
const CVector2 TURTLEBOT4_GROUND_SENSOR_FRONT_LEFT(0.16f, 0.045f);   // meters. XY position of the frontleft ground sensor, relative to the center of the robot (see https://github.com/iRobotEducation/create3_sim/blob/b7c69013d0db241df64199cae9491286635d1bcc/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro#L137)
const CVector2 TURTLEBOT4_GROUND_SENSOR_FRONT_RIGHT(0.16f, -0.045f); // meters. XY position of the front right ground sensor, relative to the center of the robot (see https://github.com/iRobotEducation/create3_sim/blob/b7c69013d0db241df64199cae9491286635d1bcc/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro#L137)
