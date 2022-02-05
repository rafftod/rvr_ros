#ifndef RVR_FORAGING
#define RVR_FORAGING

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_wheels_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_ground_color_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_rgb_leds_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_proximity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_lidar_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_quaternion_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_light_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_velocity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_imu_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_locator_sensor.h>

#include <argos3/core/utility/math/rng.h>

#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>

// ROS libraries
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// terabee
#include "teraranger_array/RangeArray.h"
#include "sensor_msgs/Range.h"

// lidar
#include "sensor_msgs/LaserScan.h"

#define WHEEL_DIAMETER 6.5
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER * M_PI) / 100.0)
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE / 1000.0)
#define WHEEL_SEPARATION 0.53
#define WHEEL_DISTANCE 0.147

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

class CRVR : public CCI_Controller
{

public:
    // Foraging state machine states
    enum State
    {
        START,     // initial state
        SEARCHING, // exploring space to find goal
        PICKUP,    // picking stuff up
        HOMING,    // going back home
        TURN_BACK, // turn back when touching borders
    };

public:
    /* Class constructor. */
    CRVR();

    /* Class destructor. */
    virtual ~CRVR() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode &t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Reset() {}

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    // ROS methods
    virtual void InitRos(); //must be called in Init

    virtual void RosControlStep(); //must be called in ControlStep

    /* Handler for the ground color sensor data from RVR driver */
    virtual void ColorHandler(const std_msgs::ColorRGBA &msg);

    /* Handler for odometry from RVR driver */
    virtual void OdometryHandler(const nav_msgs::Odometry &msg);

    /* Handler for proximity sensor data */
    virtual void TerarangerHandler(const teraranger_array::RangeArray &msg);

    /* Handler for lidar Laserscan data */
    virtual void LidarHandler(const sensor_msgs::LaserScan &msg);

    /* Start step of the foraging state machine */
    virtual void StartStep();

    /* Search step of the foraging state machine */
    virtual void SearchStep();

    /* Pickup step of the foraging state machine */
    virtual void PickupStep();

    /* Homing step of the foraging state machine */
    virtual void HomingStep();

    /* Turn back step of the foraging state machine */
    virtual void TurnBackStep();

    /* Computes wheel velocities from vector to follow */
    virtual CVector2 ComputeWheelsVelocityFromVector(CVector2 c_vector_to_follow);

    /* Virtual odometry updater */
    virtual void OdometryUpdate();

private:
    /* These are pointers to the different sensors 
    and actuators the RVR implements. */

    CCI_RVRWheelsActuator *m_pcWheels;

    CCI_RVRGroundColorSensor *m_pcColorSensor;

    CCI_RVRRGBLEDsActuator *m_pcLedsActuator;

    CCI_RVRProximitySensor *m_pcProximitySensor;

    CCI_RVRLidarSensor *m_pcLidarSensor;

    CCI_RVRQuaternionSensor *m_pcQuaternionSensor;

    CCI_RVRLightSensor *m_pcLightSensor;

    CCI_RVRVelocitySensor *m_pcVelocitySensor;

    CCI_RVRIMUSensor *m_pcImuSensor;

    CCI_RVRLocatorSensor *m_pcLocatorSensor;

    CRandom::CRNG *m_pcRng;

    CRange<Real> m_cRandomRange;

    // Sensors subscribers
    ros::Subscriber color_sensor_sub;
    ros::Subscriber odometry_subscriber;

    // teraranger subscriber
    ros::Subscriber teraranger_sub;

    // lidar subscriber
    ros::Subscriber lidar_sub;

    // Actuators publishers
    ros::Publisher vel_pub;
    std_msgs::Float32MultiArray vel_msg;

    ros::Publisher led_pub[5];
    std_msgs::ColorRGBA led_msg[5];

    // mapping publishers
    ros::Publisher mapping_odom_pub;
    ros::Publisher mapping_laser_pub;

    nav_msgs::Odometry odomMsg;
    sensor_msgs::LaserScan laserMsg;

    /* Wheel speed. */
    Real leftWheelVelocity;
    Real rightWheelVelocity;

    /* This is the default wheel velocity.
    It is usually parsed from the XML file. */
    Real m_fDefaultWheelVelocity;

    // color detected by the sensor
    CColor sensor_color;
    // proximity sensor readings
    Real prox_readings[8];
    // lidar readings
    Real lidar_readings[719];
    // quaternion sensor reading
    CQuaternion quat_reading;
    // light sensor reading
    Real light;
    // X and Y velocities readings
    Real XVelocity, YVelocity;
    // IMU readings
    CRadians pitch, roll, yaw;
    // Position reading
    CVector2 locatorPosition;

    // boolean that indicates if we are using the real robot
    bool rvr_driven;

    // current state
    State state;

    //odometry variables
    double leftStepsDiff, rightStepsDiff;
    double leftStepsPrev, rightStepsPrev;
    double xPos, yPos, theta;
    double deltaSteps, deltaTheta;
    double calibAngle, calibStep;
    //random walk parameters
    int randomWalk;
    double mu, gamma;
    double lMin, bias;
    int stepLength;
    CRadians stepAngle;

    // Home coordinates
    double homeXPos, homeYPos;
    CColor homeColor;
    // Target color
    CColor targetColor;
    // Border color
    CColor border_color;
    // pickup state info
    clock_t pickup_start;
    float pickup_duration;
    // turn back info
    double starting_theta;

    //clocks
    ros::Time currentTime, lastTime;
};

#endif