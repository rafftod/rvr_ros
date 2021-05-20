/* Include the controller definition */
#include "foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>

#define round(x) (float)(((float)((int)(x * 100 + .5))) / 100) // rounds to second decimal

CRVR::CRVR() : m_pcWheels(NULL),
               m_pcColorSensor(NULL),
               m_pcLedsActuator(NULL),
               m_pcProximitySensor(NULL),
               m_pcLidarSensor(NULL),
               sensor_color(CColor::GREEN),
               m_fDefaultWheelVelocity(155.5f),
               rvr_driven(false),
               xPos(0.0),
               yPos(0.0),
               theta(0.0),
               randomWalk(1),
               mu(3.0),
               gamma(0.0),
               lMin(1.0),
               bias(0.0),
               stepLength(0),
               stepAngle(CRadians::ZERO),
               targetColor(CColor::RED),
               border_color(CColor::GREEN),
               leftStepsDiff(0),
               rightStepsDiff(0),
               calibStep(0.67),
               calibAngle(0.4355),
               pickup_duration(3.0f)
{
}

void CRVR::Init(TConfigurationNode &t_node)
{
    /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_obstacleavoidance><actuators> and
    * <controllers><epuck_obstacleavoidance><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels = GetActuator<CCI_RVRWheelsActuator>("rvr_wheels");
    m_pcColorSensor = GetSensor<CCI_RVRGroundColorSensor>("rvr_ground");
    m_pcLedsActuator = GetActuator<CCI_RVRRGBLEDsActuator>("rvr_rgb_leds");
    m_pcProximitySensor = GetSensor<CCI_RVRProximitySensor>("rvr_proximity");
    m_pcLidarSensor = GetSensor<CCI_RVRLidarSensor>("rvr_lidar");
    m_pcRng = CRandom::CreateRNG("argos");
    m_cRandomRange.SetMax(1.0);
    /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fDefaultWheelVelocity, m_fDefaultWheelVelocity);
    leftWheelVelocity = m_fDefaultWheelVelocity;
    rightWheelVelocity = m_fDefaultWheelVelocity;
    GetNodeAttributeOrDefault(t_node, "rvr_driven", rvr_driven, rvr_driven);
    InitRos();
}

void CRVR::InitRos()
{
    std::stringstream name;
    name.str("");
    name << GetId();

    if (!ros::isInitialized())
    {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }
    std::stringstream ss;
    ros::NodeHandle rosNode;

    // setup color sensor subscriber
    color_sensor_sub = rosNode.subscribe("sensor_color", 10, &CRVR::ColorHandler, this);
    // setup odometry subscriber
    odometry_subscriber = rosNode.subscribe("odom", 10, &CRVR::OdometryHandler, this);

    // teraranger subscriber
    teraranger_sub = rosNode.subscribe("ranges", 10, &CRVR::TerarangerHandler, this);

    // lidar subscriber
    lidar_sub = rosNode.subscribe("scan", 10, &CRVR::LidarHandler, this);

    // setup velocity publisher
    vel_pub = rosNode.advertise<std_msgs::Float32MultiArray>("wheels_velocity", 10);
    // setup velocity messages
    vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    vel_msg.layout.dim[0].size = 2;
    vel_msg.layout.dim[0].stride = 1;
    vel_msg.layout.dim[0].label = "wheel_vel";
    vel_msg.data.clear();

    // setup LED publisher
    for (unsigned short int i = 0; i < 5; ++i)
    {
        ss.str("");
        ss << "led_color_" << i;
        led_pub[i] = rosNode.advertise<std_msgs::ColorRGBA>(ss.str(), 10);
    }
}

void CRVR::ControlStep()
{

    if (!rvr_driven)
    {
        // virtual sense
        sensor_color = m_pcColorSensor->GetReading();
        for (short int i = 0; i < 8; ++i)
        {
            prox_readings[i] = m_pcProximitySensor->GetReading(i).Value;
        }
        for (short int i = 0; i < 719; ++i)
        {
            lidar_readings[i] = m_pcLidarSensor->GetReading(i).Value;
        }
    }
    m_pcLedsActuator->SetColors(sensor_color);
    switch (state)
    {
    case State::START:
        // save location and start exploration (transition to SEARCHING state)
        StartStep();
        break;
    case State::SEARCHING:
        // randow walk ? until target color detected, avoid borders
        SearchStep();
        break;
    case State::PICKUP:
        // wait to simulate pickup
        PickupStep();
        break;
    case State::HOMING:
        // turn in home direction then go straight
        HomingStep();
        break;
    case State::TURN_BACK:
        // 180° turn
        TurnBackStep();
        break;
    }
    if (!rvr_driven)
        OdometryUpdate();
    RosControlStep();
    ros::spinOnce();
}

void CRVR::StartStep()
{
    // save location and start exploration (transition to SEARCHING state)
    homeXPos = xPos;
    homeYPos = yPos;
    homeColor = sensor_color;
    state = State::SEARCHING;
}

void CRVR::SearchStep()
{
    if (sensor_color == targetColor)
    {
        pickup_start = clock();
        state = State::PICKUP;
        return;
    }
    if (sensor_color == border_color)
    {
        // turn back
        starting_theta = theta;
        state = State::TURN_BACK;
        return;
    }
    if (stepLength <= 0)
    {
        Real r = m_pcRng->Uniform(m_cRandomRange);
        stepLength = ceil(lMin * pow(r, 1 / (1 - mu)) + 0.5);
        stepAngle = CRadians(2 * atan(((1 - gamma) / (1 + gamma)) * tan(M_PI * (r - 0.5))) + bias);
    }
    else
    {
        stepLength--;
    }
    CVector2 speeds = ComputeWheelsVelocityFromVector(CVector2(1.0, stepAngle));
    leftWheelVelocity = speeds.GetX();
    rightWheelVelocity = speeds.GetY();
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
    /*     leftWheelVelocity = 5;
    rightWheelVelocity = 5;
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity); */
}

void CRVR::PickupStep()
{
    leftWheelVelocity = 0.0f;
    rightWheelVelocity = 0.0f;
    clock_t t = clock();
    if (((float)(t - pickup_start) / CLOCKS_PER_SEC) >= pickup_duration)
    {
        state = State::HOMING;
    }
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
    printf("Objective reached at (%f,%f), home is at (%f,%f)\n", xPos, yPos, homeXPos, homeYPos);
}

void CRVR::HomingStep()
{
    CVector2 direction(homeXPos - xPos, homeYPos - yPos);
    if (sensor_color == homeColor)
    {
        // we are at home !
        leftWheelVelocity = 0;
        rightWheelVelocity = 0;
        m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
        return;
    }
    CVector2 pointing_vector(cos(theta), sin(theta));
    // extract angle between the vectors
    double angle = acos(direction.DotProduct(pointing_vector) / (direction.Length() * pointing_vector.Length()));
    //std::cout << "theta : " << theta << std::endl;
    std::cout << "angle : " << angle << std::endl;
    if (std::abs(angle) <= 0.1)
    {
        leftWheelVelocity = m_fDefaultWheelVelocity;
        rightWheelVelocity = m_fDefaultWheelVelocity;
    }
    else
    {
        if (angle > 0)
        {
            leftWheelVelocity = m_fDefaultWheelVelocity;
            rightWheelVelocity = -m_fDefaultWheelVelocity;
        }
        else
        {
            leftWheelVelocity = -m_fDefaultWheelVelocity;
            rightWheelVelocity = m_fDefaultWheelVelocity;
        }
    }
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
}

void CRVR::TurnBackStep()
{
    double target_theta = starting_theta >= 0 ? starting_theta - M_PI : starting_theta + M_PI;
    if (std::abs(theta - target_theta) <= 0.04)
    {
        // back to random walk
        state = State::SEARCHING;
        return;
    }
    if (theta > target_theta)
    {
        leftWheelVelocity = m_fDefaultWheelVelocity;
        rightWheelVelocity = -m_fDefaultWheelVelocity;
    }
    else
    {
        leftWheelVelocity = -m_fDefaultWheelVelocity;
        rightWheelVelocity = m_fDefaultWheelVelocity;
    }
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
}

CVector2 CRVR::ComputeWheelsVelocityFromVector(CVector2 c_vector_to_follow)
{
    Real fLeftVelocity = 0;
    Real fRightVelocity = 0;
    CRange<CRadians> cLeftHemisphere(CRadians::ZERO, CRadians::PI);
    CRange<CRadians> cRightHemisphere(CRadians::PI, CRadians::TWO_PI);
    CRadians cNormalizedVectorToFollow = c_vector_to_follow.Angle().UnsignedNormalize();

    // Compute relative wheel velocity
    if (c_vector_to_follow.GetX() != 0 || c_vector_to_follow.GetY() != 0)
    {
        if (cLeftHemisphere.WithinMinBoundExcludedMaxBoundExcluded(cNormalizedVectorToFollow))
        {
            fRightVelocity = 1;
            fLeftVelocity = Max<Real>(-0.5f, Cos(cNormalizedVectorToFollow));
        }
        else
        {
            fRightVelocity = Max<Real>(-0.5f, Cos(cNormalizedVectorToFollow));
            fLeftVelocity = 1;
        }
    }

    // Transform relative velocity according to max velocity allowed
    Real fVelocityFactor = m_fDefaultWheelVelocity / Max<Real>(std::abs(fRightVelocity), std::abs(fLeftVelocity));
    CVector2 cWheelsVelocity = CVector2(fVelocityFactor * fLeftVelocity, fVelocityFactor * fRightVelocity);

    return cWheelsVelocity;
}

void CRVR::RosControlStep()
{
    // publish velocity
    vel_msg.data.clear();
    vel_msg.data.push_back(round(leftWheelVelocity / 100)); // convert cm/s to m/s
    vel_msg.data.push_back(round(rightWheelVelocity / 100));
    vel_pub.publish(vel_msg);
    // publish leds color
    for (unsigned short int i = 0; i < 5; ++i)
    {
        led_msg[i].r = sensor_color.GetRed();
        led_msg[i].g = sensor_color.GetGreen();
        led_msg[i].b = sensor_color.GetBlue();
        led_msg[i].a = sensor_color.GetAlpha();
        led_pub[i].publish(led_msg[i]);
    }
}

void CRVR::OdometryUpdate()
{
    //odometry computation
    leftStepsDiff = leftWheelVelocity * MOT_STEP_DIST * calibStep; // 750 given from tests
    rightStepsDiff = rightWheelVelocity * MOT_STEP_DIST * calibStep;

    deltaTheta = ((rightStepsDiff - leftStepsDiff) / WHEEL_DISTANCE) * M_PI / calibAngle; // Expressed in radiant.
    deltaSteps = (rightStepsDiff + leftStepsDiff) / 2;                                    // Expressed in meters.

    xPos += deltaSteps * cos(theta + deltaTheta / 2); // Expressed in meters.

    yPos += deltaSteps * sin(theta + deltaTheta / 2); // Expressed in meters.

    theta += deltaTheta; // Expressed in radiant.
    if (theta >= 2 * M_PI)
    {
        theta -= 2 * M_PI;
    }
    else if (theta <= -2 * M_PI)
    {
        theta += 2 * M_PI;
    }
    /*     std::cout << "theta : " << theta << std::endl;
    std::cout << "x : " << xPos << std::endl;
    std::cout << "y : " << yPos << std::endl;
    std::cout << "distance from start : " << sqrt(xPos * xPos + yPos * yPos) << std::endl; */
}

void CRVR::ColorHandler(const std_msgs::ColorRGBA &msg)
{
    if (!rvr_driven)
        rvr_driven = true;
    sensor_color.SetRed(msg.r);
    sensor_color.SetGreen(msg.g);
    sensor_color.SetBlue(msg.b);
    sensor_color.SetAlpha(msg.a);
}

void CRVR::OdometryHandler(const nav_msgs::Odometry &msg)
{
    if (!rvr_driven)
        rvr_driven = true;
    xPos = msg.pose.pose.position.x;
    yPos = msg.pose.pose.position.y;
    theta = tf::getYaw(msg.pose.pose.orientation);
}

void CRVR::TerarangerHandler(const teraranger_array::RangeArray &msg)
{
    if (!rvr_driven)
        rvr_driven = true;
    for (short int i = 0; i < 8; ++i)
    {
        prox_readings[i] = msg.ranges[i].range;
    }
}

void CRVR::LidarHandler(const sensor_msgs::LaserScan &msg)
{
    if (!rvr_driven)
        rvr_driven = true;
    for (short int i = 0; i < 719; ++i)
        lidar_readings[i] = msg.ranges[i];
}

REGISTER_CONTROLLER(CRVR, "rvr_foraging")