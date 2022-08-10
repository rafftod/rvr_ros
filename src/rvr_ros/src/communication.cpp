/* Include the controller definition */
#include "communication.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>
#include <fstream>

#define round(x) (float)(((float)((int)(x * 100 + .5))) / 100) // rounds to second decimal

CRVR::CRVR() : m_pcWheels(NULL),
               m_pcColorSensor(NULL),
               m_pcLedsActuator(NULL),
               m_pcProximitySensor(NULL),
               m_pcLidarSensor(NULL),
               m_pcQuaternionSensor(NULL),
               m_pcLightSensor(NULL),
               m_pcVelocitySensor(NULL),
               m_pcImuSensor(NULL),
               m_pcLocatorSensor(NULL),
               m_pcAccelerometerSensor(NULL),
               m_pcGyroscopeSensor(NULL),
               sensor_color(CColor::RED),
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
               calibStep(5.8),
               calibAngle(3.8),
               pickup_duration(3.0f),
               m_eExplorationState(RANDOM_WALK),
               m_fProximityThreshold(0.1),
               m_unTurnSteps(0)
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
    m_pcQuaternionSensor = GetSensor<CCI_RVRQuaternionSensor>("rvr_quaternion");
    m_pcLightSensor = GetSensor<CCI_RVRLightSensor>("rvr_light");
    m_pcVelocitySensor = GetSensor<CCI_RVRVelocitySensor>("rvr_velocity");
    m_pcImuSensor = GetSensor<CCI_RVRIMUSensor>("rvr_imu");
    m_pcLocatorSensor = GetSensor<CCI_RVRLocatorSensor>("rvr_locator");
    m_pcAccelerometerSensor = GetSensor<CCI_RVRAccelerometerSensor>("rvr_accelerometer");
    m_pcGyroscopeSensor = GetSensor<CCI_RVRGyroscopeSensor>("rvr_gyroscope");
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
    GetNodeAttributeOrDefault(t_node, "proximity_range", prox_range, prox_range);
    SInt32 max_rw_steps = 0;
    GetNodeAttributeOrDefault(t_node, "rw_max_steps", max_rw_steps, max_rw_steps);
    m_cRandomStepsRange.SetMax(max_rw_steps);
    GetNodeAttributeOrDefault(t_node, "rw_prox_threshold", m_fProximityThreshold, m_fProximityThreshold);
    // set state as starting state
    state = State::AUTOMODE_RW;
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

    ss << name.str() << "/map_merge/init_pose_x";
    rosNode.setParam(ss.str(), xPos);
    ss.str("");
    ss << name.str() << "/map_merge/init_pose_y";
    rosNode.setParam(ss.str(), yPos);
    ss.str("");
    ss << name.str() << "/map_merge/init_pose_z";
    rosNode.setParam(ss.str(), 0.0);
    ss.str("");
    ss << name.str() << "/map_merge/init_pose_yaw";
    rosNode.setParam(ss.str(), theta);

    // setup color sensor subscriber
    color_sensor_sub = rosNode.subscribe("/rvr/ground_color", 10, &CRVR::ColorHandler, this);
    // setup IMU subscriber
    imu_subscriber = rosNode.subscribe("/rvr/imu", 10, &CRVR::ImuHandler, this);
    // setup light sensor subscriber
    light_subscriber = rosNode.subscribe("/rvr/ambient_light", 10, &CRVR::LightHandler, this);
    // setup odometry subscriber
    odom_subscriber = rosNode.subscribe("/rvr/odom", 10, &CRVR::OdometryHandler, this);

    // setup teraranger subscriber
    prox_sub = rosNode.subscribe("ranges", 10, &CRVR::TerarangerHandler, this);

    // setup lidar subscriber
    lidar_sub = rosNode.subscribe("scan", 10, &CRVR::LidarHandler, this);

    // setup velocity publisher
    vel_pub = rosNode.advertise<std_msgs::Float32MultiArray>("/rvr/wheels_speed", 10, true);
    // setup velocity messages
    vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    vel_msg.layout.dim[0].size = 2;
    vel_msg.layout.dim[0].stride = 1;
    vel_msg.layout.dim[0].label = "wheel_vel";
    vel_msg.data.clear();

    // mapping publishers

    mapping_laser_pub = rosNode.advertise<sensor_msgs::LaserScan>("/map_scan", 10);
    mapping_odom_pub = rosNode.advertise<nav_msgs::Odometry>("/odom", 10);

    // init laser scan
    ss.str("");
    ss << name.str() << "/base_laser";
    laserMsg.header.frame_id = ss.str();
    laserMsg.angle_min = -M_PI;
    laserMsg.angle_max = M_PI;
    laserMsg.angle_increment = M_PI / 4.0;
    laserMsg.range_min = 0;      // laser scan situated in the center of the robot
    laserMsg.range_max = 200000; // laser scan situated in the center of the robot
    laserMsg.ranges.resize(719);
    laserMsg.intensities.resize(719);
    // setup LED publisher
    ss.str("");
    ss << "/rvr/rgb_leds";
    led_pub = rosNode.advertise<rvr_ros::Leds>(ss.str(), 10, true);

    currentTime = ros::Time::now();
    lastTime = ros::Time::now();
}

void CRVR::VirtualSense()
{
    // virtual sense
    sensor_color = m_pcColorSensor->GetReading().Color;
    for (short int i = 0; i < 8; ++i)
    {
        prox_readings[i] = m_pcProximitySensor->GetReading(i).Value;
    }
    for (short int i = 0; i < 719; ++i)
    {
        lidar_readings[i] = m_pcLidarSensor->GetReading(i).Value;
    }
    quat_reading = m_pcQuaternionSensor->GetReading().Orientation;
    light = m_pcLightSensor->GetReading().Value;
    auto velocity_reading = m_pcVelocitySensor->GetReading();
    pitch = m_pcImuSensor->GetReading().Pitch;
    roll = m_pcImuSensor->GetReading().Roll;
    yaw = m_pcImuSensor->GetReading().Yaw;
    locatorPosition = m_pcLocatorSensor->GetReading().Position;
    acceleration = m_pcAccelerometerSensor->GetReading().Acceleration;
    angularVelocity = m_pcGyroscopeSensor->GetReading().AngularVelocity;
}

void CRVR::WriteFloorColorToFile()
{
    //std::cout << sensor_color.GetRed() << ' ' << sensor_color.GetGreen() << ' ' << sensor_color.GetBlue() << std::endl;

    std::ofstream floor_file("floor_color");
    floor_file << sensor_color.GetRed() << ' ' << sensor_color.GetGreen() << ' ' << sensor_color.GetBlue() << '\n';
    floor_file.close();
}

void CRVR::SetLedColorFromFile()
{
    std::ifstream led_file("led_color");
    std::string s;
    std::getline(led_file, s);
    led_file.close();

    std::string delimiter = " ";
    size_t pos = 0;
    std::string token;

    argos::CColor *color;
    argos::UInt8 red, green, blue;

    pos = s.find(delimiter);
    token = s.substr(0, pos);
    red = std::stoi(token);
    s.erase(0, pos + delimiter.length());

    pos = s.find(delimiter);
    token = s.substr(0, pos);
    green = std::stoi(token);
    s.erase(0, pos + delimiter.length());
    
    blue = std::stoi(s);

    color = new CColor(red, green, blue);
    m_pcLedsActuator->SetColors(*color);
    for (int i = 0; i < 5; i++)
    {
        led_colors[i] = *color;
    }
}

void CRVR::ControlStep()
{
    if (!rvr_driven)
    {
        VirtualSense();
    }

    //m_pcLedsActuator->SetColors(sensor_color);
    WriteFloorColorToFile();
    SetLedColorFromFile();

    // AutomodeRWStep();
    leftWheelVelocity = 0;
    rightWheelVelocity = 0;

    if (!rvr_driven)
        OdometryUpdate();
    RosControlStep();
    ros::spinOnce();
}

void CRVR::TestStep()
{
    // test state
    leftWheelVelocity = m_fDefaultWheelVelocity;
    rightWheelVelocity = -m_fDefaultWheelVelocity;
    for (int i = 0; i < 5; i++)
    {
        led_colors[i] = sensor_color;
    }
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
}

void CRVR::AutomodeRWStep()
{
    switch (m_eExplorationState)
    {
    case RANDOM_WALK:
    {
        leftWheelVelocity = m_fDefaultWheelVelocity;
        rightWheelVelocity = m_fDefaultWheelVelocity;
        m_pcWheels->SetLinearVelocity(m_fDefaultWheelVelocity, m_fDefaultWheelVelocity);
        if (IsObstacleInFront(SumProxReadingsArray(prox_readings)))
        {
            m_eExplorationState = OBSTACLE_AVOIDANCE;
            m_unTurnSteps = m_pcRng->Uniform(m_cRandomStepsRange);
            CRadians cAngle = SumProxReadingsArray(prox_readings).Angle.SignedNormalize();
            if (cAngle.GetValue() < 0)
            {
                m_eTurnDirection = LEFT;
            }
            else
            {
                m_eTurnDirection = RIGHT;
            }
        }
        break;
    }
    case OBSTACLE_AVOIDANCE:
    {
        m_unTurnSteps -= 1;
        switch (m_eTurnDirection)
        {
        case LEFT:
        {
            m_pcWheels->SetLinearVelocity(-m_fDefaultWheelVelocity, m_fDefaultWheelVelocity);
            leftWheelVelocity = -m_fDefaultWheelVelocity;
            rightWheelVelocity = m_fDefaultWheelVelocity;
            break;
        }
        case RIGHT:
        {
            m_pcWheels->SetLinearVelocity(m_fDefaultWheelVelocity, -m_fDefaultWheelVelocity);
            leftWheelVelocity = m_fDefaultWheelVelocity;
            rightWheelVelocity = -m_fDefaultWheelVelocity;
            break;
        }
        }
        if (m_unTurnSteps <= 0)
        {
            m_eExplorationState = RANDOM_WALK;
        }
        break;
    }
    }
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
    // /*
    CVector2 speeds = ComputeWheelsVelocityFromVector(CVector2(1.0, stepAngle));
    leftWheelVelocity = speeds.GetX();
    rightWheelVelocity = speeds.GetY();
    m_pcWheels->SetLinearVelocity(leftWheelVelocity, rightWheelVelocity);
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
    // std::cout << "theta : " << theta << std::endl;
    // std::cout << "angle : " << angle << std::endl;
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
    static tf::TransformBroadcaster br; // needed for tf transform

    tf::Transform transform;
    tf::Quaternion q;
    std::stringstream parent;
    std::stringstream child;

    std::stringstream name;
    name.str("");
    name << GetId();
    // publish velocity
    vel_msg.data.clear();
    vel_msg.data.push_back(round(leftWheelVelocity / 100)); // convert cm/s to m/s
    vel_msg.data.push_back(round(rightWheelVelocity / 100));
    vel_pub.publish(vel_msg);
    for (int i = 0; i < 5; i++)
    {
        led_msg.led_colors[i].r = led_colors[i].GetRed();
        led_msg.led_colors[i].g = led_colors[i].GetGreen();
        led_msg.led_colors[i].b = led_colors[i].GetBlue();
    }
    // publish leds color
    led_pub.publish(led_msg);
    // send odometry message for mapping
    odomMsg.header.stamp = ros::Time::now();
    std::stringstream ss;
    ss.str("");
    ss << name.str() << "/odom";
    odomMsg.header.frame_id = ss.str();
    ss.str("");
    ss << name.str() << "/base_link";
    odomMsg.child_frame_id = ss.str();
    odomMsg.pose.pose.position.x = xPos;
    odomMsg.pose.pose.position.y = yPos;
    odomMsg.pose.pose.position.z = 0;
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta); // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    odomMsg.pose.pose.orientation = odomQuat;
    currentTime = ros::Time::now();
    odomMsg.twist.twist.linear.x = deltaSteps / (abs((currentTime - lastTime).toSec()));
    odomMsg.twist.twist.angular.z = deltaTheta / (abs((currentTime - lastTime).toSec()));

    lastTime = ros::Time::now();

    mapping_odom_pub.publish(odomMsg);

    // odometry transform
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = odomMsg.header.stamp;
    odomTrans.header.frame_id = odomMsg.header.frame_id;
    odomTrans.child_frame_id = odomMsg.child_frame_id;
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;
    // std::cout << "x : " << xPos << ", y : " << yPos << std::endl;
    // std::cout << "theta : " << theta << std::endl;
    br.sendTransform(odomTrans);

    if (!rvr_driven)
    {
        // create laser message for mapping
        for (int i = 0; i < 719; i++)
        {
            if (laserMsg.ranges[i] > laserMsg.range_max)
            {
                laserMsg.ranges[i] = laserMsg.range_max;
            }
            else if (laserMsg.ranges[i] < laserMsg.range_min)
            {
                laserMsg.ranges[i] = laserMsg.range_min;
            }
            else
            {
                laserMsg.ranges[i] = lidar_readings[i];
            }
        }
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.03));
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        parent.str("");
        child.str("");
        parent << name.str() << "/base_laser";
        child << name.str() << "/base_link";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        laserMsg.header.stamp = ros::Time::now();
        mapping_laser_pub.publish(laserMsg);
    }
}

void CRVR::OdometryUpdate()
{
    // odometry computation
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
    sensor_color.SetRed((argos::UInt8)msg.r);
    sensor_color.SetGreen((argos::UInt8)msg.g);
    sensor_color.SetBlue((argos::UInt8)msg.b);
    sensor_color.SetAlpha((argos::UInt8)msg.a);
}

void CRVR::LightHandler(const sensor_msgs::Illuminance &msg)
{
    light = msg.illuminance;
}

void CRVR::ImuHandler(const sensor_msgs::Imu &msg)
{
    /* ORIENTATION */
    auto imu_quat = msg.orientation;
    // build matrix from quaternion
    tf::Matrix3x3 m(tf::Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w));
    tfScalar tfRoll, tfPitch, tfYaw;
    // convert to Euler angles
    m.getRPY(tfRoll, tfPitch, tfYaw);
    // copy to control variables
    roll = CRadians(tfRoll);
    pitch = CRadians(tfPitch);
    yaw = CRadians(tfYaw);
    /* VELOCITY */
    auto gyro_vel = msg.angular_velocity;
    angularVelocity.Set(gyro_vel.x, gyro_vel.y, gyro_vel.z);
    /* ACCELERATION */
    auto acc_lin = msg.linear_acceleration;
    acceleration.Set(acc_lin.x, acc_lin.y, acc_lin.z);
}

void CRVR::OdometryHandler(const nav_msgs::Odometry &msg)
{
    xPos = msg.pose.pose.position.x;
    yPos = msg.pose.pose.position.y;
    auto odom_quat = msg.pose.pose.orientation;
    quat_reading.Set(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
    auto odom_angular = msg.twist.twist.angular;
    angularVelocity.Set(odom_angular.x, odom_angular.y, odom_angular.z);
    auto odom_linear = msg.twist.twist.linear;
}

void CRVR::TerarangerHandler(const teraranger_array::RangeArray &msg)
{
    for (short int i = 0; i < 8; ++i)
    {
	if(msg.ranges[i].range <= 0.4)
        	prox_readings[i] = Exp(-msg.ranges[i].range);
	else
		prox_readings[i] = 0.0f;
	CRange<Real>(0.0f, 1.0f).TruncValue(prox_readings[i]);
    }
}

void CRVR::LidarHandler(const sensor_msgs::LaserScan &msg)
{
    for (short int i = 0; i < 719; ++i)
        lidar_readings[i] = msg.ranges[i];
    // send it back for mapping
    mapping_laser_pub.publish(msg);
}

bool CRVR::IsObstacleInFront(CCI_RVRProximitySensor::SReading s_prox_reading)
{
    CRadians cAngle = s_prox_reading.Angle;
    if (s_prox_reading.Value >= m_fProximityThreshold && ((cAngle <= CRadians::PI_OVER_TWO) && (cAngle >= -CRadians::PI_OVER_TWO)))
    {
        return true;
    }
    return false;
}

CCI_RVRProximitySensor::SReading CRVR::SumProxReadingsArray(Real *s_prox_reading)
{
    CCI_RVRProximitySensor::SReading cOutputReading;
    CVector2 cSumProxi(0, CRadians::ZERO);
    for (UInt8 i = 0; i < 8; i++)
    {
        auto angle = m_pcProximitySensor->GetReading(i).Angle;
	cSumProxi += CVector2(s_prox_reading[i], angle.SignedNormalize());
    }

    cOutputReading.Value = (cSumProxi.Length() > 1) ? 1 : cSumProxi.Length();
    cOutputReading.Angle = cSumProxi.Angle().SignedNormalize();
    return cOutputReading;
}

REGISTER_CONTROLLER(CRVR, "rvr_foraging")
