/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef RobotStatePublisher_H_
#define RobotStatePublisher_H_

//#include "RobotLib/RobotInterface.h"
//#include "RobotLib/KinematicChain.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Empty.h"
#include "eigen3/Eigen/Dense"
//#include "MathLib/Differentiator.h"

//#include "KUKARobotModel/LWRRobot.h"

#include "kuka_model/LWRRobot.h"

// ROS service

#include <memory>

#define KUKA_DOF 7

namespace kfb {


class Robot_state_publisher{


public:

    Robot_state_publisher(ros::NodeHandle& nh,const robot::LWRRobot& lWRobot);

    void initialise();

    void update();

private:

   // KinematicChain mKinematicChain;
   // Matrix mJacobian;
    arma::colvec currVel;
    arma::colvec currVel_filtered;
    arma::colvec jointVel;

    const robot::LWRRobot& mLWRRobot;

   // std::size_t ndof;
   // IndicesVector joint_map;
   // DigitalFilter velocityFilter;

    ros::Publisher jointStatePublisher;
    ros::Publisher jointStateImpedancePublisher;
    ros::Publisher posePublisher;
    ros::Publisher velocityPublisher;
    ros::Publisher filteredVelocityPublisher;
    ros::Publisher ftPublisher;
    ros::Publisher stiffPublisher;
    ros::NodeHandle * nh;

   // std_msgs::EmptyPtr emptyMsg;
    sensor_msgs::JointState jointStateMsg;
    kuka_fri_bridge::JointStateImpedance jointStateImpedanceMsg;
    geometry_msgs::PoseStamped poseStampedMsg;
    geometry_msgs::TwistStamped twistStampedMsg;
    geometry_msgs::WrenchStamped ftMsg;
    geometry_msgs::TwistStamped stiffMsg;
    //double dt;

    //RevoluteJointSensorGroup mSensorsGroup;
    //Vector3 currEEPos;
    //Eigen::Matrix3d rot_Eigen;
   // Eigen::Quaternion<double> rot_quat;
    //Matrix3 rot_MathLib;
    //Vector  eeFT;
    //Vector  eeStiff;

    // ROS Service



public:

};

}



#endif 
