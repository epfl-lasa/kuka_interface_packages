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

#include "RobotStatePublisher.h"
#include <boost/lexical_cast.hpp>

RobotStatePublisher::RobotStatePublisher()
    :RobotInterface() , mLWRRobot(0), ndof(0), nh(0){
}
RobotStatePublisher::~RobotStatePublisher(){
}

RobotInterface::Status RobotStatePublisher::RobotInit(){

    mLWRRobot = (LWRRobot*)mRobot;

    string nodename = mRobot->GetName();
    nodename += "_MIRROR";

    nh = mRobot->InitializeROS(nodename);

	GetConsole()->Print("Robot Init Publisher");
	float new_sampling_time = 0.002;
//	((LWRRobot*)(mRobot))->SetSamplingTime(new_sampling_time);
	mLWRRobot->SetSamplingTime(new_sampling_time);
//	dt = ((LWRRobot*)(mRobot))->GetSamplingTime();
	dt = mLWRRobot->GetSamplingTime();
	std::ostringstream ss;
	ss << "DT: " << dt;
	std::string msg(ss.str());
	GetConsole()->Print(msg);


    string topicName;

    topicName = mRobot->GetName();
    topicName = "/joint_states";

    jointStatePublisher = nh->advertise<sensor_msgs::JointState>(topicName,3);
    topicName = mRobot->GetName();
    topicName += "/Pose";
    posePublisher = nh->advertise<geometry_msgs::PoseStamped>(topicName,3);

//    topicName = mRobot->GetName();
//    topicName += "/Velocity";
//    velocityPublisher = nh->advertise<geometry_msgs::TwistStamped>(topicName,100);

//    topicName = mRobot->GetName();
//    topicName += "/VelocityFiltered";
//    filteredVelocityPublisher = nh->advertise<geometry_msgs::TwistStamped>(topicName,3);

    topicName = mRobot->GetName();
    topicName += "/FT";
    ftPublisher = nh->advertise<geometry_msgs::WrenchStamped>(topicName,3);

    topicName = mRobot->GetName();
    topicName += "/Stiff";
    stiffPublisher = nh->advertise<geometry_msgs::TwistStamped>(topicName,3);

    mSensorsGroup.SetSensorsList(mRobot->GetSensors());


    int linkid = mRobot->GetLinkIndex("TOOL");


    mKinematicChain.SetRobot(mRobot);
    //mKinematicChain.Create(0,0,mRobot->GetLinkIndex("TOOLPLATE"));

    mKinematicChain.Create(0,0,linkid);
    mKinematicChain.BuildJacobian();

    mJacobian.Resize(6,mRobot->GetDOFCount());
    currVel.Resize(6);

    currVel_filtered.Resize(6);
    jointVel.Resize(mRobot->GetDOFCount());


    velocityFilter.Init(6,4);
    velocityFilter.SetSamplingPeriod(0.002);
    velocityFilter.SetButterworth(20.0);

    eeFT.Resize(6);
    eeStiff.Resize(6);

    ndof = 0;
    while(true)
    {
    	if(mRobot->GetDOFIndex("DOF_" + boost::lexical_cast<string>(ndof++)) < 0)
    		break;
    }

    if(ndof == 1)
    {
    	cout<<"No DOF found!"<<endl;
    	exit(1);
    }
    ndof--;
    joint_map.resize(ndof);

    for(int i=0;i<ndof;i++)
    	joint_map[i] = mRobot->GetDOFIndex("DOF_" + boost::lexical_cast<string>(i));


    jointStateMsg.position.resize(ndof);
    jointStateMsg.velocity.resize(ndof);
    jointStateMsg.effort.resize(ndof);
    jointStateMsg.name.resize(ndof);
    char buf[255];
    pXmlTree options = GetOptionTree();
    string which_arm;
    if(options) {
    	which_arm = options->CGet("Options.Arm", string("right"));
    } else {
    	which_arm = "right";
    }
    cout<<"Using as "<<which_arm<<" arm";

    for(int i=0; i<ndof; ++i) {
    	sprintf(buf, "%s_arm_%d_joint",which_arm.c_str(), i);
    	jointStateMsg.name[i] = buf;
    }


    topicName = mRobot->GetName();
    topicName += "/CoreRate";
    //rosrt::init();
    //ratePublisher = new rosrt::Publisher<std_msgs::Empty>(*nh,topicName,1,10,std_msgs::Empty());
    //emptyMsg  = ratePublisher->allocate();

    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status RobotStatePublisher::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdate(){

  // Removing spin from here since the robot object is spinning ros now.
  //ros::spinOnce();
//   for(int ii =0;ii<3;ii++){
//        for( int jj = 0;jj<3;jj++){
//            rot_Eigen(ii,jj) = rot_MathLib(ii,jj);
//        }
//    }
//
//
//    //cout<<rot_Eigen<<endl;
//    //rot_MathLib.Print();
//    Eigen::Quaternion<double> rot_quat(rot_Eigen);
//    //cout<<rot_quat.w()<<endl;
//    //cout<<rot_quat.toRotationMatrix()<<endl;
//    for(int i=0;i<ndof;i++){
//        jointStateMsg.position[i] = mSensorsGroup.GetJointAngles()(joint_map[i]);
//        jointStateMsg.velocity[i] = mSensorsGroup.GetJointVelocities()(joint_map[i]);
//        jointStateMsg.effort[i] = mSensorsGroup.GetJointTorques()(joint_map[i]);
//    }
//
//    jointStateMsg.header.stamp = ros::Time::now();
//
//    poseStampedMsg.pose.position.x = currEEPos(0);
//    poseStampedMsg.pose.position.y = currEEPos(1);
//    poseStampedMsg.pose.position.z = currEEPos(2);
//    poseStampedMsg.pose.orientation.w = rot_quat.w();
//    poseStampedMsg.pose.orientation.x = rot_quat.x();
//    poseStampedMsg.pose.orientation.y = rot_quat.y();
//    poseStampedMsg.pose.orientation.z = rot_quat.z();
//
//    poseStampedMsg.header.stamp = ros::Time::now();
//
//    twistStampedMsg.twist.linear.x = currVel(0);
//    twistStampedMsg.twist.linear.y = currVel(1);
//    twistStampedMsg.twist.linear.z = currVel(2);
//    twistStampedMsg.twist.angular.x = currVel(3);
//    twistStampedMsg.twist.angular.y = currVel(4);
//    twistStampedMsg.twist.angular.z = currVel(5);
//
//    twistStampedMsg.header.stamp = ros::Time::now();
//
////    velocityPublisher.publish(twistStampedMsg);
//
//
//    twistStampedMsg.twist.linear.x = currVel_filtered(0);
//    twistStampedMsg.twist.linear.y = currVel_filtered(1);
//    twistStampedMsg.twist.linear.z = currVel_filtered(2);
//    twistStampedMsg.twist.angular.x = currVel_filtered(3);
//    twistStampedMsg.twist.angular.y = currVel_filtered(4);
//    twistStampedMsg.twist.angular.z = currVel_filtered(5);
//
//    twistStampedMsg.header.stamp = ros::Time::now();
////    filteredVelocityPublisher.publish(twistStampedMsg);
//    jointStatePublisher.publish(jointStateMsg);
//    posePublisher.publish(poseStampedMsg);
//
//    // New message for Cart FT
//    ftMsg.header.stamp = ros::Time::now();
////    ftMsg.twist.linear.x = eeFT[0];
////    ftMsg.twist.linear.y = eeFT[1];
////    ftMsg.twist.linear.z = eeFT[2];
////    ftMsg.twist.angular.x = eeFT[3];
////    ftMsg.twist.angular.y = eeFT[4];
////    ftMsg.twist.angular.z = eeFT[5];
//    ftMsg.wrench.force.x = eeFT[0];
//    ftMsg.wrench.force.y = eeFT[1];
//    ftMsg.wrench.force.z = eeFT[2];
//    ftMsg.wrench.torque.x = eeFT[3];
//    ftMsg.wrench.torque.y = eeFT[4];
//    ftMsg.wrench.torque.z = eeFT[5];
//    ftPublisher.publish(ftMsg);
//
//    // New message for Cart Stiffness
//    // New message for Cart FT
//    stiffMsg.header.stamp = ros::Time::now();
//    stiffMsg.twist.linear.x = eeStiff[0];
//    stiffMsg.twist.linear.y = eeStiff[1];
//    stiffMsg.twist.linear.z = eeStiff[2];
//    stiffMsg.twist.angular.x = eeStiff[3];
//    stiffMsg.twist.angular.y = eeStiff[4];
//    stiffMsg.twist.angular.z = eeStiff[5];
//    stiffPublisher.publish(stiffMsg);

    return STATUS_OK;
}
RobotInterface::Status RobotStatePublisher::RobotUpdateCore(){
    //ratePublisher->publish(emptyMsg);
    // not safe for realtime
    mSensorsGroup.ReadSensors();
    mKinematicChain.Update();
    mJacobian = mKinematicChain.GetJacobian();
    jointVel = mSensorsGroup.GetJointVelocities();
    mJacobian.Mult(jointVel,currVel);
    velocityFilter.SetInput(currVel);
    velocityFilter.Update();
    velocityFilter.GetOutput(currVel_filtered);


    currEEPos  = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrigin();
    rot_MathLib = mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrient();

    eeFT = ((LWRRobot*)mRobot)->GetEstimatedExternalCartForces();
//    Vector3 tmp1;
//    tmp1 = rot_MathLib*Vector3(eeFT(0), eeFT(1), eeFT(2));
//    tmp1 = Vector3(eeFT(0), eeFT(1), eeFT(2));
//    eeFT(0) = -1*tmp1(0);
//    eeFT(1) = -1*tmp1(1);
//    eeFT(2) = -1*tmp1(2);

//    tmp1 = rot_MathLib*Vector3(eeFT(3), eeFT(4), eeFT(5));
//    tmp1 = Vector3(eeFT(3), eeFT(4), eeFT(5));
//    eeFT(3) = -1*tmp1(0);
//    eeFT(4) = -1*tmp1(1);
//    eeFT(5) = -1*tmp1(2);

    eeStiff = ((LWRRobot*)mRobot)->GetCartStiffness();
//    tmp1 = rot_MathLib*Vector3(eeStiff(0), eeStiff(1), eeStiff(2));
//    eeStiff(0) = tmp1(0);
//    eeStiff(1) = tmp1(1);
//    eeStiff(2) = tmp1(2);
//
//    tmp1 = rot_MathLib*Vector3(eeStiff(3), eeStiff(4), eeStiff(5));
//    eeStiff(3) = tmp1(0);
//    eeStiff(4) = tmp1(1);
//    eeStiff(5) = tmp1(2);



    for(int ii =0;ii<3;ii++){
         for( int jj = 0;jj<3;jj++){
             rot_Eigen(ii,jj) = rot_MathLib(ii,jj);
         }
     }


     //cout<<rot_Eigen<<endl;
     //rot_MathLib.Print();
     Eigen::Quaternion<double> rot_quat(rot_Eigen);
     //cout<<rot_quat.w()<<endl;
     //cout<<rot_quat.toRotationMatrix()<<endl;
     for(int i=0;i<ndof;i++){
         jointStateMsg.position[i] = mSensorsGroup.GetJointAngles()(joint_map[i]);
         jointStateMsg.velocity[i] = mSensorsGroup.GetJointVelocities()(joint_map[i]);
         jointStateMsg.effort[i] = mSensorsGroup.GetJointTorques()(joint_map[i]);
     }

     jointStateMsg.header.stamp = ros::Time::now();

     poseStampedMsg.pose.position.x = currEEPos(0);
     poseStampedMsg.pose.position.y = currEEPos(1);
     poseStampedMsg.pose.position.z = currEEPos(2);
     poseStampedMsg.pose.orientation.w = rot_quat.w();
     poseStampedMsg.pose.orientation.x = rot_quat.x();
     poseStampedMsg.pose.orientation.y = rot_quat.y();
     poseStampedMsg.pose.orientation.z = rot_quat.z();

     poseStampedMsg.header.stamp = ros::Time::now();

     twistStampedMsg.twist.linear.x = currVel(0);
     twistStampedMsg.twist.linear.y = currVel(1);
     twistStampedMsg.twist.linear.z = currVel(2);
     twistStampedMsg.twist.angular.x = currVel(3);
     twistStampedMsg.twist.angular.y = currVel(4);
     twistStampedMsg.twist.angular.z = currVel(5);

     twistStampedMsg.header.stamp = ros::Time::now();

 //    velocityPublisher.publish(twistStampedMsg);


     twistStampedMsg.twist.linear.x = currVel_filtered(0);
     twistStampedMsg.twist.linear.y = currVel_filtered(1);
     twistStampedMsg.twist.linear.z = currVel_filtered(2);
     twistStampedMsg.twist.angular.x = currVel_filtered(3);
     twistStampedMsg.twist.angular.y = currVel_filtered(4);
     twistStampedMsg.twist.angular.z = currVel_filtered(5);

     twistStampedMsg.header.stamp = ros::Time::now();
 //    filteredVelocityPublisher.publish(twistStampedMsg);
     jointStatePublisher.publish(jointStateMsg);
     posePublisher.publish(poseStampedMsg);

     // New message for Cart FT
     ftMsg.header.stamp = ros::Time::now();
 //    ftMsg.twist.linear.x = eeFT[0];
 //    ftMsg.twist.linear.y = eeFT[1];
 //    ftMsg.twist.linear.z = eeFT[2];
 //    ftMsg.twist.angular.x = eeFT[3];
 //    ftMsg.twist.angular.y = eeFT[4];
 //    ftMsg.twist.angular.z = eeFT[5];
     ftMsg.wrench.force.x = eeFT[0];
     ftMsg.wrench.force.y = eeFT[1];
     ftMsg.wrench.force.z = eeFT[2];
     ftMsg.wrench.torque.x = eeFT[3];
     ftMsg.wrench.torque.y = eeFT[4];
     ftMsg.wrench.torque.z = eeFT[5];
     ftPublisher.publish(ftMsg);

     // New message for Cart Stiffness
     // New message for Cart FT
     stiffMsg.header.stamp = ros::Time::now();
     stiffMsg.twist.linear.x = eeStiff[0];
     stiffMsg.twist.linear.y = eeStiff[1];
     stiffMsg.twist.linear.z = eeStiff[2];
     stiffMsg.twist.angular.x = eeStiff[3];
     stiffMsg.twist.angular.y = eeStiff[4];
     stiffMsg.twist.angular.z = eeStiff[5];
     stiffPublisher.publish(stiffMsg);



    return STATUS_OK;
}
int RobotStatePublisher::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}



extern "C"{
// These two "C" functions manage the creation and destruction of the class
RobotStatePublisher* create(){return new RobotStatePublisher();}
void destroy(RobotStatePublisher* module){delete module;}
}
