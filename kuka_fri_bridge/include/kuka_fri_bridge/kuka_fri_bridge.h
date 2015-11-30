#ifndef KUKA_FRI_BRIDGE_H_
#define KUKA_FRI_BRIDGE_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Empty.h"
//#include "eigen3/Eigen/Dense"


#include "std_tools/Console.h"
#include "std_tools/NCConsole.h"
#include "std_tools/Various.h"

//#include "RobotLib/RobotInterface.h"
//#include "MathLib/MathLib.h"
//#include "RobotLib/Sensor.h"
//#include "RobotLib/Robot.h"
//#include "LWRRobot.h"
#include "kuka_model/LWRRobot.h"
#include "FastResearchInterface.h"
#include "LinuxAbstraction.h"

//#inlcude "DisplayThread"
#include "boost/thread.hpp"
#include "time.h"
#include <deque>
#include <ros/package.h>

#include <armadillo>

#define MEASUREMENT_HISTORY_LENGTH 20

#define MODE_NONE 0
#define MODE_REC  1
#define MODE_PREP 2
#define MODE_GO   3

#ifndef RAD
#define RAD(A)	((A) * M_PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / M_PI )
#endif

#define FRI_CONN_TIMEOUT_SEC	30

#define JOINT_MAX_VEL_DEG_SEC  60.0
#define JOINT_MAX_ACC_DEG_SEC  60.0
#define CART_MAX_VEL_M_SEC  0.2
#define CART_MAX_ACC_M_SEC  0.05

#define FRI_JOINT_STIFFNESS 1000
#define FRI_JOINT_DAMPING   0.7

#define FRI_CART_STIFFNESS_POS 		300
#define FRI_CART_STIFFNESS_ORIENT 	30
#define FRI_CART_DAMPING_POS 		0.7
#define FRI_CART_DAMPING_ORIENT 	0.7

#define KUKA_DOF    7


/*this parameter determines how often the impedance should be updated relative to the position.
 * example: if REL_FREQ_IMPEDANCE_UPDATE = 10, then the desired impedance will be sent to the robot
 * every 10 iterations of desrired pose command update.*/
#define REL_FREQ_IMPEDANCE_UPDATE 10

namespace kfb {

typedef arma::colvec  Vector;
typedef arma::colvec3 Vector3;
typedef arma::mat44   Matrix4;
typedef arma::mat33   Matrix3;
typedef arma::mat     Matrix;

typedef enum ControlMode {
    CTRLMODE_NONE = 0,
    CTRLMODE_POSITION ,
    CTRLMODE_VELOCITY,
    CTRLMODE_ACCELERATION,
    CTRLMODE_TORQUE,
    CTRLMODE_DEFAULT,
    CTRLMODE_CARTIMPEDANCE,
    CTRLMODE_JOINTIMPEDANCE,
    CTRLMODE_GRAVITYCOMPENSATION,
} ControlMode;

// a little structure used for storing measurements from the robot
struct LWRMeasurement{

  arma::colvec JointPositions;
  arma::colvec JointVelocities;
  arma::colvec JointAccelerations;
  arma::colvec JointTorques;
  double t;

};

class Kuka_fri_bridge{

public:

       Kuka_fri_bridge();

       int initialise();
       int start();
       int stop();
       int update_core();

       int RespondToConsoleCommand(const string cmd, const vector<string> &args);

       bool SetControlMode(const int desiredMode);
       void SensorsUpdate();
       void ControlUpdate();


       //these are the methods for user interaction through console. they are run in a separate thread.
       void ConsoleLoop();
       void ConsoleUpdate();
       void SetCommandedJPos(float*);

       const robot::LWRRobot& GetLWRRobot() const{
           return mLWRRobot;
       }


private:

    FastResearchInterface * mFRI;
    robot::LWRRobot mLWRRobot;
    ControlMode     ctrl_mode;

    bool bRunConsoleLoop;
    boost::thread mDisplayThread;
    //  boost::thread mCoreThread;
    NCConsole mNCConsole;
    Console   mConsole;

    streambuf *mStdout;
    stringstream mOutputStream;
    char static_txt[1025];

    float cc[7];

    int nMode;
    int curr_traj_index;
    int nCurrQuality;
    bool bPrep;
    bool bGo;
    int nControl;
    double mCurrJDamp, mCurrJStiff;
    double mCurrCDamp_pos, mCurrCStiff_pos, mCurrCDamp_or, mCurrCStiff_or;

    arma::colvec tempJ_v; //NJOINTS-sized vector for intermediate data storage.
    arma::colvec tempC_v; // vector for intermediate storage of cartesian wrench (6d)
    arma::colvec3 temp3_1, temp3_2;

    arma::colvec LWR_state;


    float jnt2[LBR_MNJ];
    float cart2[FRI_CART_FRM_DIM];
    float cart_imp_params[FRI_CART_VEC];

    float** tempMassMatrix;
    float** tempJacobian;


    arma::colvec currJoint;
    arma::colvec currCart;

    //SensorsList     mJointSensors;
    //ActuatorsList   mJointActuators;

    std::deque<LWRMeasurement> MeasurementHistory;


};


}


#endif
