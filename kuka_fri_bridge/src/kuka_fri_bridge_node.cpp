#include "ros/package.h"
#include <iostream>
#include <thread>

#include "kuka_fri_bridge/kuka_fri_bridge.h"
#include "kuka_fri_bridge/robot_state_publisher.h"


void update_kuka_fri_bridge(kfb::Kuka_fri_bridge& kuka_fri_bridge){
    while(true)
    {
        kuka_fri_bridge.update_core();
    }
}


int main(int argc, char** argv){


    std::cout<< "==== KUKA FRI BRIDGE ===" << std::endl;


    ros::init(argc, argv,"KUKA_FRI_BRIDGE");
    ros::NodeHandle nh("KUKA_FRI_BRIDGE");


    kfb::Kuka_fri_bridge kuka_fri_bridge;


    std::cout<< "before kuka_fri_bridge.initialise()"<<std::endl;
    kuka_fri_bridge.initialise();


    std::cout<< "before kuka_fri_bridge.start()"<<std::endl;
    // the LWRCore starts a console thread here
    if(kuka_fri_bridge.start() != 1)
    {
        kuka_fri_bridge.stop();;
        cout<<"Cannot start"<<endl;
        return 0;
    }

    kfb::Robot_state_publisher robot_state_publisher(nh,kuka_fri_bridge.GetLWRRobot());

    std::cout<< "before kuka_fri_bridge.update_core()"<<std::endl;

    while(true)
    {
        kuka_fri_bridge.update_core();
        robot_state_publisher.update();
        ros::spinOnce();
    }


    //std::thread fri_bridge_thread(update_kuka_fri_bridge);

   // auto f = boost::bind(test,_1);

    // boost::thread thead(boost::bind(update_kuka_fri_bridge,boost::ref(kuka_fri_bridge)));
    // thead.detach();
    //control loop is here

    // thead.join();
    // std::cout<< "after thread detach" << std::endl;

//    update_kuka_fri_bridge(kuka_fri_bridge);

    return 0;
}
