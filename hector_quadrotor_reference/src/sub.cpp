#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <std_srvs/Empty.h>
#include <common_msgs/state.h>
#include <common_msgs/target.h>

#define posCtrl 0
#define pvaCtrl 1

int main(int argc, char **argv)
{

    ros::init(argc, argv, "refPub");
    ros::NodeHandle n;
    std::string refAdr;
    
    n.getParam("/refPub/refAdr", refAdr="noFile");
    float init_pos[3]={0,8,0.5};//{2.5,-2,0.25};
    n.getParam("/refPub/initx", init_pos[0]=0);
    n.getParam("/refPub/inity", init_pos[1]=0);
    n.getParam("/refPub/initz", init_pos[2]=0);
    std::cout<<"init pos is("<<init_pos[0]<<","<<init_pos[1]<<","<<init_pos[2]<<")"<<std::endl;
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;

    int uavId=999;
    n.getParam("uavId", uavId);
    if(uavId==1)
    {
      init_pos[1]+=1;
      ros::service::call("/uav1/engage", eReq, eRes);
    }else if(uavId==2)
    {
      init_pos[1]-=1;
      ros::service::call("/uav2/engage", eReq, eRes);
    }else
    {
      ros::service::call("/engage", eReq, eRes);
    }
    std::cout<<"motor engaged!"<<std::endl;
    ros::Publisher ref_pos_pub = n.advertise<geometry_msgs::PoseStamped>("command/pose",1);
//    ros::Publisher ref_vel_pub = n.advertise<geometry_msgs::TwistStamped>("command/twist",1);
     ros::Publisher ref_pva_pub=n.advertise<common_msgs::state>("commonCMD/pva",1);

    ros::Rate loop_rate(50);

    /* Read the reference txt file*/
    std::ifstream infile;
    std::string line;
    infile.open(refAdr.c_str());

    int count = 0;


    std::vector<common_msgs::state> ref_list;
    bool if_replay=0;
    while (ros::ok())
    {

        

         common_msgs::state p;
        if(!if_replay)
        {
        std::getline(infile, line);
        std::istringstream iss(line);
        double _1, _2, _3, _4, _5, _6, _7, _8, _9;
            if (!(iss >> _1 >> _2 >> _3>> _4 >> _5 >> _6>> _7 >> _8 >> _9)) 
            { 
                ROS_ERROR("traj file end! restart from head\n");
                if_replay=1;
                count=0;
            } // error
           
            p.pos.x = _1-init_pos[0];
            p.pos.y = _2-init_pos[1];
            p.pos.z = _3-init_pos[2];
            p.vel.x = _4;
            p.vel.y = _5;
            p.vel.z = _6;
            p.acc.x = _7;
            p.acc.y = _8;
            p.acc.z = _9;
            ref_list.push_back(p);        
             ref_pva_pub.publish(p);

        
        }else
        {
            if(count+1>=ref_list.size())
                count=0;
            p=ref_list[count];
             ref_pva_pub.publish(p);
            count++;
        

        }
            geometry_msgs::PoseStamped cmdPos;
            cmdPos.pose.position.x=p.pos.x;
            cmdPos.pose.position.y=p.pos.y;
            cmdPos.pose.position.z=p.pos.z;
            ref_pos_pub.publish(cmdPos);

//            geometry_msgs::TwistStamped cmdVel;
//            cmdVel.twist.linear.x=p.vel.x;
//             cmdVel.twist.linear.y=p.vel.y;
//             cmdVel.twist.linear.z=p.vel.z;
//             ref_vel_pub.publish(cmdVel);

//        std::cout<<" P,V,A REF is "<<p.pos.x<<",  "<< p.pos.y<<",  "<< p.pos.z<<",  "<< p.vel.x<<",  "<< p.vel.y<<
//        ",  "<< p.vel.z<<",  "<< p.acc.x<<",  "<< p.acc.y<<",  "<< p.acc.z<<std::endl;

    

        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}
