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
    int ctrlMethod;
    n.getParam("/refPub/ctrlMethod", ctrlMethod=posCtrl);
    ros::Publisher ref_pos_pub = n.advertise<geometry_msgs::PoseStamped>("command/pose",1);
    ros::Publisher ref_vel_pub = n.advertise<geometry_msgs::TwistStamped>("command/twist",1);
    // ros::Publisher ref_pva_pub=n.advertise<common_msgs::state>("commonCMD/pva",1);
    // ros::Duration(3).sleep(); 
    ros::Rate loop_rate(20);

    /* Read the reference txt file*/
    std::ifstream infile;
    std::string line;
    infile.open(refAdr.c_str());

    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("/engage", eReq, eRes);

    float refout[11];
    int count = 0;


    std::vector<common_msgs::state> ref_list;
    bool if_replay=0;
    while (ros::ok())
    {
        // ++count;
        
    if(ctrlMethod==0)
    {
        // if (refFile >> refout[0]>> refout[1]>> refout[2]>> refout[3]>> refout[4]>> refout[5]
        //         >> refout[6]>> refout[7]>> refout[8]>> refout[9]>> refout[10])
        // {	
	    // geometry_msgs::PoseStamped msgGe;
        //     msgGe.pose.position.x = refout[0];
        //     msgGe.pose.position.y = -refout[1];
        //     msgGe.pose.position.z = -refout[2];
	    // ref_pos_pub.publish(msgGe);
        // }
            ROS_WARN("cfg fail!\n");
    }else
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
           
            p.pos.x = _1-2.5;
            p.pos.y = _2+2;
            p.pos.z = _3-0.25;
            p.vel.x = _4;
            p.vel.y = _5;
            p.vel.z = _6;
            p.acc.x = _7;
            p.acc.y = _8;
            p.acc.z = _9;
            ref_list.push_back(p);        
            // ref_pva_pub.publish(p);

        
        }else
        {
            if(count+1>=ref_list.size())
                count=0;
            p=ref_list[count];
            // ref_pva_pub.publish(p);
            count++;
        

        }
            geometry_msgs::PoseStamped cmdPos;
            cmdPos.pose.position.x=p.pos.x;
            cmdPos.pose.position.y=p.pos.y;
            cmdPos.pose.position.z=p.pos.z;
            ref_pos_pub.publish(cmdPos);

            geometry_msgs::TwistStamped cmdVel;
            cmdVel.twist.linear.x=p.vel.x;
             cmdVel.twist.linear.y=p.vel.y;            
             cmdVel.twist.linear.z=p.vel.z;      
             ref_vel_pub.publish(cmdVel);     

        std::cout<<" P,V,A REF is "<<p.pos.x<<",  "<< p.pos.y<<",  "<< p.pos.z<<",  "<< p.vel.x<<",  "<< p.vel.y<<
        ",  "<< p.vel.z<<",  "<< p.acc.x<<",  "<< p.acc.y<<",  "<< p.acc.z<<std::endl;

    }
    

        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}
