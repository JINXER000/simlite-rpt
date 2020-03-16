//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_quadrotor_controller/quadrotor_interface.h>
#include <hector_quadrotor_controller/pid.h>

#include <controller_interface/controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <common_msgs/state.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <limits>

namespace hector_quadrotor_controller {

using namespace controller_interface;

class PVAController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  PVAController()
  {}

  ~PVAController()
  {}

  bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_          = interface->getPose();
    twist_         = interface->getTwist();
    acceleration_  = interface->getAcceleration();
    pose_input_   =interface->addInput<PoseCommandHandle>("pose");
    twist_input_   = interface->addInput<TwistCommandHandle>("twist");
//    pva_input_  =interface->addInput<common_msgs::state>("pva_cmd");
    wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench");
    node_handle_ = root_nh;

    // subscribe to commanded twist (geometry_msgs/TwistStamped) and cmd_vel (geometry_msgs/Twist)
//    pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>("command/pose", 1, boost::bind(&PVAController::poseCommandCallback, this, _1));
//    twist_subscriber_ = node_handle_.subscribe<geometry_msgs::TwistStamped>("command/twist", 1, boost::bind(&PVAController::twistCommandCallback, this, _1));
    pva_subscriber_ = node_handle_.subscribe<common_msgs::state>("commonCMD/pva", 1, boost::bind(&PVAController::pvaCommandCallback, this, _1));
    // engage/shutdown service servers
    engage_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("engage", boost::bind(&PVAController::engageCallback, this, _1, _2));
    shutdown_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("shutdown", boost::bind(&PVAController::shutdownCallback, this, _1, _2));

    // initialize PID controllers
//    pid_.linear.x.init(ros::NodeHandle(controller_nh, "linear/xy"));
//    pid_.linear.y.init(ros::NodeHandle(controller_nh, "linear/xy"));
//    pid_.linear.z.init(ros::NodeHandle(controller_nh, "linear/z"));
    pid_.angular.x.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.y.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.z.init(ros::NodeHandle(controller_nh, "angular/z"));

    // initalize RPT or LQR
    lqr_.x.init(ros::NodeHandle(controller_nh, "LQR/xy"));
    lqr_.y.init(ros::NodeHandle(controller_nh, "LQR/xy"));
    lqr_.z.init(ros::NodeHandle(controller_nh, "LQR/z"));

    // load other parameters
    controller_nh.getParam("auto_engage", auto_engage_ = false);
    controller_nh.getParam("limits/load_factor", load_factor_limit = 1.5);
    controller_nh.getParam("limits/force/z", limits_.force.z);
    controller_nh.getParam("limits/torque/xy", limits_.torque.x);
    controller_nh.getParam("limits/torque/xy", limits_.torque.y);
    controller_nh.getParam("limits/torque/z", limits_.torque.z);
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    // get mass and inertia from QuadrotorInterface
    interface->getMassAndInertia(mass_, inertia_);

    command_given_in_stabilized_frame_ = false;
    std::cout<<"init ok"<<std::endl;
    return true;
  }

  void reset()
  {
    pid_.linear.x.reset();
    pid_.linear.y.reset();
    pid_.linear.z.reset();
    pid_.angular.x.reset();
    pid_.angular.y.reset();
    pid_.angular.z.reset();

    wrench_.wrench.force.x  = 0.0;
    wrench_.wrench.force.y  = 0.0;
    wrench_.wrench.force.z  = 0.0;
    wrench_.wrench.torque.x = 0.0;
    wrench_.wrench.torque.y = 0.0;
    wrench_.wrench.torque.z = 0.0;

    linear_z_control_error_ = 0.0;
    motors_running_ = false;
  }
  void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr& command)
  {
    pose_command_ = *command;
    if (!(pose_input_->connected())) *pose_input_ = &(pose_command_.pose);
    pose_input_->start();

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);

  }

  void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    command_ = *command;
    if (command_.header.stamp.isZero()) command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = false; //--indicates in world frame

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);

  }

  void pvaCommandCallback(const common_msgs::stateConstPtr& commandPVA)
  {
    geometry_msgs::PoseStamped msgGe;
    msgGe.pose.position.x=commandPVA->pos.x;
    msgGe.pose.position.y=commandPVA->pos.y;
    msgGe.pose.position.z=commandPVA->pos.z;

    pose_command_ = msgGe;
    if (!(pose_input_->connected())) *pose_input_ = &(pose_command_.pose);
    pose_input_->start();

    geometry_msgs::TwistStamped msgtw;
    msgtw.twist.linear.x=commandPVA->vel.x;
    msgtw.twist.linear.y=commandPVA->vel.y;
    msgtw.twist.linear.z=commandPVA->vel.z;

    command_=msgtw;
    if (command_.header.stamp.isZero()) command_.header.stamp = ros::Time::now();


    AccCmd.x=commandPVA->acc.x;
    AccCmd.y=commandPVA->acc.y;
    AccCmd.z=commandPVA->acc.z;

//    pva_cmd=*commandPVA;
//    if (pva_cmd.header.stamp.isZero()) pva_cmd.header.stamp = ros::Time::now();

    ros::Time start_time = commandPVA->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);
  }
  bool engageCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Engaging motors!");
    motors_running_ = true;
    return true;
  }

  bool shutdownCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
    motors_running_ = false;
    return true;
  }

  void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {


    boost::mutex::scoped_lock lock(command_mutex_);
    //--no use
//    if(pose_input_->connected()&&pose_input_->enabled())
//    {
////      std::cout<<"start get pose input"<<std::endl;
//      pose_command_.pose=pose_input_->getCommand();
//    }
    // Get twist command input---no use
    if (twist_input_->connected() && twist_input_->enabled()) {
      command_.twist = twist_input_->getCommand();
      command_given_in_stabilized_frame_ = false;
       std::cout<<"get intrinsic twist  "<<command_.twist.linear.x<<","<<command_.twist.linear.y<<","<<command_.twist.linear.z<<std::endl;
    }

    // Get current state and command
//    Pose  pose_command=pose_command_.pose;
//    Pose pose_now=pose_->pose();
    Twist twist_command = command_.twist;
    Twist twist_now = twist_->twist();
    Twist twist_body;
    twist_body.linear =  pose_->toBody(twist_now.linear);
    twist_body.angular = pose_->toBody(twist_now.angular);

//    // Transform to world coordinates if necessary (yaw only),
    if (command_given_in_stabilized_frame_) { // indicates in body frame
      std::cout<<"vel body to world"<<std::endl;
      double yaw = pose_->getYaw();
      Twist transformed = twist_command;
      transformed.linear.x  = cos(yaw) * twist_command.linear.x  - sin(yaw) * twist_command.linear.y;
      transformed.linear.y  = sin(yaw) * twist_command.linear.x  + cos(yaw) * twist_command.linear.y;
      transformed.angular.x = cos(yaw) * twist_command.angular.x - sin(yaw) * twist_command.angular.y;
      transformed.angular.y = sin(yaw) * twist_command.angular.x + cos(yaw) * twist_command.angular.y;
      twist_command = transformed;
    }

    // Get gravity and load factor
    const double gravity = 9.8065;
    double load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w
                                 - pose_->pose().orientation.x * pose_->pose().orientation.x
                                 - pose_->pose().orientation.y * pose_->pose().orientation.y
                                 + pose_->pose().orientation.z * pose_->pose().orientation.z );
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit > 0.0 && !(load_factor < load_factor_limit)) load_factor = load_factor_limit;

    // Auto engage/shutdown
    // TODO NEED REVISE
    if (auto_engage_) {
      if (!motors_running_ && twist_now.linear.z > 0.1 && load_factor > 0.0) {
        motors_running_ = true;
        ROS_INFO_NAMED("twist_controller", "Engaging motors!");
      } else if (motors_running_ && twist_command.linear.z < -0.1 /* && (twist.linear.z > -0.1 && twist.linear.z < 0.1) */) {
        double shutdown_limit = 0.25 * std::min(twist_command.linear.z, -0.5);
        if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
        if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 5.0, period) < shutdown_limit) {
          motors_running_ = false;
          ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
        } else {
          ROS_DEBUG_STREAM_NAMED("twist_controller", "z control error = " << linear_z_control_error_ << " >= " << shutdown_limit);
        }
      } else {
        linear_z_control_error_ = 0.0;
      }
    }
    // flip over?
    if (motors_running_ && load_factor < 0.0) {
      motors_running_ = false;
      ROS_WARN_NAMED("twist_controller", "Shutting down motors due to flip over!");
    }
    // Update output
    if (motors_running_) {

      Vector3 acceleration_command,acceleration_command_tmp;
      double error_n, error_w,error_u;

      if (pose_input_->enabled()) {

        std::cout<<"get vel cmd  "<<command_.twist.linear.x<<","<<command_.twist.linear.y<<","<<command_.twist.linear.z<<std::endl;
        // control horizontal position
        HorizontalPositionCommandHandle(*pose_input_).getError(*pose_, error_n, error_w);
        error_u=HeightCommandHandle(*pose_input_).getError(*pose_);


        acceleration_command_tmp.x=lqr_.x.update(error_n,period,0);
        acceleration_command_tmp.y=lqr_.y.update(error_w,period,0);
        // control height
        acceleration_command_tmp.z=lqr_.z.update(error_u,period,0);
      }

      // update acc cmd in world frame
      acceleration_command.x=lqr_.x.update(twist_command.linear.x, twist_now.linear.x,period,1)+acceleration_command_tmp.x;
      acceleration_command.y=lqr_.y.update(twist_command.linear.y, twist_now.linear.y,period,1)+acceleration_command_tmp.y;
      acceleration_command.z=lqr_.z.update(twist_command.linear.z, twist_now.linear.z,period,1)+acceleration_command_tmp.z+ gravity;


      if(!std::isnan(AccCmd.x)&&!std::isnan(AccCmd.y)&&!std::isnan(AccCmd.z))
      {
        Vector3 acc_now=acceleration_->acceleration();
        acceleration_command.x+=(AccCmd.x/*-acc_now.x*/);
        acceleration_command.y+=(AccCmd.y/*-acc_now.y*/);
        acceleration_command.z+=(AccCmd.z/*-acc_now.z*/);

      }
      lqr_.x.Limitor(acceleration_command.x);
      lqr_.y.Limitor(acceleration_command.y);
      lqr_.z.Limitor(acceleration_command.z);


      //      acceleration_command.x = pid_.linear.x.update(twist_command.linear.x, twist_now.linear.x, acceleration_->acceleration().x, period);
//      acceleration_command.y = pid_.linear.y.update(twist_command.linear.y, twist_now.linear.y, acceleration_->acceleration().y, period);
//      acceleration_command.z = pid_.linear.z.update(twist_command.linear.z, twist_now.linear.z, acceleration_->acceleration().z, period) + gravity;
      Vector3 acceleration_command_body = pose_->toBody(acceleration_command);
      std::cout<<"get output acc  "<<acceleration_command<<std::endl;
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_now.linear:               [" << twist_now.linear.x << " " << twist.linear.y << " " << twist.linear.z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.linear:       [" << twist_command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.angular:      [" << twist_command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration:               [" << acceleration_->acceleration().x << " " << acceleration_->acceleration().y << " " << acceleration_->acceleration().z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
//      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_body:  [" << acceleration_command_body.x << " " << acceleration_command_body.y << " " << acceleration_command_body.z << "]");
      wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update(-acceleration_command_body.y / gravity, 0.0, twist_body.angular.x, period);
      wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, 0.0, twist_body.angular.y, period);
      wrench_.wrench.torque.z = inertia_[2] * pid_.angular.z.update( twist_command.angular.z, twist_now.angular.z, 0.0, period);
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = mass_ * ((acceleration_command.z - gravity) * load_factor + gravity);

      if (limits_.force.z > 0.0 && wrench_.wrench.force.z > limits_.force.z) wrench_.wrench.force.z = limits_.force.z;
      if (wrench_.wrench.force.z <= std::numeric_limits<double>::min()) wrench_.wrench.force.z = std::numeric_limits<double>::min();
      if (limits_.torque.x > 0.0) {
        if (wrench_.wrench.torque.x >  limits_.torque.x) wrench_.wrench.torque.x =  limits_.torque.x;
        if (wrench_.wrench.torque.x < -limits_.torque.x) wrench_.wrench.torque.x = -limits_.torque.x;
      }
      if (limits_.torque.y > 0.0) {
        if (wrench_.wrench.torque.y >  limits_.torque.y) wrench_.wrench.torque.y =  limits_.torque.y;
        if (wrench_.wrench.torque.y < -limits_.torque.y) wrench_.wrench.torque.y = -limits_.torque.y;
      }
      if (limits_.torque.z > 0.0) {
        if (wrench_.wrench.torque.z >  limits_.torque.z) wrench_.wrench.torque.z =  limits_.torque.z;
        if (wrench_.wrench.torque.z < -limits_.torque.z) wrench_.wrench.torque.z = -limits_.torque.z;
      }

      ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.force:       [" << wrench_.wrench.force.x << " " << wrench_.wrench.force.y << " " << wrench_.wrench.force.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.torque:      [" << wrench_.wrench.torque.x << " " << wrench_.wrench.torque.y << " " << wrench_.wrench.torque.z << "]");

    } else {
      reset();
    }

    // set wrench output
    wrench_.header.stamp = time;
    wrench_.header.frame_id = base_link_frame_;
    wrench_output_->setCommand(wrench_.wrench);
  }

private:
  //state
  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr acceleration_;
  //cmd
  PoseCommandHandlePtr pose_input_;
  TwistCommandHandlePtr twist_input_;
//  PVACommandHandlePtr pva_input_;
  //ctrl
  WrenchCommandHandlePtr wrench_output_;

  ros::NodeHandle node_handle_;
  ros::Subscriber pva_subscriber_;
  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  geometry_msgs::PoseStamped pose_command_;
  geometry_msgs::TwistStamped command_;
  common_msgs::state pva_cmd;
  Vector3 AccCmd;
  geometry_msgs::WrenchStamped wrench_;
  bool command_given_in_stabilized_frame_;
  std::string base_link_frame_;

  struct {
    struct {
      PID x;
      PID y;
      PID z;
    } linear, angular;
  } pid_;

  struct
  {
    LQR x,y,z;
  }lqr_;
  geometry_msgs::Wrench limits_;
  bool auto_engage_;
  double load_factor_limit;
  double mass_;
  double inertia_[3];

  bool motors_running_;
  double linear_z_control_error_;
  boost::mutex command_mutex_;

};

} // namespace hector_quadrotor_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller::PVAController, controller_interface::ControllerBase)
