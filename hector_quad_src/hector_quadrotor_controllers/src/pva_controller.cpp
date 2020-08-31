
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TwistStamped.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <limits>
#include <ros/subscriber.h>
#include <hector_quadrotor_interface/limiters.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <hector_quadrotor_interface/helpers.h>
#include <common_msgs/state.h>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class LQR
{

  double *k;

  double limit_output;
  double *input_old;
  bool enabled;

public:
  /// for RPT, let natural frequency=0.707, damping ratio=1.
  LQR(){
    k=new double[3];//{0.5,1.414,0}
    limit_output;//=10
    input_old=new double[3];
  }
  ~LQR(){
    delete k;
    delete input_old;
  }

  void init(const ros::NodeHandle &param_nh)
  {
    param_nh.getParam("K1", k[0]);
    param_nh.getParam("K2", k[1]);
    param_nh.getParam("K3", k[2]);
    param_nh.getParam("limit_output", limit_output);
  }

  double update(double input_new, double state,  const ros::Duration& dt,int direvitive)
  {
    if(direvitive>2||direvitive<0)
    {
      ROS_ERROR("can only handle p,v,a");
      return 0;
    }

    if(std::isnan(input_old[direvitive]))
      input_old[direvitive]=input_new;
    input_old[direvitive]=input_new*0.7+input_old[direvitive]*0.3;
    return update(input_old[direvitive]-state,dt,direvitive);
  }
  double update(double error,  const ros::Duration& dt,int direvitive)
  {
    if(direvitive>2||direvitive<0)
    {
      ROS_ERROR("can only handle p,v,a");
      return 0;
    }

    return k[direvitive]*error;
  }
  double Limitor(double output)
  {
    if(output>limit_output)
    {
      output=limit_output;
    }
    if(output<-limit_output)
    {
      output=-limit_output;
    }
    return output;
  }
};

class PVAController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  PVAController()
    : pose_command_valid_(false), twist_limit_valid_(false)
  {
  }

  virtual ~PVAController()
  {
    // pose_subscriber_.shutdown();
    // twist_limit_subscriber_.shutdown();
    pva_subscriber_.shutdown();
  }

  virtual bool init(QuadrotorInterface *interface,
                    ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    motor_status_ = interface->getMotorStatus();

    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "/world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    world_frame_ = tf::resolve(tf_prefix_, world_frame_);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    //    // Initialize PID controllers
    //    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    //    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    //    pid_.z.init(ros::NodeHandle(controller_nh, "z"));
    //    pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));

    // initalize RPT or LQR
    lqr_.x.init(ros::NodeHandle(controller_nh, "LQR/xy"));
    lqr_.y.init(ros::NodeHandle(controller_nh, "LQR/xy"));
    lqr_.z.init(ros::NodeHandle(controller_nh, "LQR/z"));

    position_limiter_.init(controller_nh);

    // Setup pose visualization marker output
    initMarker(root_nh.getNamespace());
    marker_publisher_ = root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

    // Initialize inputs/outputs
    pose_input_ = interface->addInput<PoseCommandHandle>("pose");
    twist_input_  = interface->addInput<TwistCommandHandle>("pose/twist");
    twist_limit_input_  = interface->addInput<TwistCommandHandle>("pose/twist_limit");
    twist_output_ = interface->addOutput<TwistCommandHandle>("twist");
    attitude_output_ = interface->addOutput<AttitudeCommandHandle>("attitude");
    yawrate_output_ = interface->addOutput<YawrateCommandHandle>("yawrate");
    thrust_output_ = interface->addOutput<ThrustCommandHandle>("thrust");

    // subscribe to commanded pose and velocity
    // pose_subscriber_ = root_nh.subscribe<geometry_msgs::PoseStamped>("command/pose", 1, boost::bind(
    //     &PVAController::poseCommandCallback, this, _1));
    // twist_limit_subscriber_ = root_nh.subscribe<geometry_msgs::Twist>("command/twist_limit", 1, boost::bind(
    //     &PVAController::twistLimitCallback, this, _1));
    pva_subscriber_ = root_nh.subscribe<common_msgs::state>("commonCMD/pva", 1, boost::bind(&PVAController::pvaCommandCallback, this, _1));

    getMassAndInertia(root_nh, mass_, inertia_);
    controller_nh.param("limits/load_factor", load_factor_limit_, 1.5);
    return true;
  }

  void reset()
  {
    //    pid_.x.reset();
    //    pid_.y.reset();
    //    pid_.z.reset();
    //    pid_.yaw.reset();

    // Set commanded pose to robot's current pose
    //    updatePoseCommand(pose_->pose());
    pose_command_valid_ = false;
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
  }

  virtual void stopping(const ros::Time &time)
  {
    twist_output_->stop();
    pose_command_valid_ = false;
    //    twist_limit_valid_ = false;
  }

  //  void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr &command)
  //  {
  //    boost::mutex::scoped_lock lock(command_mutex_);

  //    ros::Time start_time = command->header.stamp;
  //    if (start_time.isZero()) start_time = ros::Time::now();
  //    if (!isRunning()) this->startRequest(start_time);

  //    updatePoseCommand(*command);
  //  }

  // void twistLimitCallback(const geometry_msgs::TwistConstPtr &limit)
  // {
  //   boost::mutex::scoped_lock lock(command_mutex_);

  //   twist_limit_ = *limit;
  //   twist_limit_valid_ = true;
  // }

  void pvaCommandCallback(const common_msgs::stateConstPtr& commandPVA)
  {
    // add mutex
    //    boost::mutex::scoped_lock lock(command_mutex_);

    geometry_msgs::PoseStamped msgGe;
    msgGe.pose.position.x=commandPVA->pos.x;
    msgGe.pose.position.y=commandPVA->pos.y;
    msgGe.pose.position.z=commandPVA->pos.z;

    pose_command_ = msgGe;
    if (!(pose_input_->connected())) *pose_input_ = &(pose_command_.pose);
    pose_input_->start();
    pose_command_valid_ = true;

    geometry_msgs::TwistStamped msgtw;
    msgtw.twist.linear.x=commandPVA->vel.x;
    msgtw.twist.linear.y=commandPVA->vel.y;
    msgtw.twist.linear.z=commandPVA->vel.z;

    twist_command=msgtw;
    if (twist_command.header.stamp.isZero()) twist_command.header.stamp = ros::Time::now();


    AccCmd.x=commandPVA->acc.x;
    AccCmd.y=commandPVA->acc.y;
    AccCmd.z=commandPVA->acc.z;

    ros::Time start_time = commandPVA->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);
  }
  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    //    Twist output;

    // // Get pose command command input
    // if (pose_input_->connected() && pose_input_->enabled())
    // {
    //   updatePoseCommand(pose_input_->getCommand());
    // }

    // // Get twist limit input
    // if (twist_limit_input_->connected() && twist_limit_input_->enabled())
    // {
    //   twist_limit_ = twist_limit_input_->getCommand();
    //   twist_limit_valid_ = true;
    // }

    // check command timeout
    // TODO

    //        std::cout<<"pose_command_valid_ is"<<pose_command_valid_<<std::endl;
    // Check if pose control was preempted
    if (twist_output_->preempted()) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Position control preempted!");
      }
      pose_command_valid_ = false;
    }

    // Check if motors are running
    if (motor_status_->motorStatus().running == false) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Disabled position control while motors are not running.");
      }
      pose_command_valid_ = false;
    }
    // Abort if no pose command is available
    if (!pose_command_valid_) {
      //      reset();
      //      twist_output_->stop();
      return;
    } else {
      //      twist_output_->start();
      attitude_output_->start();
      yawrate_output_->start();
      thrust_output_->start();
    }



    // Get current pose,yaw and twist in body frame
    Pose pose = pose_->pose();
    double yaw = pose_->getYaw(), sin_yaw, cos_yaw;
    sincos(yaw, &sin_yaw, &cos_yaw);
    Twist twist_now = twist_->twist(), twist_body;
    twist_body.linear = pose_->toBody(twist_now.linear);
    twist_body.angular = pose_->toBody(twist_now.angular);


    // Get gravity and load factor
    const double gravity = 9.8065;
    double load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w
                                 - pose_->pose().orientation.x * pose_->pose().orientation.x
                                 - pose_->pose().orientation.y * pose_->pose().orientation.y
                                 + pose_->pose().orientation.z * pose_->pose().orientation.z );
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit_ > 0.0 && !(load_factor < load_factor_limit_)) load_factor = load_factor_limit_;


    pose_command_.pose.position = position_limiter_(pose_command_.pose.position);
    Vector3 acceleration_command,acceleration_command_tmp;
    double error_n, error_w,error_u;

    if (pose_input_->enabled()) {

      std::cout<<"get vel cmd  "<<twist_command.twist.linear.x<<","<<twist_command.twist.linear.y<<","<<twist_command.twist.linear.z<<std::endl;
      // control horizontal position
      HorizontalPositionCommandHandle(*pose_input_).getError(*pose_, error_n, error_w);
      error_u=HeightCommandHandle(*pose_input_).getError(*pose_);


      acceleration_command_tmp.x=lqr_.x.update(error_n,period,0);
      acceleration_command_tmp.y=lqr_.y.update(error_w,period,0);
      // control height
      acceleration_command_tmp.z=lqr_.z.update(error_u,period,0);
    }

    // update acc cmd in world frame
    acceleration_command.x=lqr_.x.update(twist_command.twist.linear.x, twist_now.linear.x,period,1)+acceleration_command_tmp.x;
    acceleration_command.y=lqr_.y.update(twist_command.twist.linear.y, twist_now.linear.y,period,1)+acceleration_command_tmp.y;
    acceleration_command.z=lqr_.z.update(twist_command.twist.linear.z, twist_now.linear.z,period,1)+acceleration_command_tmp.z+ gravity;

    if(!std::isnan(AccCmd.x)&&!std::isnan(AccCmd.y)&&!std::isnan(AccCmd.z))
    {
      geometry_msgs::Accel acc_now=acceleration_->acceleration();
      acceleration_command.x+=(AccCmd.x/*-acc_now.x*/);
      acceleration_command.y+=(AccCmd.y/*-acc_now.y*/);
      acceleration_command.z+=(AccCmd.z/*-acc_now.z*/);

    }
    lqr_.x.Limitor(acceleration_command.x);
    lqr_.y.Limitor(acceleration_command.y);
    lqr_.z.Limitor(acceleration_command.z);

    //    std::cout<<acceleration_command.x<<","<<acceleration_command.y<<","<<acceleration_command.z<<std::endl;
    // Transform acceleration command to base_stabilized frame
    Vector3 acceleration_command_base_stabilized;
    acceleration_command_base_stabilized.x  =  cos_yaw * acceleration_command.x  + sin_yaw * acceleration_command.y;
    acceleration_command_base_stabilized.y  = -sin_yaw * acceleration_command.x  + cos_yaw * acceleration_command.y;
    acceleration_command_base_stabilized.z  = acceleration_command.z;

    hector_uav_msgs::AttitudeCommand attitude_control;
    hector_uav_msgs::YawrateCommand yawrate_control;
    hector_uav_msgs::ThrustCommand thrust_control;
    attitude_control.roll    = -asin(std::min(std::max(acceleration_command_base_stabilized.y / gravity, -1.0), 1.0));
    attitude_control.pitch   =  asin(std::min(std::max(acceleration_command_base_stabilized.x / gravity, -1.0), 1.0));
    //    yawrate_control.turnrate = command.angular.z;
    thrust_control.thrust    = mass_ * ((acceleration_command.z - gravity) * load_factor + gravity);

    std::cout<<"mass is "<<mass_<<"; acctmpz is"<<acceleration_command_tmp.z<<"; accz is"<<acceleration_command.z<<"; "<<"thrust is "<<thrust_control.thrust<<std::endl;
    // pass down time stamp from twist command
    attitude_control.header.stamp = twist_command.header.stamp;
    yawrate_control.header.stamp = twist_command.header.stamp;
    thrust_control.header.stamp = twist_command.header.stamp;

    // Update output from controller
    attitude_output_->setCommand(attitude_control);
    yawrate_output_->setCommand(yawrate_control);
    thrust_output_->setCommand(thrust_control);
    // output.linear.x = pid_.x.computeCommand(pose_command_.position.x - pose.position.x, period);
    // output.linear.y = pid_.y.computeCommand(pose_command_.position.y - pose.position.y, period);
    // output.linear.z = pid_.z.computeCommand(pose_command_.position.z - pose.position.z, period);


    //    // YAW control
    //    double yaw_command;
    //    {
    //      tf2::Quaternion q;
    //      double temp;
    //      tf2::fromMsg(pose_command_.orientation, q);
    //      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
    //    }

    //    double yaw_error = yaw_command - yaw;
    //    // detect wrap around pi and compensate
    //    if (yaw_error > M_PI)
    //    {
    //      yaw_error -= 2 * M_PI;
    //    }
    //    else if (yaw_error < -M_PI)
    //    {
    //      yaw_error += 2 * M_PI;
    //    }
    //    output.angular.z = pid_.yaw.computeCommand(yaw_error, period);

    // // add twist command if available
    // if (twist_input_->connected() && twist_input_->enabled())
    // {
    //   output.linear.x  += twist_input_->getCommand().linear.x;
    //   output.linear.y  += twist_input_->getCommand().linear.y;
    //   output.linear.z  += twist_input_->getCommand().linear.z;
    //   output.angular.x += twist_input_->getCommand().angular.x;
    //   output.angular.y += twist_input_->getCommand().angular.y;
    //   output.angular.z += twist_input_->getCommand().angular.z;
    // }

    // // limit twist
    // if (twist_limit_valid_)
    // {
    //   double linear_xy = sqrt(output.linear.x*output.linear.x + output.linear.y*output.linear.y);
    //   double limit_linear_xy  = std::max(twist_limit_.linear.x, twist_limit_.linear.y);
    //   if (limit_linear_xy > 0.0 && linear_xy > limit_linear_xy) {
    //     output.linear.x *= limit_linear_xy / linear_xy;
    //     output.linear.y *= limit_linear_xy / linear_xy;
    //   }
    //   if (twist_limit_.linear.z > 0.0 && fabs(output.linear.z) > twist_limit_.linear.z) {
    //     output.linear.z *= twist_limit_.linear.z / fabs(output.linear.z);
    //   }
    //   double angular_xy = sqrt(output.angular.x*output.angular.x + output.angular.y*output.angular.y);
    //   double limit_angular_xy  = std::max(twist_limit_.angular.x, twist_limit_.angular.y);
    //   if (limit_angular_xy > 0.0 && angular_xy > limit_angular_xy) {
    //     output.angular.x *= limit_angular_xy / angular_xy;
    //     output.angular.y *= limit_angular_xy / angular_xy;
    //   }
    //   if (twist_limit_.angular.z > 0.0 && fabs(output.angular.z) > twist_limit_.angular.z) {
    //     output.angular.z *= twist_limit_.angular.z / fabs(output.angular.z);
    //   }
    // }

    // set twist output
    //    twist_output_->setCommand(output);
  }

private:
  //  void updatePoseCommand(const geometry_msgs::PoseStamped &new_pose)
  //  {
  //    // TODO TF to world frame
  //    if (new_pose.header.frame_id != world_frame_) {
  //      std::cout<<"new pose id is "<<new_pose.header.frame_id<<" and world frame is "<<world_frame_<<std::endl;
  //      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "position_controller", "Pose commands must be given in the " << world_frame_ << " frame, ignoring command");
  //    }
  //    else
  //    {
  //      updatePoseCommand(new_pose.pose);
  //    }
  //  }

  //  void updatePoseCommand(const geometry_msgs::Pose &new_pose)
  //  {
  //    {
  //      pose_command_.position = new_pose.position;
  //      // Strip non-yaw components from orientation
  //      tf2::Quaternion q;
  //      double roll, pitch, yaw;
  //      tf2::fromMsg(new_pose.orientation, q);
  //      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  //      q.setRPY(0, 0, yaw);
  //      pose_command_.orientation = tf2::toMsg(q);
  //      pose_command_valid_ = true;
  //    }
  //    pose_marker_.pose = pose_command_;
  //    marker_publisher_.publish(pose_marker_);
  //  }

  void initMarker(std::string name)
  {
    pose_marker_.header.frame_id = world_frame_;
    pose_marker_.ns = name;
    pose_marker_.id = 0;
    pose_marker_.type = visualization_msgs::Marker::ARROW;
    pose_marker_.scale.x = 0.15;
    pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.g = 0.5;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.a = 1.0;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr acceleration_;
  MotorStatusHandlePtr motor_status_;

  PoseCommandHandlePtr pose_input_;
  TwistCommandHandlePtr twist_input_;
  TwistCommandHandlePtr twist_limit_input_;
  TwistCommandHandlePtr twist_output_;

  AttitudeCommandHandlePtr attitude_output_;
  YawrateCommandHandlePtr yawrate_output_;
  ThrustCommandHandlePtr thrust_output_;

  hector_quadrotor_interface::PointLimiter position_limiter_;

  ros::Subscriber pose_subscriber_, twist_limit_subscriber_, pva_subscriber_;
  ros::Publisher marker_publisher_;

  visualization_msgs::Marker pose_marker_;

  geometry_msgs::PoseStamped pose_command_;
  geometry_msgs::TwistStamped twist_command;
  bool pose_command_valid_, twist_limit_valid_;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;
  std::string tf_prefix_;

  Vector3 AccCmd;
  //  struct
  //  {
  //    control_toolbox::Pid x, y, z, yaw;
  //  } pid_;
  struct
  {
    LQR x,y,z;
  }lqr_;

  double mass_;
  double inertia_[3];
  double load_factor_limit_;
  boost::mutex command_mutex_;
};

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::PVAController, controller_interface::ControllerBase)
