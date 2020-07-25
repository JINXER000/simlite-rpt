#ifndef HECTOR_QUADROTOR_CONTROLLER_PID_H
#define HECTOR_QUADROTOR_CONTROLLER_PID_H

#include <ros/node_handle.h>

namespace hector_quadrotor_controller {

class PID
{
public:
  struct parameters {
    parameters();
    bool enabled;
    double time_constant;
    double k_p;
    double k_i;
    double k_d;
    double limit_i;
    double limit_output;
  } parameters_;

  struct state {
    state();
    double p, i, d;
    double input, dinput;
    double dx;
  } state_;

public:
  PID();
  PID(const parameters& parameters);
  ~PID();

  void init(const ros::NodeHandle &param_nh);
  void reset();

  double update(double input, double x, double dx, const ros::Duration& dt);
  double update(double error, double dx, const ros::Duration& dt);

  double getFilteredControlError(double& filtered_error, double time_constant, const ros::Duration& dt);
};

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

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_PID_H
