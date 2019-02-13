/*
 File name: RobotContainerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */

#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>

#include <ctime>
#include <Eigen/Dense>

#include "ros_node_base/RosNodeModuleBase.hpp"
using namespace ros_node_utils;

namespace robot {
class RobotContainerBase : public RosNodeModuleBase
{
 public:
  RobotContainerBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        dt_(0.0),
        isSimulation_(true)
  {
  }
  ;

  virtual ~RobotContainerBase()
  {
  }
  ;



  virtual void readParameters()
  {
    paramRead(this->nodeHandle_ ,this->namespace_+  "/controller/simulation", isSimulation_);
    if (!isSimulation_) {
      WARNING("controller is running on real robot");
    }
  }
  ;

  virtual void advance(double dt)
  {
  }
  ;
  virtual void publish()
  {
  }
  ;
  virtual void setTimeStep(double dt)
  {
    dt_ = dt;
  }
  ;

 protected:

  bool isSimulation_;
  std::mutex mutex_;
  double dt_;

};
}
