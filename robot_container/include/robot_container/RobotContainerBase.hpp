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

#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_custom_hardware_adapter/HardwareAdapterFrameBase.hpp"

using namespace ros_node_utils;

namespace robot {

class RobotContainerBase : public RosNodeModuleBase
{
 public:
  RobotContainerBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        dt_(0.0),
        isSimulation_(true),
        stateMutex_()
  {
  }


  virtual ~RobotContainerBase()
  {
  }


  virtual void create()
  {
    RosNodeModuleBase::create();
   //CONFIRM("create : [Robot_Container_Base]");
  }

  virtual void readParameters()
  {
    paramRead(this->nodeHandle_,"/simulation", isSimulation_);
    if (!isSimulation_) {
      WARNING("controller is running on real robot");
    }
   //CONFIRM("readParameters : [Robot_Container_Base]");
  }
  ;

  virtual void initialize() override
  {
    RosNodeModuleBase::initialize();
   //CONFIRM("initialize : [Robot_Container_Base]");
  }

  virtual void shutdown() override
  {
    RosNodeModuleBase::shutdown();
   //ERROR("shutdown : [Robot_container_base]");
  }


  virtual void initializePublishers() override
  {
    RosNodeModuleBase::initializePublishers();
  }

  virtual void initializeSubscribers() override
  {
    RosNodeModuleBase::initializeSubscribers();
  }

  virtual void initializeServices() override
  {
    RosNodeModuleBase::initializeServices();
  }

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
  std::mutex* stateMutex_;
  double dt_;

};
}
