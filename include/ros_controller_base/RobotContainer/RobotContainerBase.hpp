#pragma once


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>

#include <ctime>
#include <Eigen/Dense>
namespace robot {
class RobotContainerBase
{
 public:
  RobotContainerBase()
      : dt_(0.0),
      isSimulation_(true)
  {
    ns_ = ros::this_node::getNamespace();
    ns_.erase(0, 1);
  }
  ;

  virtual ~RobotContainerBase()
  {
  }
  ;

  virtual void create()
  {
  }
  ;

  virtual void initilize(ros::NodeHandle* nodeHandle)
  {
    nodeHandle_ = nodeHandle;
  }
  ;

  virtual void readParameters()
  {
    nodeHandle_->getParam(ns_ + "/controller/simulation", isSimulation_);
    if(!isSimulation_){
      std::cerr<<"controller is running on real robot"<<std::endl;
    }
  }
  ;

  virtual void initilizePublishers()
  {
  }
  ;
  virtual void initilizeSubscribers()
  {
  }
  ;
  virtual void initilizeServices()
  {
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

  std::string ns_;
  ros::NodeHandle* nodeHandle_;

  bool isSimulation_;

  std::mutex mutex_;
  double dt_;

};
}
