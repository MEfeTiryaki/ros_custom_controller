/*
 File name: ControllerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 19.06.2018
 */
#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>


namespace controller {
template<typename Robot>
class ControllerBase
{
 public:
  ControllerBase()
      : isSimulation_(true),
        dt_(0.0),
        robot_(),
        nodeHandle_(),
        controllerRate_(0)
  {
    ns_ = ros::this_node::getNamespace();
    ns_.erase(0, 1);
  }
  ;
  virtual ~ControllerBase()
  {
  }
  ;

  virtual void create(Robot* r)
  {
    robot_ = r;
  }
  ;

  virtual void initilize(ros::NodeHandle* nodeHandle)
  {
    nodeHandle_ = nodeHandle;
  }
  ;
  virtual void initilizeSubscribers()
  {
  }
  ;

  virtual void initilizePublishers()
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

  virtual void readParameters()
  {
    nodeHandle_->getParam(ns_ + "/controller/simulation", isSimulation_);
    nodeHandle_->getParam(ns_ + "/controller/rate", controllerRate_);
    dt_ = 1.0 / controllerRate_;
  }
  ;
  virtual void publish()
  {
    robot_->publish();
  }
  ;

 protected:

  ros::NodeHandle* nodeHandle_;
  std::string ns_;
  std::mutex mutex_;
  double dt_;
  double controllerRate_;

  bool isSimulation_;

  std::vector<ros::Publisher> publishers_;
  std::vector<ros::Subscriber> subscribers_;

  Robot* robot_;
};
}  // namespace estimator
