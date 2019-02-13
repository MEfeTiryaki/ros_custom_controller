/*
 File name: ControllerFrameBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
#pragma once

#include "ros_node_base/RosExecuterNodeBase.hpp"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include <std_srvs/SetBool.h>
#include "ros_custom_controller/RobotContainer/RobotContainerBase.hpp"

namespace controller {

template<typename Robot>
class ControllerFrameBase : public ros_node_utils::RosExecuterNodeBase
{
 public:
  ControllerFrameBase(std::string nodeName)
      : ros_node_utils::RosExecuterNodeBase(nodeName),
        controllerRate_(100),
        isSimulation_(true),
        run_(false),
        dt_(0.0),
        robot_()
  {
  }
  ;
  virtual ~ControllerFrameBase()
  {

  }
  ;
  virtual void create() override
  {
    RosExecuterNodeBase::create();
    robot_ = new Robot(this->nodeHandle_);
    robot_->create();

  }
  ;
  virtual void readParameters() override
  {
    this->nodeHandle_->getParam(this->namespace_ + "/controller/simulation", isSimulation_);
    this->nodeHandle_->getParam(this->namespace_ + "/controller/rate", controllerRate_);
    dt_ = 1.0 / controllerRate_;
    this->rate_ = new ros::Rate(controllerRate_);

    std::string data;
    this->nodeHandle_->getParam(this->namespace_ + "/controller/services/stop/topic", data);
    this->serviceNames_.push_back(data);

    robot_->readParameters();
  }
  ;
  virtual void initialize() override
  {
    robot_->initialize();
    RosExecuterNodeBase::initialize();
  }
  ;

  virtual void advance(double dt)
  {
    dt_ = dt;
    robot_->advance(dt_);
  }
  ;

  virtual void execute()
  {
    while (ros::ok()) {
      if (run_) {
        advance(dt_);
      }
      ros::spinOnce();
      this->rate_->sleep();
    }
  }
  ;

  virtual void initializePublishers()
  {
    robot_->initializePublishers();
  }
  ;

  virtual void initializeSubscribers()
  {
    robot_->initializeSubscribers();
  }
  ;

  virtual void initializeServices()
  {
    // Controller Stop Service
    this->services_[this->serviceNames_[0]] = this->nodeHandle_->advertiseService(
         this->nodeName_ + "/" + this->serviceNames_[0],
        &ControllerFrameBase::controllerStopServiceCallback, this);
    robot_->initializeServices();
  }
  ;
  bool controllerStopServiceCallback(std_srvs::SetBool::Request& request,
                                     std_srvs::SetBool::Response& response)
  {
    run_ = !request.data;
    response.success = true;
    return true;
  }
  ;

 protected:
  std::mutex mutex_;
  double controllerRate_;
  double dt_;

  bool isSimulation_;
  std::string stopServiceName_;
  ros::ServiceServer stopServices_;

  // Flag for running
  bool run_;

  Robot* robot_;
};
}  // namespace estimator
