/*
 File name: ControllerFrameBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
#pragma once

#include "ros_node_utils/RosNodeModuleBase.hpp"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include <std_srvs/SetBool.h>
#include "robot_container/RobotContainerBase.hpp"
#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_hardware_adapter/HardwareBase.hpp"

namespace controller {
template<typename Robot>
class ControllerFrameBase : public ros_node_utils::RosNodeModuleBase
{
 public:
  ControllerFrameBase(ros::NodeHandle* nodeHandle)
      : ros_node_utils::RosNodeModuleBase(nodeHandle),
        controllerRate_(100),
        isSimulation_(true),
        run_(false),
        dt_(0.0),
        robot_(),
        controllerThread_()
  {
  }

  virtual ~ControllerFrameBase()
  {

  }

  virtual void create() override
  {
    RosNodeModuleBase::create();
    controllerRate_ = 100;
    isSimulation_ = true;
    run_ = false;
    dt_ = 0.0;

    robot_.reset(new Robot(this->nodeHandle_));
    robot_->create();
  }

  virtual void readParameters() override
  {
    RosNodeModuleBase::readParameters();
    paramRead(getNodeHandle(), "/simulation", isSimulation_);
    paramRead(getNodeHandle(), "/" + this->namespace_ + "/controller/rate", controllerRate_);
    dt_ = 1.0 / controllerRate_;
    this->rate_ = new ros::Rate(controllerRate_);


    robot_->readParameters();

  }

  virtual void initializePublishers() override
  {
    RosNodeModuleBase::initializePublishers();
    robot_->initializePublishers();

  }

  virtual void initializeSubscribers() override
  {
    RosNodeModuleBase::initializeSubscribers();
    robot_->initializeSubscribers();
  }

  virtual void initializeServices() override
  {
    RosNodeModuleBase::initializeServices();
    // Controller Stop Service

    stopServices_ = this->nodeHandle_->advertiseService(
        "/" + this->namespace_ + "/controller/stop",
        &ControllerFrameBase::controllerStopServiceCallback, this);

    robot_->initializeServices();
  }

  virtual void initialize() override
  {
    RosNodeModuleBase::initialize();
    robot_->initialize();
  }

  virtual void shutdown() override
  {
    std::lock_guard<std::mutex> lock(*shutdownMutex_);
    RosNodeModuleBase::shutdown();
    stop();
    stopServices_.shutdown();
    robot_->shutdown();

    // int i = 0;
    // while (!(hardware_->isTerminated() && stateEstimator_->isTerminated())) {
    // }



    robot_.reset();
  }

  virtual void advance(double dt)
  {
    dt_ = dt;
    robot_->advance(dt_);
  }

  virtual void execute()
  {
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(*shutdownMutex_);
        if (!isTerminationStarted()) {
          if (run_) {
            advance(dt_);
          }
        } else {
          break;
        }
      }
      this->rate_->sleep();
    }
    terminate();
  }

  void start() override
  {
    controllerThread_ = new boost::thread(boost::bind(&ControllerFrameBase::execute, this));
  }

  virtual void stop()
  {
    controllerThread_->detach();
  }

 protected:
  virtual bool controllerStopServiceCallback(std_srvs::SetBool::Request& request,
                                     std_srvs::SetBool::Response& response)
  {
    run_ = !request.data;
    response.success = true;
    return true;
  }

 protected:

  boost::thread* controllerThread_;

  std::mutex mutex_;
  double controllerRate_;
  double dt_;

  bool isSimulation_;
  std::string stopServiceName_;
  ros::ServiceServer stopServices_;
// Flag for running
  bool run_;

  std::unique_ptr<Robot> robot_;
};
}  // namespace estimator
