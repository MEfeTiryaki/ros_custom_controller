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
#include "ros_custom_hardware_adapter/HardwareAdapterFrameBase.hpp"

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
        stateEstimator_(),
        hardwareAdapterFrame_(),
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
    this->nodeHandle_->getParam("/simulation", isSimulation_);
    this->nodeHandle_->getParam(this->namespace_ + "/controller/rate", controllerRate_);
    dt_ = 1.0 / controllerRate_;
    this->rate_ = new ros::Rate(controllerRate_);

    std::string data;
    this->nodeHandle_->getParam(this->namespace_ + "/controller/services/stop/topic", data);
    this->serviceNames_.push_back(data);

    hardwareAdapterFrame_->readParameters();

    robot_->readParameters();

    stateEstimator_->readParameters();
  }

  virtual void initializePublishers() override
  {
    RosNodeModuleBase::initializePublishers();
    hardwareAdapterFrame_->initializePublishers();
    robot_->initializePublishers();
    stateEstimator_->initializePublishers();

  }

  virtual void initializeSubscribers() override
  {
    RosNodeModuleBase::initializeSubscribers();
    hardwareAdapterFrame_->initializeSubscribers();
    robot_->initializeSubscribers();
    stateEstimator_->initializeSubscribers();
  }

  virtual void initializeServices() override
  {
    RosNodeModuleBase::initializeServices();
    // Controller Stop Service

    stopServices_ = this->nodeHandle_->advertiseService(
        this->nodeName_ + "/" + this->serviceNames_[0],
        &ControllerFrameBase::controllerStopServiceCallback, this);

    hardwareAdapterFrame_->initializeServices();
    robot_->initializeServices();
    stateEstimator_->initializeServices();
  }

  virtual void initialize() override
  {
    RosNodeModuleBase::initialize();
    hardwareAdapterFrame_->initialize();
    robot_->initialize();
    stateEstimator_->initialize();
  }

  virtual void shutdown() override
  {
    std::lock_guard<std::mutex> lock(*shutdownMutex_);
    RosNodeModuleBase::shutdown();
    stop();
    stopServices_.shutdown();
    hardwareAdapterFrame_->shutdown();
    robot_->shutdown();
    stateEstimator_->shutdown();

    int i = 0;
    while (!(hardwareAdapterFrame_->isTerminated() && stateEstimator_->isTerminated())) {
    }

    hardwareAdapterFrame_->clean();
    stateEstimator_->clean();

    hardwareAdapterFrame_.reset();
    stateEstimator_.reset();

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
  bool controllerStopServiceCallback(std_srvs::SetBool::Request& request,
                                     std_srvs::SetBool::Response& response)
  {
    run_ = !request.data;
    stateEstimator_->setRun(run_);
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
  std::unique_ptr<estimator::EstimatorBase> stateEstimator_;
  std::unique_ptr<hardware_adapter::HardwareAdapterFrameBase> hardwareAdapterFrame_;
};
}  // namespace estimator
