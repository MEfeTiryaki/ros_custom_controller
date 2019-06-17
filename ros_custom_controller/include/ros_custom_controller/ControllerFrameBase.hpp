/*
 File name: ControllerFrameBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
#pragma once

#include "ros_node_utils/RosExecuterNodeBase.hpp"
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
class ControllerFrameBase : public ros_node_utils::RosExecuterNodeBase
{
 public:
  ControllerFrameBase(std::string nodeName)
      : ros_node_utils::RosExecuterNodeBase(nodeName),
        controllerRate_(100),
        isSimulation_(true),
        run_(false),
        dt_(0.0),
        robot_(),
        stateEstimator_(),
        hardwareAdapterFrame_()
  {
  }

  virtual ~ControllerFrameBase()
  {

  }

  virtual void create() override
  {
    RosExecuterNodeBase::create();
    controllerRate_ = 100 ;
    isSimulation_ = true ;
    run_ = false;
    dt_ = 0.0;

    robot_ = new Robot(this->nodeHandle_);
    robot_->create();
   //CONFIRM("create : [Controller_Frame_Base]");
  }

  virtual void readParameters() override
  {
    RosExecuterNodeBase::readParameters();
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

   //CONFIRM("readParameters : [Controller_Frame_Base]");
  }

  virtual void initializePublishers() override
  {
    RosExecuterNodeBase::initializePublishers();
    hardwareAdapterFrame_->initializePublishers();
    robot_->initializePublishers();
    stateEstimator_->initializePublishers();

  }


  virtual void initializeSubscribers() override
  {
    RosExecuterNodeBase::initializeSubscribers();
    hardwareAdapterFrame_->initializeSubscribers();
    robot_->initializeSubscribers();
    stateEstimator_->initializeSubscribers();
  }

  virtual void initializeServices() override
  {
    RosExecuterNodeBase::initializeServices();
    // Controller Stop Service
    nodeRestartServices_ = this->nodeHandle_->advertiseService(
        "/controller_restart",&ControllerFrameBase::nodeRestartCallback, this);

    stopServices_ = this->nodeHandle_->advertiseService(
        this->nodeName_ + "/" + this->serviceNames_[0],
        &ControllerFrameBase::controllerStopServiceCallback, this);

    hardwareAdapterFrame_->initializeServices();
    robot_->initializeServices();
    stateEstimator_->initializeServices();
  }

  virtual void initialize() override
  {
    RosExecuterNodeBase::initialize();
    hardwareAdapterFrame_->initialize();
    robot_->initialize();
    stateEstimator_->initialize();
   //CONFIRM("initialize : [Controller_Frame_Base]");
  }

  virtual void shutdown() override
  {
    RosExecuterNodeBase::shutdown();
    disconnect();
    nodeRestartServices_.shutdown();
    stopServices_.shutdown();
    hardwareAdapterFrame_->shutdown();
    robot_->shutdown();
    stateEstimator_->shutdown();
    delete hardwareAdapterFrame_ ;
    delete robot_;
    delete stateEstimator_;
   //ERROR("shutdown : [Controller_Frame_Base]");
  }

  virtual void advance(double dt)
  {
    //CONFIRM(boost::lexical_cast<std::string>(controllerThread_->get_id()) + "[Controller_Frame_base] ");
    dt_ = dt;
    robot_->advance(dt_);
  }


  virtual void execute()
  {
    while (ros::ok()) {
      if (run_) {
        advance(dt_);
      }
      ros::spinOnce();
      this->rate_->sleep();
      //CONFIRM("[Controller] : " + std::to_string(ros::Time::now().toNSec()/1000000.0));
    }
  }

  virtual void run()
  {
    ros::Rate mainRate = ros::Rate(100);
    while (ros::ok()) {
      ros::spinOnce();
      mainRate.sleep();
    }
  }



  virtual void connect()
  {
    controllerThread_ = new boost::thread(boost::bind(&ControllerFrameBase::execute, this));
   //WARNING("connect : [Controller_Frame_Base]");
  }

  virtual void disconnect()
  {
    controllerThread_->detach();
   //WARNING("disconnect : [Controller_Frame_Base]");
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

  bool nodeRestartCallback(std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response)
  {
    run_ = false ;
    restartThread_ = new boost::thread(boost::bind(&ControllerFrameBase::restart, this));
    return true;
  }

  void restart()
  {
    sleep(0.5);
    shutdown();
    create();
    readParameters();
    initialize();
    connect();
  }

 protected:

  boost::thread* controllerThread_;
  boost::thread* restartThread_;

  std::mutex mutex_;
  double controllerRate_;
  double dt_;

  bool isSimulation_;
  std::string stopServiceName_;
  ros::ServiceServer stopServices_;
  ros::ServiceServer nodeRestartServices_;
  // Flag for running
  bool run_;

  Robot* robot_;
  estimator::EstimatorBase* stateEstimator_;
  hardware_adapter::HardwareAdapterFrameBase* hardwareAdapterFrame_;
};
}  // namespace estimator
