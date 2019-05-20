#pragma once

#include "ros_node_utils/RosNodeModuleBase.hpp"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <ctime>
#include <Eigen/Dense>

#include "std_msgs/Float64MultiArray.h"

#include "robot_container/SetState.h"

using namespace ros_node_utils;
namespace robot {
struct TrajectoryPoint
{
  Eigen::VectorXd point;
  double timeLeft;
};

class RobotModuleContainerBase : public RosNodeModuleBase
{
 public:
  RobotModuleContainerBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        n_(0),
        m_(0),
        trajectoryLength_(2),
        commandPublisheQueueSize_(1),
        dt_(0.0),
        isSimulation_(false),
        stateMutex_(new std::mutex())
  {
  }

  virtual ~RobotModuleContainerBase()
  {
  }

  virtual void create() override
  {
    input_ = std_msgs::Float64MultiArray();
  }

  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, this->namespace_ + "/simulation", isSimulation_);
    if (!isSimulation_) {
      WARNING("[RobotModuleContainerBase] : is running on real robot");
    }
  }
  ;

  virtual void initialize()
  {
    RosNodeModuleBase::initialize();
    x_ = Eigen::VectorXd::Zero(n_);
    x_des_ = Eigen::VectorXd::Zero(n_);
    u_ = Eigen::VectorXd::Zero(m_);
    x_trajectory_ = std::vector<Eigen::VectorXd>(trajectoryLength_);
    for (int i = 0; i < trajectoryLength_; i++) {
      x_trajectory_[i] = Eigen::VectorXd::Zero(n_);
    }
  }

  virtual void initializeServices() override
  {
    if (!setDesiredStateServiceName_.empty()) {

      setDesiredStateService_ = this->nodeHandle_->advertiseService(
          this->namespace_ + "/" + setDesiredStateServiceName_,
          &RobotModuleContainerBase::setDesiredStateServiceCallback, this);
    }
  }

  virtual void advance(double dt)
  {
    updateTrajectory();
    x_des_ = x_trajectory_[0];
    this->dt_ = dt;
  }

  virtual void setState(Eigen::VectorXd x)
  {
    x_ = x;
  }

  Eigen::VectorXd getState()
  {
    return x_;
  }

  void setDesiredSate(Eigen::VectorXd x_des)
  {
    x_des_ = x_des;
  }

  Eigen::VectorXd getDesiredState()
  {
    return x_des_;
  }

  void setInput(Eigen::VectorXd u)
  {
    u_ = u;
  }

  Eigen::VectorXd getInput()
  {
    return u_;
  }

  int getN()
  {
    return n_;
  }

  void setN(int n)
  {
    n_ = n;
  }

  int getM()
  {
    return m_;
  }

  void setM(int m)
  {
    m_ = m;
  }

  void setTrajectoryLength(int length)
  {
    trajectoryLength_ = length;
    x_trajectory_ = std::vector<Eigen::VectorXd>(trajectoryLength_);
    for (int i = 0; i < trajectoryLength_; i++) {
      x_trajectory_[i] = Eigen::VectorXd::Zero(n_);
    }
  }

  int getTrajectoryLength()
  {
    return trajectoryLength_;
  }

  void setTrajectory(std::vector<Eigen::VectorXd> trajectory)
  {
    x_trajectory_ = trajectory;
  }

  std::vector<Eigen::VectorXd> getTrajectory()
  {
    return x_trajectory_;
  }

  void setBuffer(std::vector<TrajectoryPoint> trajectory)
  {
    trajectoryBuffer_ = trajectory;
  }

  void setBuffer(int i, TrajectoryPoint value)
  {
    trajectoryBuffer_[i] = value;
  }

  std::vector<TrajectoryPoint> getTrajectoryBuffer()
  {
    return trajectoryBuffer_;
  }

  void appendToBuffer(TrajectoryPoint x)
  {
    trajectoryBuffer_.push_back(x);
  }

  void appendToBuffer(std::vector<TrajectoryPoint> traj)
  {
    trajectoryBuffer_.insert(trajectoryBuffer_.end(), traj.begin(), traj.end());
  }

  void clearBuffer()
  {
    trajectoryBuffer_.clear();
  }

  virtual Eigen::VectorXd getNextTrajectoryFromBuffer()
  {
    Eigen::VectorXd nextTrajectoryPoint = Eigen::VectorXd::Zero(n_);
    ;
    if (!isBufferEmpty()) {
      TrajectoryPoint& next = trajectoryBuffer_[0];
      if (next.timeLeft > 0.0) {
        nextTrajectoryPoint = x_trajectory_.back()
            + (next.point - x_trajectory_.back()) * dt_ / next.timeLeft;
        next.timeLeft -= dt_;
      } else {
        trajectoryBuffer_.erase(trajectoryBuffer_.begin());
        if (!isBufferEmpty()) {
          next = trajectoryBuffer_[0];
          nextTrajectoryPoint = x_trajectory_.back()
              + (next.point - x_trajectory_.back()) * dt_ / next.timeLeft;
          next.timeLeft -= dt_;
        } else {
          nextTrajectoryPoint = x_trajectory_.back();
        }
      }
    } else {
      nextTrajectoryPoint = x_trajectory_.back();
    }
    return nextTrajectoryPoint;
  }

  Eigen::VectorXd getFutureDesiredState(int i)
  {
    return x_trajectory_[i];
  }

  virtual void updateCommand()
  {
  }

  bool isBufferEmpty()
  {
    return trajectoryBuffer_.size() == 0;
  }

  void updateTrajectory()
  {
    x_trajectory_.erase(x_trajectory_.begin());
    x_trajectory_.push_back(getNextTrajectoryFromBuffer());
  }

  std::mutex* getStateMutex()
  {
    return stateMutex_;
  }

  virtual Eigen::VectorXd getCommand()
  {
    return Eigen::VectorXd::Zero(3);
  }
 protected:

  virtual void StateSubscriberCallback(std_msgs::Float64MultiArray msg)
  {
  }

  virtual bool setDesiredStateServiceCallback(robot_container::SetState::Request& request,
                                              robot_container::SetState::Response& response)
  {
    std::cerr << "Robot Module Container set desired state!!" << std::endl;
    return true;
  }

  std::mutex* stateMutex_;

  double dt_;
  bool isSimulation_;

  int n_;
  int m_;
  int trajectoryLength_;

  Eigen::VectorXd x_;
  Eigen::VectorXd x_des_;
  Eigen::VectorXd u_;
  std::vector<Eigen::VectorXd> x_trajectory_;
  std::vector<TrajectoryPoint> trajectoryBuffer_;

  std_msgs::Float64MultiArray input_;

  ros::Publisher commandPublisher_;
  std::string commandPublisherName_;
  int commandPublisheQueueSize_;

  ros::Subscriber stateSubscriber_;
  std::string stateSubscriberName_;

  ros::ServiceServer setDesiredStateService_;
  std::string setDesiredStateServiceName_;

};
}
