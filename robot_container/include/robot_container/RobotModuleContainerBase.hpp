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

/*! \~english
 *  @class RobotModuleContainer is a sub-unit of robotic system. New modules can be used in cases where
 *  the robot has multiple states which is controlled separately, such position, shape and temperature
 *  of the robot. If these states somehow effecting each other one may pass the one robotModule to another
 *  one as a pointer to reach information.
 *  RobotModuleContainers hold the information of the robot such as robot state, control input and
 *  desired global trajectory.
 */
class RobotModuleContainerBase : public RosNodeModuleBase
{
 public:
  /*! \~english
   * @brief Constructor
   * @details
   * @param[in] nodeHandle pointer to the nodeHandle
   */
  RobotModuleContainerBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        n_(0),
        m_(0),
        trajectoryLength_(2),
        dt_(0.0),
        isSimulation_(false),
        stateMutex_(new std::mutex())
  {
  }

  /*! \~english
   * @brief Destructor
   * @details
   *
   */

  virtual ~RobotModuleContainerBase()
  {
  }

  /*! \~english
   * @brief read ros parameters
   * @details it reads if the system runs in simulation or not
   */
  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/simulation", isSimulation_);
    if (!isSimulation_) {
      WARNING("[RobotModuleContainerBase] : is running on real robot");
    }
  }

  /*! \~english
   * @brief initialize robot parameters
   * @details this method initializes following variables
   * x_ : state of the robot with zero vector of n_ length
   * x_des_ : desired state of the robot with zero vector of n_ length
   * u_ : control input of the robot with zero vector of m_ length
   * x_trajectory_ : vector of  desired states  with zero vector of n_ length
   */
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

  /*! \~english
   * @brief initialize services
   * @details initialize set desired state service
   */
  virtual void initializeServices() override
  {
    if (!setDesiredStateServiceName_.empty()) {
      // TODO Efe 24.05.2019 : setDesiredStateService should be general
      setDesiredStateService_ = this->nodeHandle_->advertiseService(
          this->namespace_ + "/" + setDesiredStateServiceName_,
          &RobotModuleContainerBase::setDesiredStateServiceCallback, this);
    }
  }

  /*! \~english
   * @brief advance robot module in each control step
   * @details updates the trajectory and the desired state from trajectory
   */
  virtual void advance(double dt)
  {
    updateTrajectory();
    x_des_ = x_trajectory_[0];
    this->dt_ = dt;
  }

  /*! \~english
   * @brief updates the trajectory
   * @details this method erases the first element of the trajectory in every
   * advance and adds the next trajectory point from trajectory buffer
   */
  void updateTrajectory()
  {
    x_trajectory_.erase(x_trajectory_.begin());
    x_trajectory_.push_back(getNextTrajectoryFromBuffer());
  }

  /*! \~english
   * @brief gets next trajectory point from trajectory buffer
   * @details This methods first checks if the trajectory buffer is empty or not. If
   * it is empty than it returns the last trajectory point in trajectory vector. If
   * the buffer is not empty, method checks the time remaining to the next trajectory
   * point in the buffer. and return the interpolation of the last desired state in trajectory
   * vector and the next trajectory point in the trajectory buffer using time increment dt_.
   */
  virtual Eigen::VectorXd getNextTrajectoryFromBuffer()
  {
    Eigen::VectorXd nextTrajectoryPoint = Eigen::VectorXd::Zero(n_);

    // Checks whether buffer is empty  or not
    if (!isBufferEmpty()) {
      TrajectoryPoint& next = trajectoryBuffer_[0];
      // Checks time left for the next trajectory buffer point
      if (next.timeLeft > 0.0) {
        // interpolation
        nextTrajectoryPoint = x_trajectory_.back()
            + (next.point - x_trajectory_.back()) * dt_ / next.timeLeft;
        // updates the time
        next.timeLeft -= dt_;
      } else {
        // if there is no time left update the buffer
        trajectoryBuffer_.erase(trajectoryBuffer_.begin());
        // Check again if buffer is empty after removing first element
        if (!isBufferEmpty()) {
          next = trajectoryBuffer_[0];
          // interpolate
          nextTrajectoryPoint = x_trajectory_.back()
              + (next.point - x_trajectory_.back()) * dt_ / next.timeLeft;
          next.timeLeft -= dt_;
        } else {
          // return the last element in the trajectory buffer
          nextTrajectoryPoint = x_trajectory_.back();
        }
      }
    } else {
      // return the last element in the trajectory buffer
      nextTrajectoryPoint = x_trajectory_.back();
    }
    return nextTrajectoryPoint;
  }

  /*! \~english
   * @brief Checks the buffer size
   * @details
   */
  bool isBufferEmpty()
  {
    return trajectoryBuffer_.size() == 0;
  }

  /*! \~english
   * @brief append new trajectory point to the buffer
   * @details
   */
  void appendToBuffer(TrajectoryPoint x)
  {
    trajectoryBuffer_.push_back(x);
  }

  /*! \~english
   * @brief append new trajectory vector to the buffer
   * @details
   */
  void appendToBuffer(std::vector<TrajectoryPoint> traj)
  {
    trajectoryBuffer_.insert(trajectoryBuffer_.end(), traj.begin(), traj.end());
  }

  /*! \~english
   * @brief clears the buffer
   * @details
   */
  void clearBuffer()
  {
    trajectoryBuffer_.clear();
  }

  // SETTER AND GETTER
 public:

  /*! \~english
   * @brief getter for state mutex
   * @details
   */
  std::mutex* getStateMutex()
  {
    return stateMutex_;
  }

  /*! \~english
   * @brief setter for state
   * @details
   */
  virtual void setState(Eigen::VectorXd x)
  {
    x_ = x;
  }

  /*! \~english
   * @brief getter for state
   * @details
   */
  Eigen::VectorXd getState()
  {
    return x_;
  }

  /*! \~english
   * @brief setter for desired state
   * @details
   */
  void setDesiredSate(Eigen::VectorXd x_des)
  {
    x_des_ = x_des;
  }

  /*! \~english
   * @brief getter for desired state
   * @details
   */
  Eigen::VectorXd getDesiredState()
  {
    return x_des_;
  }

  /*! \~english
   * @brief setter for input
   * @details
   */
  void setInput(Eigen::VectorXd u)
  {
    u_ = u;
  }

  /*! \~english
   * @brief getter for input
   * @details
   */
  Eigen::VectorXd getInput()
  {
    return u_;
  }

  /*! \~english
   * @brief setter for state length
   * @details
   */
  void setN(int n)
  {
    n_ = n;
  }

  /*! \~english
   * @brief getter for state length
   * @details
   */
  int getN()
  {
    return n_;
  }

  /*! \~english
   * @brief setter for input length
   * @details
   */
  void setM(int m)
  {
    m_ = m;
  }

  /*! \~english
   * @brief getter for input length
   * @details
   */
  int getM()
  {
    return m_;
  }

  /*! \~english
   * @brief setter for trajectory length
   * @details
   */
  void setTrajectoryLength(int length)
  {
    trajectoryLength_ = length;
    x_trajectory_ = std::vector<Eigen::VectorXd>(trajectoryLength_);
    for (int i = 0; i < trajectoryLength_; i++) {
      x_trajectory_[i] = Eigen::VectorXd::Zero(n_);
    }
  }

  /*! \~english
   * @brief setter for trajectory length
   * @details
   */
  int getTrajectoryLength()
  {
    return trajectoryLength_;
  }

  /*! \~english
   * @brief setter for trajectory
   * @details
   */
  void setTrajectory(std::vector<Eigen::VectorXd> trajectory)
  {
    x_trajectory_ = trajectory;
  }

  /*! \~english
   * @brief getter for trajectory
   * @details
   */
  std::vector<Eigen::VectorXd> getTrajectory()
  {
    return x_trajectory_;
  }

  /*! \~english
   * @brief getter for i^th trajectory trajectory point
   * @details
   */
  Eigen::VectorXd getFutureDesiredState(int i)
  {
    return x_trajectory_[i];
  }
  /*! \~english
   * @brief setter for trajectory buffer
   * @details TODO
   */
  void setTrajectoryBuffer(std::vector<TrajectoryPoint> trajectory)
  {
    trajectoryBuffer_ = trajectory;
  }

  /*! \~english
   * @brief setter for trajectory buffer
   * @details TODO
   */
  void setTrajectoryBuffer(int i, TrajectoryPoint value)
  {
    trajectoryBuffer_[i] = value;
  }

  /*! \~english
   * @brief getter for trajectory buffer
   * @details TODO
   */
  std::vector<TrajectoryPoint> getTrajectoryBuffer()
  {
    return trajectoryBuffer_;
  }

  // CALLBACKS
 protected:

  virtual bool setDesiredStateServiceCallback(robot_container::SetState::Request& request,
                                              robot_container::SetState::Response& response)
  {
    ERROR("Robot Module Container set desired state!!");
    return true;
  }

  // VARIABLES
 protected:
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



  ros::ServiceServer setDesiredStateService_;
  std::string setDesiredStateServiceName_;

};
}
