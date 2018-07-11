#pragma once

#include "ros_custom_controller_base/ControllerBase.hpp"

#include <ctime>

namespace controller {

template<typename Robot>
class FeedbackControllerBase : public ControllerBase<Robot>
{
 public:
  FeedbackControllerBase()
      : ControllerBase<Robot>()
  {
    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  }
  ;

  virtual ~FeedbackControllerBase()
  {
  }
  ;

  virtual void create(Robot* r) override
  {
    ControllerBase<Robot>::create(r);
    x_err_ = Eigen::VectorXd::Zero(this->robot_->getState().size());
  }
  ;

  virtual void advance(double dt) override
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->dt_ = dt;
    calculateError();
    calculateInput();
    //time_stop_ = clock();
    //time_stop_ros_ = ros::Time::now().toSec();
    //std::cout << "dt : " <<this->dt_<<" sec" << std::endl;
    //std::cout << "Time: " << (time_stop_-time_start_)/double(CLOCKS_PER_SEC) << " sec "<< std::endl;
    //std::cout << "Rime: " << time_stop_ros_-time_start_ros_<<" sec" <<std::endl;
    //std::cout << "_____________" ;

    //time_start_ = clock();
    //time_start_ros_ = ros::Time::now().toSec();

  }

 protected:

  virtual void calculateError(){};

  virtual void calculateInput(){};

  Eigen::VectorXd x_err_;

  double time_start_;
  double time_stop_;
  double time_start_ros_;
  double time_stop_ros_;

}
;
}
