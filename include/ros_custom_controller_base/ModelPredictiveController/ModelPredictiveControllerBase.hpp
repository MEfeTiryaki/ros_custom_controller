#pragma once

#include "ros_custom_controller_base/ControllerBase.hpp"

#include <ctime>

#include <Eigen/Core>
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"

namespace controller {

template<typename Robot>
class ModelPredictiveControllerBase : public ControllerBase<Robot>
{
 public:
  ModelPredictiveControllerBase()
      : ControllerBase<Robot>(),
        horizonLength_(2),
        time_stop_(0.0),
        time_stop_ros_(0.0)
  {
    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  }
  ;

  virtual ~ModelPredictiveControllerBase()
  {
  }
  ;

  virtual void create(Robot* r) override
  {
    ControllerBase<Robot>::create(r);
    horizonLength_ = 0;

    // State Vectors
    A_ = Eigen::MatrixXd::Zero(this->robot_->getN(), this->robot_->getN());
    B_ = Eigen::MatrixXd::Zero(this->robot_->getN(), this->robot_->getM());

    // state constrains matrix and vector (A_*x<=b)
    A_x_ = Eigen::MatrixXd();
    b_x_ = Eigen::VectorXd();
    // input constrains matrix and vector (A_*u<=b)
    A_u_ = Eigen::MatrixXd();
    b_u_ = Eigen::VectorXd();
    // final state constrains matrix and vector (A_*u<=b)
    A_f_ = Eigen::MatrixXd();
    b_f_ = Eigen::VectorXd();
    //

  }
  ;

  virtual void initilize(ros::NodeHandle* nodeHandle) override
  {
    ControllerBase<Robot>::initilize(nodeHandle);
  }

  virtual void readParameters() override
  {
    ControllerBase<Robot>::readParameters();
    int n = this->robot_->getN();
    int m = this->robot_->getM();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    double* ptr;
    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/N")) {
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/N", horizonLength_);
    } else {
      ROS_INFO("The MPC horizon length couldn't be found");
    }
    this->robot_->setTrajectoryLength(horizonLength_);
    int N = horizonLength_;

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/P")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/P", value);
      ptr = &value[0];
      P.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC P Matrix couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/Q")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/Q", value);
      ptr = &value[0];
      Q.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC Q Matrix couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/R")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/R", value);
      ptr = &value[0];
      R.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC Q Matrix couldn't be found");
    }

    //
    setCostMatrices(P, Q, R);

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/b_x")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/b_x", value);
      ptr = &value[0];
      b_x_ = Eigen::VectorXd(value.size());
      b_x_ << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC b_x couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/A_x")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/A_x", value);
      ptr = &value[0];
      A_x_ = Eigen::MatrixXd(value.size() / n, n);
      A_x_ << Eigen::Map<Eigen::MatrixXd>(ptr, value.size() / n, n);
    } else {
      ROS_INFO("The MPC A_x couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/b_f")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/b_f", value);
      ptr = &value[0];
      b_f_ = Eigen::VectorXd(value.size());
      b_f_ << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC b_f couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/A_f")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/A_f", value);
      ptr = &value[0];
      A_f_ = Eigen::MatrixXd(value.size() / n, n);
      A_f_ << Eigen::Map<Eigen::MatrixXd>(ptr, value.size() / n, n);
    } else {
      ROS_INFO("The MPC A_f couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/b_u")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/b_u", value);
      ptr = &value[0];
      b_u_ = Eigen::VectorXd(value.size());
      b_u_ << Eigen::Map<Eigen::VectorXd>(ptr, value.size());
    } else {
      ROS_INFO("The MPC b_u couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/controller/MPC/A_u")) {
      std::vector<double> value;
      this->nodeHandle_->getParam(this->ns_ + "/controller/MPC/A_u", value);
      ptr = &value[0];
      A_u_ = Eigen::MatrixXd(value.size() / m, n);
      A_u_ << Eigen::Map<Eigen::MatrixXd>(ptr, value.size() / m, m);
    } else {
      ROS_INFO("The MPC A_u couldn't be found");
    }

    //std::cerr << b_x_.transpose() << std::endl;
    //std::cerr << A_x_ << std::endl;
    //std::cerr << b_f_.transpose() << std::endl;
    //std::cerr << A_f_ << std::endl;
    //std::cerr << b_u_.transpose() << std::endl;
    //std::cerr << A_u_ << std::endl;

  }
  ;

  virtual void advance(double dt) override
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->dt_ = dt;
    calculateCostMatrixes();
    solveQuadraticOptimization();
    setCommand();
    //time_stop_ = clock();
    //time_stop_ros_ = ros::Time::now().toSec();
    //std::cout << "dt : " << this->dt_ << " sec" << std::endl;
    //std::cout << "Time: " << (time_stop_ - time_start_) / double(CLOCKS_PER_SEC) << " sec "
    //          << std::endl;
    // std::cout << "Rime: " << time_stop_ros_ - time_start_ros_ << " sec" << std::endl;
    //std::cout << "____________\n";
    //time_start_ = clock();
    //time_start_ros_ = ros::Time::now().toSec();
  }

  virtual void setCostMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R)
    {
      int n = P.cols();
      int m = R.cols();
      Q_ = Eigen::MatrixXd::Identity(n * (horizonLength_ + 1), n * (horizonLength_ + 1));

      R_ = Eigen::MatrixXd::Identity(m * horizonLength_, m * horizonLength_);

      for (int i = 0; i < horizonLength_; i++) {
        Q_.block(i * n, i * n, n, n) = Q;
        R_.block(i * m, i * m, m, m) = R;
      }
      Q_.block(horizonLength_ * n, horizonLength_ * n, n, n) = P;
    }
 protected:

  virtual void initilizeCostMatrixes()
  {
    int n_x = b_x_.size();
    int n_u = b_u_.size();
    int n_f = b_f_.size();
    int N = horizonLength_;
    int n = this->robot_->getN();
    int m = this->robot_->getM();

    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(n, n);

    S_x_ = Eigen::MatrixXd::Zero(n * (N + 1), n);
    S_u_ = Eigen::MatrixXd::Zero(n * (N + 1), m * N);
    Eigen::MatrixXd G_upper = Eigen::MatrixXd::Zero(n_u * N, m * N);
    Eigen::MatrixXd E_upper = Eigen::MatrixXd::Zero(n_u * N, n);
    Eigen::VectorXd w_upper = Eigen::VectorXd::Zero(n_u * N);
    Eigen::MatrixXd G_lower = Eigen::MatrixXd::Zero(n_x * N + n_f, m * N);
    Eigen::MatrixXd E_lower = Eigen::MatrixXd::Zero(n_x * N + n_f, n);
    Eigen::VectorXd w_lower = Eigen::VectorXd::Zero(n_x * N + n_f);

    // S_X and S_u_
    S_x_.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    for (int i = 1; i < N + 1; i++) {
      // Matrix power
      temp = A_;
      for (int j = 1; j < i; j++) {
        temp = A_ * temp;
      }
      S_x_.block(i * n, 0, n, n) = temp;
    }

    for (int i = 1; i < N + 1; i++) {
      for (int j = 0; j < i-1; j++) {
        // Matrix power
        temp = A_;
        for (int k = 1; k < i - j - 1; k++) {
          temp = A_ * temp;
        }
        S_u_.block(i * n, j * m, n, m) = temp * B_;
      }

      S_u_.block(i * n, i - 1 * m, n, m) = this->B_;
    }

    // UPPER PART OF Constrains
    for (int i = 0; i < N; i++) {
      G_upper.block(i * n_u, i * m, n_u, m) = A_u_;
      w_upper.segment(i * n_u, n_u) = b_u_;
    }
    // LOWER PART OF CONSTRAINS
    w_lower.segment(0, n_x) = b_x_;
    E_lower.block(0, 0, n_x, n) = -A_x_;
    for (int i = 1; i < N; i++) {
      for (int j = 0; j < i; j++) {
        G_lower.block(i * n_x, j * m, n_x, m) = A_x_ * S_u_.block(i * n, j * m, n, m);
      }
      E_lower.block(i * n_x, 0, n_x, n) = -A_x_ * S_x_.block(i * n, 0, n, n);
      w_lower.segment(i * n_x, n_x) = b_x_;
    }
    for (int j = 0; j < N; j++) {
      G_lower.block(N * n_x, j * m, n_f, m) = A_f_ * S_u_.block(N * n, j * m, n, m);
    }
    E_lower.block(N * n_x, 0, n_f, E_lower.cols()) = -A_f_ * S_x_.block(N * n, 0, n, S_x_.cols());
    w_lower.segment(N * n_x, n_x) = b_f_;

    Eigen::MatrixXd H = S_u_.transpose() * Q_ * S_u_ + R_;
    // Concatenate
    G_.resize(G_upper.rows() + G_lower.rows(), G_upper.cols());
    H_.resize(H.rows(), H.cols());
    G_.topRows(G_upper.rows()) = G_upper.sparseView();
    G_.bottomRows(G_lower.rows()) = G_lower.sparseView();

    E_ = Eigen::MatrixXd::Zero(n_u * N + n_x * N + n_f, n);
    W_ = Eigen::VectorXd::Zero(w_upper.size() + w_lower.size());

    E_.block(0, 0, E_upper.rows(), E_upper.cols()) = E_upper;
    E_.block(E_upper.rows(), 0, E_lower.rows(), E_lower.cols()) = E_lower;

    //W_.segment(0, w_upper.size()) << w_upper;
    //W_.segment(w_upper.size(), w_lower.size()) << w_lower;
    W_ << w_upper, w_lower;
    H_ = H.sparseView();
    F_ = S_x_.transpose() * Q_ * S_u_;

    //*
    //std::cerr << "Sx\n" << S_x_ << std::endl;
    //std::cerr << "SU\n" << S_u_ << std::endl;

    //std::cerr << "G_\n" << G_ << std::endl;
    // std::cerr << "H_\n" << H_ << std::endl;
    // std::cerr << "E_\n" << E_ << std::endl;
    //std::cerr << "W_\n" << W_ << std::endl;
    //std::cerr << "F_\n" << F_ << std::endl;
    //*/
  }

  ;

  virtual void calculateCostMatrixes()
  {
    Eigen::VectorXd delta_x = this->robot_->getState() - this->robot_->getDesiredState();
    if (delta_x[2] > M_PI)
      delta_x[2] -= 2 * M_PI;
    else if (delta_x[2] < -M_PI)
      delta_x[2] += 2 * M_PI;
    q_ = delta_x.transpose() * F_;
    b_ = (W_ + E_ * delta_x);
  }
  ;

  virtual void solveQuadraticOptimization()
  {
    //time_start_ = clock();
    ooqpei::OoqpEigenInterface::solve(H_, q_, G_, b_, solution_, true);
    //std::cout << "Time: " << (clock()-time_start_)/double(CLOCKS_PER_SEC) << " sec "<<std::endl;
  }
  ;

  virtual void setCommand()
  {
    this->robot_->setInput(solution_.segment(0, this->robot_->getM()));
  }
  ;
 public:
  Eigen::MatrixXd getH()
  {
    return this->H_;
  }
  Eigen::MatrixXd getG()
  {
    return this->G_;
  }
  Eigen::MatrixXd getE()
  {
    return this->E_;
  }
  Eigen::VectorXd getW()
  {
    return this->W_;
  }
  Eigen::MatrixXd getF()
  {
    return this->F_;
  }

  void setHorizonLength(double N)
  {
    this->horizonLength_ = N;
  }
  void setA(Eigen::MatrixXd A)
  {
    this->A_ = A;
  }
  void setB(Eigen::MatrixXd B)
  {
    this->B_ = B;
  }
  void setAx(Eigen::MatrixXd A)
  {
    this->A_x_ = A;
  }
  void setBx(Eigen::MatrixXd B)
  {
    this->b_x_ = B;
  }
  void setAu(Eigen::MatrixXd A)
  {
    this->A_u_ = A;
  }
  void setBu(Eigen::MatrixXd B)
  {
    this->b_u_ = B;
  }
  void setAf(Eigen::MatrixXd A)
  {
    this->A_f_ = A;
  }
  void setBf(Eigen::MatrixXd B)
  {
    this->b_f_ = B;
  }
  void test()
  {
    initilizeCostMatrixes();
  }

 protected:

  int horizonLength_;

// Liner Dynamics
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;

// Constrains
  Eigen::MatrixXd A_x_;
  Eigen::VectorXd b_x_;
  Eigen::MatrixXd A_u_;
  Eigen::VectorXd b_u_;
  Eigen::MatrixXd A_f_;
  Eigen::VectorXd b_f_;

// MPC Weigth matrices
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

// Constrain Matrices
  Eigen::SparseMatrix<double, Eigen::RowMajor> G_;
//Eigen::MatrixXd G_;
  Eigen::MatrixXd E_;
  Eigen::MatrixXd W_;

  Eigen::SparseMatrix<double, Eigen::RowMajor> H_;
//Eigen::MatrixXd H_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd S_x_;
  Eigen::MatrixXd S_u_;

// optimization variables
  Eigen::VectorXd q_;
  Eigen::VectorXd b_;

  Eigen::VectorXd solution_;

  double time_start_;
  double time_stop_;
  double time_start_ros_;
  double time_stop_ros_;

}
;
}
