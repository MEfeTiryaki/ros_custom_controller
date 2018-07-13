#include "ros/ros.h"
#include <iostream>
#include "ros_custom_controller_base/ModelPredictiveController/DeltaInputFormulationBase.hpp"
#include "ros_custom_controller_base/ModelPredictiveController/TrajectoryTrackingBase.hpp"
#include "ros_custom_controller_base/RobotContainer/RobotModuleContainerBase.hpp"
#include "ros_custom_controller_base/State.h"

void testTrajectoryTracking(ros::NodeHandle* nh)
{
  robot::RobotModuleContainerBase<ros_custom_controller_base::State>* robot_ =
      new robot::RobotModuleContainerBase<ros_custom_controller_base::State>();
  robot_->setN(1);
  robot_->setM(1);
  controller::TrajectoryTrackingBase<
      robot::RobotModuleContainerBase<ros_custom_controller_base::State>>* controller_ =
      new controller::TrajectoryTrackingBase<
          robot::RobotModuleContainerBase<ros_custom_controller_base::State>>();
  controller_->create(robot_);

  Eigen::MatrixXd P = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd R = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd A = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd B = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Ax = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bx = Eigen::VectorXd(2);
  Eigen::MatrixXd Au = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bu = Eigen::VectorXd(2);
  Eigen::MatrixXd Af = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bf = Eigen::VectorXd(2);
  P << 10;
  Q << 100;
  R << 1000;
  A << 2;
  B << 3;
  Ax << 5, 5;
  bx << 7, 7;
  Au << 11, 11;
  bu << 13, 13;
  Af << 17, 17;
  bf << 19, 19;
  std::cerr << "A\n" << A << std::endl;
  std::cerr << "B\n" << B << std::endl;
  std::cerr << "Ax\n" << Ax << std::endl;
  std::cerr << "bx\n" << bx << std::endl;
  std::cerr << "Au\n" << Au << std::endl;
  std::cerr << "bu\n" << bu << std::endl;
  std::cerr << "Af\n" << Af << std::endl;
  std::cerr << "bf\n" << bf << std::endl;
  controller_->setHorizonLength(5);
  controller_->setCostMatrices(P, Q, R);

  controller_->setA(A);
  controller_->setB(B);
  controller_->setAx(Ax);
  controller_->setBx(bx);
  controller_->setAu(Au);
  controller_->setBu(bu);
  controller_->setAf(Af);
  controller_->setBf(bf);

  controller_->test();

  std::cerr << "H\n" << controller_->getH() << std::endl;
  std::cerr << "G\n" << controller_->getG() << std::endl;
  std::cerr << "E\n" << controller_->getE() << std::endl;
  std::cerr << "E_s\n" << controller_->getEs() << std::endl;
  std::cerr << "W\n" << controller_->getW().transpose() << std::endl;
  std::cerr << "F\n" << controller_->getF() << std::endl;
  std::cerr << "A_bar\n" << controller_->getAbar() << std::endl;
}

void testDeltaFormulation(ros::NodeHandle* nh)
{
  robot::RobotModuleContainerBase<ros_custom_controller_base::State>* robot_ =
      new robot::RobotModuleContainerBase<ros_custom_controller_base::State>();
  robot_->setN(1);
  robot_->setM(1);
  controller::DeltaInputFormulationBase<
      robot::RobotModuleContainerBase<ros_custom_controller_base::State>>* controller_ =
      new controller::DeltaInputFormulationBase<
          robot::RobotModuleContainerBase<ros_custom_controller_base::State>>();
  controller_->create(robot_);

  Eigen::MatrixXd P = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd R = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd A = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd B = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Ax = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd bx = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Au = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd bu = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Af = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd bf = Eigen::MatrixXd(1, 1);
  P << 10;
  Q << 100;
  R << 1000;
  A << 2;
  B << 3;
  Ax << 5;
  bx << 7;
  Au << 11;
  bu << 13;
  Af << 17;
  bf << 19;
  std::cerr << "A\n" << A << std::endl;
  std::cerr << "B\n" << B << std::endl;
  std::cerr << "Ax\n" << Ax << std::endl;
  std::cerr << "bx\n" << bx << std::endl;
  std::cerr << "Au\n" << Au << std::endl;
  std::cerr << "bu\n" << bu << std::endl;
  std::cerr << "Af\n" << Af << std::endl;
  std::cerr << "bf\n" << bf << std::endl;
  controller_->setHorizonLength(5);
  controller_->setCostMatrices(P, Q, R);

  controller_->setA(A);
  controller_->setB(B);
  controller_->setAx(Ax);
  controller_->setBx(bx);
  controller_->setAu(Au);
  controller_->setBu(bu);
  controller_->setAf(Af);
  controller_->setBf(bf);

  controller_->test();

  std::cerr << "H\n" << controller_->getH() << std::endl;
  std::cerr << "G\n" << controller_->getG() << std::endl;
  std::cerr << "E\n" << controller_->getE() << std::endl;
  std::cerr << "W\n" << controller_->getW().transpose() << std::endl;
  std::cerr << "F\n" << controller_->getF() << std::endl;
  std::cerr << "Fu\n" << controller_->getFu() << std::endl;
}

void testMPC(ros::NodeHandle* nh)
{
  robot::RobotModuleContainerBase<ros_custom_controller_base::State>* robot_ =
      new robot::RobotModuleContainerBase<ros_custom_controller_base::State>();
  robot_->setN(1);
  robot_->setM(1);
  controller::ModelPredictiveControllerBase<
      robot::RobotModuleContainerBase<ros_custom_controller_base::State>>* controller_ =
      new controller::ModelPredictiveControllerBase<
          robot::RobotModuleContainerBase<ros_custom_controller_base::State>>();
  controller_->create(robot_);

  Eigen::MatrixXd P = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Q = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd R = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd A = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd B = Eigen::MatrixXd(1, 1);
  Eigen::MatrixXd Ax = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bx = Eigen::VectorXd(2);
  Eigen::MatrixXd Au = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bu = Eigen::VectorXd(2);
  Eigen::MatrixXd Af = Eigen::MatrixXd(2, 1);
  Eigen::VectorXd bf = Eigen::VectorXd(2);
  P << 10;
  Q << 100;
  R << 1000;
  A << 2;
  B << 3;
  Ax << 5, 5;
  bx << 7, 7;
  Au << 11, 11;
  bu << 13, 13;
  Af << 17, 17;
  bf << 19, 19;
  std::cerr << "A\n" << A << std::endl;
  std::cerr << "B\n" << B << std::endl;
  std::cerr << "Ax\n" << Ax << std::endl;
  std::cerr << "bx\n" << bx << std::endl;
  std::cerr << "Au\n" << Au << std::endl;
  std::cerr << "bu\n" << bu << std::endl;
  std::cerr << "Af\n" << Af << std::endl;
  std::cerr << "bf\n" << bf << std::endl;
  controller_->setHorizonLength(5);
  controller_->setCostMatrices(P, Q, R);

  controller_->setA(A);
  controller_->setB(B);
  controller_->setAx(Ax);
  controller_->setBx(bx);
  controller_->setAu(Au);
  controller_->setBu(bu);
  controller_->setAf(Af);
  controller_->setBf(bf);

  controller_->test();

  std::cerr << "H\n" << controller_->getH() << std::endl;
  std::cerr << "G\n" << controller_->getG() << std::endl;
  std::cerr << "E\n" << controller_->getE() << std::endl;
  std::cerr << "W\n" << controller_->getW().transpose() << std::endl;
  std::cerr << "F\n" << controller_->getF() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle* nh = new ros::NodeHandle("~");
  //testMPC(nh);

  testTrajectoryTracking(nh);
  return 0;
}
