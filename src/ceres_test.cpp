#include <ros/ros.h>

#include "ceres/ceres.h"
#include <vector>
#include <Eigen/Dense>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// 연습. 두 2D 상의 pose들 optimization해보기
class Pose2dErrorTerm{
  public:
  Pose2dErrorTerm(double initial_a[], double initial_b[]) :
  a_init(initial_a[0], initial_a[1], initial_a[2]),
  b_init(initial_b[0], initial_b[1], initial_b[2]) {}

  template <typename T> bool operator()(const T* th, const T* tr, T* residual) const {

    residual[0] = (b_init[0] - a_init[0]) + tr[0];
    residual[1] = (b_init[1] - a_init[1]) + tr[1];
    residual[2] = (b_init[2] - a_init[2]) * th[0];
    return true;
  }

private:
  const Eigen::Vector3d a_init;
  const Eigen::Vector3d b_init;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ceres_test");

  ROS_INFO("\033[1;32m--->\033[0m Node is started.");

  double * a = (double *)malloc(3*sizeof(double));
  a[0] = 3.0;
  a[1] = 1.0;
  a[2] = 0.523599; // radian
  double * b = (double *)malloc(3*sizeof(double));
  b[0] = 5.1;
  b[1] = 2.9;
  b[2] = 0.79; // radian

  double theta;
  double *translation = (double *)malloc(3*sizeof(double));

  // Build the problem.
  Problem problem;

  CostFunction* cost_function = new AutoDiffCostFunction<Pose2dErrorTerm, 3, 1, 3>(new Pose2dErrorTerm(a, b));

  problem.AddResidualBlock(cost_function, NULL, &theta, translation);

  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";

  std::cout << "--> " <<a[0]<<", "<<a[1]<<", "<< a[2] << ")\n";
  std::cout << "--> " <<b[0]<<", "<<b[1]<<", "<< b[2] << ")\n";

  ros::spin();
  return 0;
}

