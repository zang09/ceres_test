#include <ros/ros.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <vector>
#include <Eigen/Dense>
#include "math.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class Pose2dErrorTerm{
public:
    Pose2dErrorTerm(double initial_a[], double initial_b[]) :
        a{initial_a[0], initial_a[1], initial_a[2]},
        b{initial_b[0], initial_b[1], initial_b[2]} {}

    template <typename T> bool operator()(const T* th, const T* tr, T* residual) const {
        T r[] = {T(0.0), T(0.0), *th};

        T tp1[] = {T(a[0]), T(a[1]), T(a[2])};
        T tp2[] = {T(b[0]), T(b[1]), T(b[2])};

        T n_tp1[3];
        T n_tp2[3];

        ceres::AngleAxisRotatePoint(r, tp1, n_tp1);
        ceres::AngleAxisRotatePoint(r, tp2, n_tp2);

        n_tp1[0] += tr[0];  n_tp1[1] += tr[1];  n_tp1[2] += tr[2];
        n_tp2[0] += tr[0];  n_tp2[1] += tr[1];  n_tp2[2] += tr[2];

        residual[0] = 4.0 - n_tp1[0];
        residual[1] = -2.0 - n_tp1[1];
        residual[2] = -1.0 - n_tp2[0];
        residual[3] = 1.0 - n_tp2[1];

        return true;
    }

private:
    double a[3];
    double b[3];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ceres_test");

    ROS_INFO("\033[1;32m--->\033[0m Node is started.");

    double * a = (double *)malloc(3*sizeof(double));
    a[0] = 5.0;
    a[1] = 3.0;
    a[2] = 0.0;
    double * b = (double *)malloc(3*sizeof(double));
    b[0] = 0.0;
    b[1] = 0.0;
    b[2] = 0.0;

    double theta = 0.0;
    double translation[3] = {0,};

    // Build the problem.
    Problem problem;

    CostFunction* cost_function = new AutoDiffCostFunction<Pose2dErrorTerm, 4, 1, 3>(new Pose2dErrorTerm(a, b));

    problem.AddResidualBlock(cost_function, NULL, &theta, translation);

    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "--> (" << a[0]<<", "<<a[1]<<", "<< a[2] << ")\n";
    std::cout << "--> (" << b[0]<<", "<<b[1]<<", "<< b[2] << ")\n\n";

    std::cout << "--> (" << a[0]*cos(theta)-a[1]*sin(theta)+translation[0] <<", " << a[0]*sin(theta)+a[1]*cos(theta)+translation[1] << ", " << a[2] << ")\n";
    std::cout << "--> (" << b[0]*cos(theta)-b[1]*sin(theta)+translation[0] <<", " << b[0]*sin(theta)+b[1]*cos(theta)+translation[1] << ", " << b[2] << ")\n";
    ros::spin();
    return 0;
}

