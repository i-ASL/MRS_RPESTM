#include <iostream>
#include <random>

#include <ceres/ceres.h>
#include "csvparser.h"

using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct ExponentialResidual
{
    (double x, double y)
        : x_(x), y_(y)
    {
    }

    template <typename T>
    bool operator()(const T *const m, const T *const c, T *residual) const
    {
        residual[0] = y_ - (m[0] * x_ + c[0]);
        return true;
    }

    static ceres::CostFunction* Create(
            const double x,
            const double y)
    {
        return (new ceres::AutoDiffCostFunction<ExponentialResidual,2,1,1>(
                    new ExponentialResidual(x, y)));
    }

private:
    const double x_;
    const double y_;
};

int main()
{
    /*
    const int kNumObservations = 67;

    double data[kNumObservations * 2];

    std::mt19937 gen(23497);
    std::normal_distribution<double> dist(0.0, 0.2);
    for (int i = 0; i < kNumObservations; ++i)
    {
        double x = 0.075 * static_cast<double>(i);
        double y = exp(0.3 * x + 0.1);

        data[2*i] = x;
        data[2*i + 1] = y + dist(gen);
    }

    double m(0.0), c(0.0); // y = e^(mx+c)
    */

    vector<double> m;
    vector<double> c;  
    vector<double> x;
    vector<double> y;


    CSVParser csvparser("/home/asl/rpestm_ws/src/2023-10-16-15-00-50-sync_data.csv");

    int colSize = csvparser.ncol_;

     for(int i=0;i< colSize ;i++){
      x.push_back(csvparser.data_[i][5]);
      y.push_back(csvparser.data_[i][6]);
      thetaji.push_back(csvparser.data_[i][7]);

 
  }
    
    ceres::Problem problem;
    for (int i = 0; i < colSize - 1; ++i)
    {
             problem.AddResidualBlock(
            ExponentialResidual::Create(x[i], y[i]),
          NULL,
          &m[i],
          &c[i]
            );
    }

    ceres::Solver::Options options;

    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final m : " << m << ", c : " << c << "\n";
    return 0;
}