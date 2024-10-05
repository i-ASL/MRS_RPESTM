#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "math.h"
#include "csvparser.h"
#include "matplotlibcpp.h"
#include <algorithm>

using namespace std;

namespace plt = matplotlibcpp;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::HuberLoss;
using ceres::TukeyLoss;
using ceres::ArctanLoss;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


struct MyConstraintCase3{
    MyConstraintCase3(double rho, double beta)
        :rho_(rho), beta_(beta) {}

    template<typename T>
    bool operator()(const T* const x_ji, const T* const y_ji, T* residual) const
    {
        residual[0] = (sqrt(x_ji[0]*x_ji[0] + y_ji[0]*y_ji[0]) - T(rho_)) ;
        T tmp = atan2(y_ji[0], x_ji[0]);
        if (tmp < 0)
            tmp += 2 * M_PI;
        residual[1] = (tmp - T(beta_ * M_PI / 180)) ;
        return true;
    }

    static ceres::CostFunction* Create(
            const double rho,
            const double beta)
    {
        return (new ceres::AutoDiffCostFunction<MyConstraintCase3,2,1,1>(
                    new MyConstraintCase3(rho, beta)));
    }

private:
    const double rho_;
    const double beta_;
//    const double theta_;
};


struct withoutNetworkOdomConstraint{
    withoutNetworkOdomConstraint(double delta_t)
        : delta_t_(delta_t) {}

    template<typename T>
    bool operator()(
            const T* const cx,
            const T* const cy,
            const T* const cyaw,
            const T* const vj,
            const T* const wj,
            const T* const nx,
            const T* const ny,
            const T* const nyaw,
            const T* const nvj,
            const T* const nwj,
            T* residual) const {
        residual[0] = (nx[0] - (cx[0] + delta_t_*(vj[0]*cos(cyaw[0])+cy[0]*w_i-v_i))) / 0.1;
        residual[1] = (ny[0] - (cy[0] + delta_t_*(vj[0]*sin(cyaw[0])-cx[0]*w_i))) / 0.1;
        residual[2] = (nyaw[0] - (cyaw[0] + delta_t_*(wj[0]-w_i))) / 0.01;
        residual[3] = (nvj[0] - vj[0]) / 0.01;
        residual[4] = (nwj[0] - wj[0]) / 0.01;
        return true;
    }

    static ceres::CostFunction* Create(
            const double delta_t)
    {
        return (new ceres::AutoDiffCostFunction<withoutNetworkOdomConstraint,5,1,1,1,1,1,1,1,1,1,1>(
                    new withoutNetworkOdomConstraint(delta_t)));
    }

private:
    const double delta_t_;
    const double v_i = 0.1;
    const double w_i = 0.09;
//    const double v_j = 0.4;
//    const double w_j = 0.09;
};


int main(int argc, char** argv)
{
    std::cout<<"Start similation"<<endl;
    google::InitGoogleLogging(argv[0]);

    //data read
  //  CSVParser csvparser("/home/gihun/catkin_tools_ws/files/1003/_2023-10-03-15-19-08-sync_data.csv");
    CSVParser csvparser("/home/asl/rpestm_ws/src/nlstestsc2_1.csv");

    int colSize = csvparser.ncol_ / 120;

    //parameter
    vector<double> xji;
    vector<double> yji;
    vector<double> thetaji;

    vector<double> xji_;
    vector<double> yji_;

    //ground truth
    vector<double> t_xji;
    vector<double> t_yji;
    vector<double> t_thetaji;

    //kf estimated
    // case3
    //vector<double> c3_xji;
    //vector<double> c3_yji;
    //vector<double> c3_thetaji;
    // case4
    vector<double> c4_xji;
    vector<double> c4_yji;
    vector<double> thetai;
    // utilize 'c4_' for EKF plot


    // observation
    vector<double> rho;
    vector<double> beta;
    vector<double> theta;
    // obervation with outliers
    //vector<double> o_rho;
    //vector<double> o_beta;
    //vector<double> o_theta;
    
    // input
    vector<double> delta_t;
    // for visualization
    vector<double> visualization;

    

    // for case4
  //  vector<double> vj(csvparser.ncol_, 0.3);
  //  vector<double> wj(csvparser.ncol_, 0.05);
  //  vector<double> trueVj(csvparser.ncol_, 0.2);
  //  vector<double> trueWj(csvparser.ncol_, 0.1);

    vector<double> vj(colSize+1, 0.3);
    vector<double> wj(colSize+1, 0.05);
    vector<double> trueVj(colSize, 0.4);
    vector<double> trueWj(colSize, 0.09);

    for(int i=0;i< colSize ;i++){
        xji.push_back(csvparser.data_[i][5]);
        yji.push_back(csvparser.data_[i][6]);
        thetaji.push_back(csvparser.data_[i][7]);
        
        rho.push_back(csvparser.data_[i][8]);
        beta.push_back(csvparser.data_[i][9]);
        theta.push_back(csvparser.data_[i][10]);
        
        delta_t.push_back(csvparser.data_[i][11]);
        
        // Actually not ji, but i for hardware experiment analysis
        t_xji.push_back(csvparser.data_[i][12]);
        t_yji.push_back(csvparser.data_[i][13]);
        t_thetaji.push_back(csvparser.data_[i][14]);
        
        //c3_xji.push_back(csvparser.data_[i][24]);
        //c3_yji.push_back(csvparser.data_[i][25]);
        //c3_thetaji.push_back(csvparser.data_[i][26]);
        c4_xji.push_back(csvparser.data_[i][27]);
        c4_yji.push_back(csvparser.data_[i][28]);
        thetai.push_back(csvparser.data_[i][29]);
       
        visualization.push_back(i); // for matplotlib plot
    }

    // for SF
    vector<double> slidingXji;
    vector<double> slidingYji;
    vector<double> slidingThetaji;
    vector<double> slidingVj;
    vector<double> slidingWj;

    int slidingWindow = colSize / 8;
    int numOfSliding = colSize - slidingWindow + 1; // 862

    // ====================================Optimization========================================

    // Record the starting time
    auto start_time = std::chrono::high_resolution_clock::now();

    for(int i = 0; i < numOfSliding; ++i)
    {
      ceres::Problem problem;

      xji.clear();
      yji.clear();
      thetaji.clear();

      for(int j = 0; j < colSize; j++){
        xji.push_back(csvparser.data_[j][5]);
        yji.push_back(csvparser.data_[j][6]);
        thetaji.push_back(csvparser.data_[j][7]);
      }
//      cout << xji.size() << ", " << vj.size() << endl;

      for(int k = 0 + i; k < slidingWindow + i; ++k)
      {
        // case 1~3 odometry constraint
//        problem.AddResidualBlock(
//              NetworkOdomConstraint::Create(delta_t[k]),
//              NULL,
//              &(xji[k]),
//              &(yji[k]),
//              &(thetaji[k]),
//              &(xji[k+1]),
//              &(yji[k+1]),
//              &(thetaji[k+1]));

         // case4 odometry constraint
         problem.AddResidualBlock(
               withoutNetworkOdomConstraint::Create(delta_t[k]),
               NULL,
               &(xji[k]),
               &(yji[k]),
               &(thetaji[k]),
               &(vj[k]),
               &(wj[k]),
               &(xji[k+1]),
               &(yji[k+1]),
               &(thetaji[k+1]),
               &(vj[k+1]),
               &(wj[k+1])
               );

        // measurement constraint

//             problem.AddResidualBlock(
//               MyConstraintCase1::Create(rho[k]),
//               NULL,
//               &xji[k],
//               &yji[k]
//               );

//             problem.AddResidualBlock(
//                   MyConstraintCase2::Create(beta[k]),
//                 NULL,
//                 &xji[k],
//                 &yji[k]
//                   );

        problem.AddResidualBlock(
              MyConstraintCase3::Create(rho[k], beta[k]),
              NULL,
              &xji[k],
              &yji[k]
              );

        }

        // Optimization
//        cout << "The number of residual block is : " << problem.NumResidualBlocks() << endl;
        Solver::Options options;
        options.linear_solver_type=ceres::DENSE_QR;
        options.minimizer_progress_to_stdout=true;
        Solver::Summary summary;
        Solve(options,&problem,&summary);

        if (i == 0) // first iteration
        {
          for(int k = 0; k < slidingWindow; ++k){
            slidingXji.push_back(xji[k]);
            slidingYji.push_back(yji[k]);
            slidingThetaji.push_back(thetaji[k] + 2*M_PI);
          }
        }
        else {
          slidingXji.push_back(xji[i+slidingWindow-1]);
          slidingYji.push_back(yji[i+slidingWindow-1]);
          slidingThetaji.push_back(thetaji[i+slidingWindow-1] + 2*M_PI);
        }

        std::cout << "i is : " << i << ", size of slidinXji is " << slidingXji.size() << endl;
        //std::cout<<summary.FullReport()<<std::endl;
        std::cout<<summary.BriefReport()<<std::endl;


        //for (size_t m = 0; m < xji.size(); ++m) {
        //  std::cout << "x_ji[" << m << "] = " << xji[m] << std::endl;}


    }

    // Record the ending time
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the time duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Output the duration in milliseconds
    std::cout << "Time duration: " << duration.count() << "milliseconds" << endl;

    std::ofstream outputFile("xji.txt");
    if (!outputFile.is_open()){
      std::cerr << "Error: Unable to open the file for writing." << std::endl;
      return 1;
    }

    // outputFile << "xji" << " " << "yji" << " " << "thetaji" << std::endl;
    //outputFile << "xji" << std::endl;
  
    //for (size_t i = 0; i < xji.size(); ++i){
      // outputFile << xji[i] << " " << yji[i] << " " << thetaji[i] << std::endl;
    //  outputFile << xji[i] << std::endl;
    //}

    outputFile.close();


    for (int i = 0; i < colSize; ++i) {
      double tirad = thetai[i] * M_PI / 180 + M_PI / 2; 
      xji_[i] = cos(tirad) * xji[i] - sin(tirad) * yji[i] + t_xji[i];
      yji_[i] = sin(tirad) * xji[i] + cos(tirad) * yji[i] + t_yji[i];
    }

    // std::cout << "Data saved to output.txt" << std::endl;
    plt::title("xji");
    plt::plot(visualization, xji_, "r*");
    plt::plot(visualization, t_xji, "b*");
    //plt::plot(visualization, c4_xji, "g*"); //ekf
    plt::show();

    plt::title("yji");
    plt::plot(visualization, yji_, "r*");
    plt::plot(visualization, t_yji, "b*");
    //plt::plot(visualization, c4_yji, "g*");
    plt::show();

    //plt::title("thetaji");
    //plt::plot(visualization, thetaji, "r*");
    //plt::plot(visualization, o_theta, "b*");
    //plt::show();


  ////**********************Save CSV file****************************
/*
    std::string header1 = "xji";
    std::string header2 = "yji";
    std::string header3 = "thetaji";
    std::string header4 = "vj";
    std::string header5 = "wj";
    std::string header6 = "trueXji";
    std::string header7 = "trueYji";
    std::string header8 = "trueThetaji";

    const std::string filename = "/home/asl/rpestm_ws/src/multiple_turtlebots_estm/data/Hardware/NLS/SlidingFilter/test4.csv";

    std::ofstream outputFile(filename);

    if (!outputFile.is_open()){
        std::cerr << "Error: Unable to open the file " << filename << std::endl;
        return 1;
    }

    // Write headers to the CSV file
    outputFile << header1 << "," << header2 << "," << header3 <<  "," << header4 << "," << header5 <<
                  "," << header6 << "," << header7 << "," << header8 << std::endl;

    cout << "here" << endl;

    // Write the vectors as columns in the CSV file
    for (size_t i = 0; i < colSize; ++i){
        outputFile << slidingXji[i];
        outputFile << ","; // Add a comma to separate columns
        outputFile << slidingYji[i];
        outputFile << ",";
        outputFile << slidingThetaji[i];
        outputFile << ",";
        outputFile << vj[i];
        outputFile << ",";
        outputFile << wj[i];
        outputFile << ",";
        outputFile << t_xji[i];
        outputFile << ",";
        outputFile << t_yji[i];
        outputFile << ",";
        outputFile << t_thetaji[i];
        outputFile << std::endl; // Start a new row
    }


    outputFile.close();

    std::cout << "Data saved to " << filename << std::endl;
*/
    return 0;
}