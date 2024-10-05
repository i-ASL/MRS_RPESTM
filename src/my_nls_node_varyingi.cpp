/**
 *  @brief Robot 2D trajectory optimization
 *
 *  @author Atsushi Sakai
 **/
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "math.h"
#include "csvparser.h"
//#include "matplotlibcpp.h"

using namespace std;

//namespace plt = matplotlibcpp;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::HuberLoss;
using ceres::TukeyLoss;
using ceres::ArctanLoss;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
// M estimation. Robust curve fitting

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
    withoutNetworkOdomConstraint(double delta_t, double v_i, double w_i)
        : delta_t_(delta_t), v_i_(v_i), w_i_(w_i) {}

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
        residual[0] = (nx[0] - (cx[0] + delta_t_*(vj[0]*cos(cyaw[0])+cy[0]*w_i_-v_i_))) / 0.1;
        residual[1] = (ny[0] - (cy[0] + delta_t_*(vj[0]*sin(cyaw[0])-cx[0]*w_i_))) / 0.1;
        residual[2] = (nyaw[0] - (cyaw[0] + delta_t_*(wj[0]-w_i_))) / 0.01;
        residual[3] = (nvj[0] - vj[0]) / 0.01;
        residual[4] = (nwj[0] - wj[0]) / 0.01;
        return true;
    }

    static ceres::CostFunction* Create(
            const double delta_t,
            const double v_i,
            const double w_i)
    {
        return (new ceres::AutoDiffCostFunction<withoutNetworkOdomConstraint,5,1,1,1,1,1,1,1,1,1,1>(
                    new withoutNetworkOdomConstraint(delta_t, v_i, w_i)));
    }

private:
    const double delta_t_;
    const double v_i_;
    const double w_i_;
//    const double v_i = 0.2;
//    const double w_i = 0.1;
//    const double v_j = 0.4;
//    const double w_j = 0.09;
};

double findMedian(const vector<double>& data)
{
    vector<double> sortedData = data;
    // Sort the vector in ascending order
    std::sort(sortedData.begin(), sortedData.end());

    // Calculate the median
    double median;
    int size = sortedData.size();

    if (size % 2 == 0){
        // If enven number of elements, take the average of the two middle elements
        median = static_cast<double>(sortedData[size/2 - 1] + sortedData[size/2 + 1]) / 2.0;
    } else{
        // If odd number of elements, the median is the middle element
        median = static_cast<double>(sortedData[size/2]);
    }
    return median;
}

double findFirstQuartile(const vector<double>& data){
    vector<double> sortedData = data;

    // Sort the copy in ascending order
    std::sort(sortedData.begin(), sortedData.end());

    // Calculate the index for the first quartile (Q1)
    int n = sortedData.size();
    double index = (n + 1.0) / 4.0;
//    cout << "index is : " <<  index << endl;
    int lowerIndex = static_cast<int>(index);
//    cout << "lower index is : " <<  lowerIndex << endl;

    int upperIndex = lowerIndex + 1;

    // Check if the index is an integer or requires interpolation
    if (index == lowerIndex){
        // If the index is an integer, Q1 is the value at that index
        return sortedData[lowerIndex - 1];
    } else{
        // If the index is not an integer, interpolate between the two nearest values
        double lowerValue = sortedData[lowerIndex - 1];
        double upperValue = sortedData[upperIndex - 1];
        double interpolationFactor = index - lowerIndex;

        return lowerValue + interpolationFactor * (upperValue - lowerValue);
    }
}

double findThirdQuartile(const std::vector<double>& data) {
    // Make a copy of the input data vector
    std::vector<double> sortedData = data;

    // Sort the copy in ascending order
    std::sort(sortedData.begin(), sortedData.end());

    // Calculate the index for the third quartile (Q3)
    int n = sortedData.size();
    double index = (3.0 * n + 1.0) / 4.0; // 3/4 of the way from the start
    int lowerIndex = static_cast<int>(index);
    int upperIndex = lowerIndex + 1;
//    cout << "Index is : " << index << endl;
//    cout << "lowerIndex is : " << lowerIndex << endl;

    // Check if the index is an integer or requires interpolation
    if (index == lowerIndex) {
        // If the index is an integer, Q3 is the value at that index
        return sortedData[lowerIndex - 1]; // Subtract 1 to convert to 0-based index
    } else {
        // If the index is not an integer, interpolate between the two nearest values
        double lowerValue = sortedData[lowerIndex - 1];
        double upperValue = sortedData[upperIndex - 1];
        double interpolationFactor = index - lowerIndex;

        return lowerValue + interpolationFactor * (upperValue - lowerValue);
    }
}


int main(int argc, char** argv){
  cout<<"Start similation"<<endl;
  google::InitGoogleLogging(argv[0]);

  //data read
//  CSVParser csvparser("/home/gihun/catkin_tools_ws/files/1003/_2023-10-03-15-19-08-sync_data.csv");
  CSVParser csvparser("/home/asl/rpestm_ws/src/2023-10-16-15-00-50-sync_data.csv");

  int colSize = csvparser.ncol_ / 2;

  //parameter
  vector<double> xji;
  vector<double> yji;
  vector<double> thetaji;

  vector<double> v_i;
  vector<double> w_i;

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
  vector<double> c4_thetaji;

  //For adding outliers to the observation data.
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

  vector<double> vj(colSize, 0.3);
  vector<double> wj(colSize, 0.05);
  vector<double> trueVj(colSize, 0.4);
  vector<double> trueWj(colSize, 0.09);

// read data from CSV
  for(int i=0;i< colSize ;i++){
      xji.push_back(csvparser.data_[i][5]);
      yji.push_back(csvparser.data_[i][6]);
      thetaji.push_back(csvparser.data_[i][7]);

      v_i.push_back(csvparser.data_[i][30]);
      w_i.push_back(csvparser.data_[i][31]);

      rho.push_back(csvparser.data_[i][8]);
      beta.push_back(csvparser.data_[i][9]);
      theta.push_back(csvparser.data_[i][10]);

      delta_t.push_back(csvparser.data_[i][11]);

      t_xji.push_back(csvparser.data_[i][12]);
      t_yji.push_back(csvparser.data_[i][13]);
      t_thetaji.push_back(csvparser.data_[i][14]);

      c4_xji.push_back(csvparser.data_[i][27]);
      c4_yji.push_back(csvparser.data_[i][28]);
      c4_thetaji.push_back(csvparser.data_[i][29]);
            
      visualization.push_back(i); // for matplotlib plot
  }

  /*************************Adds outliers to measurement data************************/
  
  /**********************************************************************************/

  // Initialize parameters for optimization

  // init param
  vector<double> initialXji;
  vector<double> initialYji;
  vector<double> initialThetaji;
  initialXji=xji;
  initialYji=yji;
  initialThetaji=thetaji;
  vector<double> initialVj;
  vector<double> initialWj;
  initialVj = vj;
  initialWj = wj;
// ====================================Optimization========================================
  // Record the starting time
  auto start_time = std::chrono::high_resolution_clock::now();

  // Perform task that you want to measure the duration

  ceres::Problem problem;
  for(int i = 0; i < colSize - 1; i++){

//    // case1~3 odometry constraint
//    problem.AddResidualBlock(
//        NetworkOdomConstraint::Create(delta_t[i]),
//        NULL,
//        &(xji[i]),
//        &(yji[i]),
//        &(thetaji[i]),
//        &(xji[i+1]),
//        &(yji[i+1]),
//        &(thetaji[i+1])
//        );

     // case4 odometry constraint
    problem.AddResidualBlock(
          withoutNetworkOdomConstraint::Create(delta_t[i], v_i[i], w_i[i]),
          NULL,
          &(xji[i]),
          &(yji[i]),
          &(thetaji[i]),
          &(vj[i]),
          &(wj[i]),
          &(xji[i+1]),
          &(yji[i+1]),
          &(thetaji[i+1]),
          &(vj[i+1]),
          &(wj[i+1])
          );

    // // measurement constraint
//    problem.AddResidualBlock(
//      MyConstraintCase0::Create(rho[i],beta[i],theta[i]),
//      NULL,
//      &xji[i],
//      &yji[i],
//      &thetaji[i]
//      );

//    problem.AddResidualBlock(
//      MyConstraintCase1::Create(rho[i]),
//      NULL,
//      &xji[i],
//      &yji[i]
//      );

//    problem.AddResidualBlock(
//          MyConstraintCase2::Create(beta[i]),
//        NULL,
//        &xji[i],
//        &yji[i]
//          );

      problem.AddResidualBlock(
            MyConstraintCase3::Create(rho[i], beta[i]), // o_rhom, o_beta : data with outliers
          NULL,
          &xji[i],
          &yji[i]
            );
  }

  // Optimization
  Solver::Options options;
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;
  Solver::Summary summary;
  Solve(options,&problem,&summary);

  for (auto& ele : thetaji)
    ele += 2 * M_PI;

  // Record the ending time
  auto end_time = std::chrono::high_resolution_clock::now();

  // Calculate the time duration
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  // Output the duration in milliseconds
  std::cout << "Time duration: " << duration.count() << "milliseconds" << endl;



////**********************Save CSV file****************************

//  std::string header1 = "xji";
//  std::string header2 = "yji";
//  std::string header3 = "thetaji";
//  std::string header4 = "vj";
//  std::string header5 = "wj";
//  std::string header6 = "trueXji";
//  std::string header7 = "trueYji";
//  std::string header8 = "trueThetaji";

//  const std::string filename = "/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withoutOutlier/Nls/test.csv";


//  std::ofstream outputFile(filename);

//  if (!outputFile.is_open()){
//      std::cerr << "Error: Unable to open the file " << filename << std::endl;
//      return 1;
//  }

//  // Write headers to the CSV file
//  outputFile << header1 << "," << header2 << "," << header3 <<  "," << header4 << "," << header5 <<
//                "," << header6 << "," << header7 << "," << header8 << std::endl;

//  // Write the vectors as columns in the CSV file
//  size_t maxLength = t_xji.size();
//  for (size_t i = 0; i < maxLength; ++i){
//      if (i < xji.size()){
//          outputFile << xji[i];
//      }
//      outputFile << ","; // Add a comma to separate columns
//      if (i < yji.size()){
//          outputFile << yji[i];
//      }
//      outputFile << ",";
//      if (i < thetaji.size()){
//          outputFile << thetaji[i];
//      }
//      outputFile << ",";
//      if (i < vj.size()){
//          outputFile << vj[i];
//      }
//      outputFile << ",";
//      if (i < wj.size()){
//          outputFile << wj[i];
//      }
//      outputFile << ",";
//      if (i < t_xji.size()){
//          outputFile << t_xji[i];
//      }
//      outputFile << ",";
//      if (i < t_yji.size()){
//          outputFile << t_yji[i];
//      }
//      outputFile << ",";
//      if (i < t_thetaji.size()){
//          outputFile << t_thetaji[i];
//      }
//      outputFile << std::endl; // Start a new row
//  }

//  outputFile.close();

//  std::cout << "Data saved to " << filename << std::endl;


//**********************Visualization****************************
//    // Set the "super title"
//    plt::suptitle("Odom_weight_(0.1,0.1,0.01), meas_weight_null, odom_rho_null, meas_rho_null");
//    plt::subplot(3,1,1);
//    plt::plot(t_xji, {{"label", "true_x"}});
//    plt::plot(xji, {{"label", "ceres_x"}});
//  //  plt::plot(ix, {{"label", "init_x"}});
//  //  plt::plot(c4_xji, {{"label", "kf_x"}});
//  //  plt::title("x_ji");
//    plt::grid(true);
//    plt::legend();


//    plt::subplot(3,1,2);
//    plt::plot(t_yji, {{"label", "true_y"}});
//    plt::plot(yji, {{"label", "ceres_y"}});
//  //  plt::plot(iy, {{"label", "init_y"}});
//  //  plt::plot(c4_yji, {{"label", "Kf_y"}});
//  //  plt::title("y_ji");
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(3,1,3);
//    plt::plot(t_thetaji, {{"label", "true_theta"}});
//    plt::plot(thetaji, {{"label", "ceres_theta"}});
//  //  plt::plot(iyaw, {{"label", "init_theta"}});
//  //  plt::plot(c4_thetaji, {{"label", "Kf_theta"}});
//  //  plt::title("theta_ji");
//    plt::grid(true);
//    plt::legend();

//    plt::show();

//    plt::subplot(3,1,1);
//    plt::plot(t_xji, {{"label", "true_x"}});
//    plt::plot(xji, {{"label", "ceres_x"}});
//  //  plt::plot(ix, {{"label", "init_x"}});
//  //  plt::plot(c4_xji, {{"label", "kf_x"}});
//  //  plt::title("x_ji");
//    plt::grid(true);
//    plt::legend();


//    plt::subplot(3,1,2);
//    plt::plot(t_yji, {{"label", "true_y"}});
//    plt::plot(yji, {{"label", "ceres_y"}});
//  //  plt::plot(iy, {{"label", "init_y"}});
//  //  plt::plot(c4_yji, {{"label", "Kf_y"}});
//  //  plt::title("y_ji");
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(3,1,3);
//    plt::plot(t_thetaji, {{"label", "true_theta"}});
//    plt::plot(thetaji, {{"label", "ceres_theta"}});
//  //  plt::plot(iyaw, {{"label", "init_theta"}});
//  //  plt::plot(c4_thetaji, {{"label", "Kf_theta"}});
//  //  plt::title("theta_ji");
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(5,1,1);
//    plt::plot(t_xji, {{"label", "true_x"}});
//    plt::plot(xji, {{"label", "ceres_x"}});
//    plt::grid(true);
//    plt::legend();


//    plt::subplot(5,1,2);
//    plt::plot(t_yji, {{"label", "true_y"}});
//    plt::plot(yji, {{"label", "ceres_y"}});
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(5,1,3);
//    plt::plot(t_thetaji, {{"label", "true_theta"}});
//    plt::plot(thetaji, {{"label", "ceres_theta"}});
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(5,1,4);
//    plt::plot(trueVj, {{"label", "true_vj"}});
//    plt::plot(vj, {{"label", "vj"}});
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(5,1,5);
//    plt::plot(trueWj, {{"label", "true_wj"}});
//    plt::plot(wj, {{"label", "wj"}});
//    plt::grid(true);
//    plt::legend();

//    plt::show();

  return 0;
}