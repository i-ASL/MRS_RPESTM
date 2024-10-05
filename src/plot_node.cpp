#include <iostream>
#include <vector>
#include "matplotlibcpp.h"
#include "csvparser.h"

using namespace std;

namespace plt = matplotlibcpp;

double calculateRMSE(const std::vector<double>& gt, const std::vector<double>& input)
{
  if (gt.size() != input.size()){
    throw std::runtime_error("Vectors must have the same size");
  }

  double sumSquaredError = 0.0;
  for (size_t i = 0; i < gt.size(); ++i){
    double error = gt[i] - input[i];
    sumSquaredError += error * error;
  }

  double meanSquaredError = sumSquaredError / gt.size();
  double rmse = std::sqrt(meanSquaredError);

  return rmse;
}

int main()
{
//    // 10%
//    CSVParser csvparserTrue("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/l2norm.csv");
//    CSVParser csvparserNull("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/huber0.5.csv");
//    CSVParser csvparserCauchy("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/cauchy0.5.csv");
//    CSVParser csvparserHuber("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/tukey3.0.csv");
//    CSVParser csvparserTukey("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/arctan3.0.csv");
//    CSVParser csvparserArctan("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/arctan3.0.csv");

////    // 20%
//    CSVParser csvparserTrue("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier20%/l2norm.csv");
//    CSVParser csvparserNull("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier20%/huber0.5.csv");
//    CSVParser csvparserCauchy("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier20%/cauchy0.5.csv");
//    CSVParser csvparserHuber("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier20%/tukey3.0.csv");
//    CSVParser csvparserTukey("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier20%/arctan3.0.csv");

////    // 30%
//    CSVParser csvparserTrue("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/l2norm.csv");
//    CSVParser csvparserNull("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/huber0.5.csv");
//    CSVParser csvparserCauchy("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/cauchy0.5.csv");
//    CSVParser csvparserHuber("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/tukey3.0.csv");
//    CSVParser csvparserTukey("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier30%/arctan3.0.csv");

////    // 40%
//    CSVParser csvparserTrue("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier40%/l2norm.csv");
//    CSVParser csvparserNull("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier40%/huber0.5.csv");
//    CSVParser csvparserCauchy("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier40%/cauchy0.5.csv");
//    CSVParser csvparserHuber("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier40%/tukey3.0.csv");
//    CSVParser csvparserTukey("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier40%/arctan3.0.csv");

////    // 50%
//    CSVParser csvparserTrue("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier50%/l2norm.csv");
//    CSVParser csvparserNull("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier50%/huber0.5.csv");
//    CSVParser csvparserCauchy("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier50%/cauchy0.5.csv");
//    CSVParser csvparserHuber("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier50%/tukey3.0.csv");
//    CSVParser csvparserTukey("/home/gihun/catkin_ws/src/multiple_turtlebots_sim/data/withOutlier/case4/outlier50%/arctan3.0.csv");

//    vector<double> trueXji;
//    vector<double> trueYji;
//    vector<double> trueThetaji;

//    vector<double> nullXji;
//    vector<double> nullYji;
//    vector<double> nullThetaji;

//    vector<double> cauchyXji;
//    vector<double> cauchyYji;
//    vector<double> cauchyThetaji;

//    vector<double> huberXji;
//    vector<double> huberYji;
//    vector<double> huberThetaji;

//    vector<double> tukeyXji;
//    vector<double> tukeyYji;
//    vector<double> tukeyThetaji;

//    vector<double> arctanXji;
//    vector<double> arctanYji;
//    vector<double> arctanThetaji;

//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        trueXji.push_back(csvparserTrue.data_[i][0]);
//        trueYji.push_back(csvparserTrue.data_[i][1]);
//        trueThetaji.push_back(csvparserTrue.data_[i][2]);
//    }
//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        nullXji.push_back(csvparserNull.data_[i][0]);
//        nullYji.push_back(csvparserNull.data_[i][1]);
//        nullThetaji.push_back(csvparserNull.data_[i][2]);
//    }
//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        cauchyXji.push_back(csvparserCauchy.data_[i][0]);
//        cauchyYji.push_back(csvparserCauchy.data_[i][1]);
//        cauchyThetaji.push_back(csvparserCauchy.data_[i][2]);
//    }
//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        huberXji.push_back(csvparserHuber.data_[i][0]);
//        huberYji.push_back(csvparserHuber.data_[i][1]);
//        huberThetaji.push_back(csvparserHuber.data_[i][2]);
//    }
//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        tukeyXji.push_back(csvparserTukey.data_[i][0]);
//        tukeyYji.push_back(csvparserTukey.data_[i][1]);
//        tukeyThetaji.push_back(csvparserTukey.data_[i][2]);
//    }
//    for (int i = 0; i < csvparserNull.ncol_; i++){
//        arctanXji.push_back(csvparserArctan.data_[i][0]);
//        arctanYji.push_back(csvparserArctan.data_[i][1]);
//        arctanThetaji.push_back(csvparserArctan.data_[i][2]);
//    }

//    /*********************calculate RMSE********************/
//    double leastSquareRmseX = calculateRMSE(trueXji, nullXji);
//    double leastSquareRmseY = calculateRMSE(trueYji, nullYji);
//    double leastSquareRmseT = calculateRMSE(trueThetaji, nullThetaji);

//    double cauchyRmseX = calculateRMSE(trueXji, cauchyXji);
//    double cauchyRmseY = calculateRMSE(trueYji, cauchyYji);
//    double cauchyRmseT = calculateRMSE(trueThetaji, cauchyThetaji);

//    double huberRmseX = calculateRMSE(trueXji, huberXji);
//    double huberRmseY = calculateRMSE(trueYji, huberYji);
//    double huberRmseT = calculateRMSE(trueThetaji, huberThetaji);

//    double tukeyRmseX = calculateRMSE(trueXji, tukeyXji);
//    double tukeyRmseY = calculateRMSE(trueYji, tukeyYji);
//    double tukeyRmseT = calculateRMSE(trueThetaji, tukeyThetaji);

//    double arctanRmseX = calculateRMSE(trueXji, arctanXji);
//    double arctanRmseY = calculateRMSE(trueYji, arctanYji);
//    double arctanRmseT = calculateRMSE(trueThetaji, arctanThetaji);


//    /*********************visualize graph********************/
//    plt::suptitle("M-Estimator Xji, Yji, Thetaji");

//    plt::subplot(3,1,1);
//    plt::plot(trueXji, {{"label", "true"}});
//    plt::plot(nullXji, {{"label", "least-square"}});
//    plt::plot(cauchyXji, {{"label", "cauchy"}});
//    plt::plot(huberXji, {{"label", "huber"}});
//    plt::plot(tukeyXji, {{"label", "tukey"}});
//    plt::plot(arctanXji, {{"label", "arctan"}});
//    plt::grid(true);
//    plt::legend();

//    plt::subplot(3,1,2);
//    plt::plot(trueYji, {{"label", "true"}});
//    plt::plot(nullYji, {{"label", "least-square"}});
//    plt::plot(cauchyYji, {{"label", "cauchy"}});
//    plt::plot(huberYji, {{"label", "huber"}});
//    plt::plot(tukeyYji, {{"label", "tukey"}});
//    plt::plot(arctanYji, {{"label", "arctan"}});
//    plt::grid(true);
////    plt::legend();

//    plt::subplot(3,1,3);
//    plt::plot(trueThetaji, {{"label", "true"}});
//    plt::plot(nullThetaji, {{"label", "least-square"}});
//    plt::plot(cauchyThetaji, {{"label", "cauchy"}});
//    plt::plot(huberThetaji, {{"label", "huber"}});
//    plt::plot(tukeyThetaji, {{"label", "tukey"}});
//    plt::plot(arctanThetaji, {{"label", "arctan"}});
//    plt::grid(true);
////    plt::legend();

//    plt::show();

//    /*********************visualize RMSE********************/
//    vector<double> vectorOfRmseXji ={leastSquareRmseX, cauchyRmseX, huberRmseX, tukeyRmseX, arctanRmseX};
//    vector<double> vectorOfRmseYji ={leastSquareRmseY, cauchyRmseY, huberRmseY, tukeyRmseY, arctanRmseY};
//    vector<double> vectorOfRmseThetaji ={leastSquareRmseT, cauchyRmseT, huberRmseT, tukeyRmseT, arctanRmseT};

//    vector<string> categories = {"L2-norm", "Cauchy", "Huber", "Tukey", "Arctan"};
//    vector<int> nums = {0,1,2,3,4};
//    plt::suptitle("RMSE");

//    plt::subplot(3,1,1);
//    plt::bar(vectorOfRmseXji);
//    plt::xticks(nums, categories);

//    plt::subplot(3,1,2);
//    plt::bar(vectorOfRmseYji);
//    plt::xticks(nums, categories);

//    plt::subplot(3,1,3);
//    plt::bar(vectorOfRmseThetaji);
//    plt::xticks(nums, categories);

//    plt::show();

    return 0;
}
