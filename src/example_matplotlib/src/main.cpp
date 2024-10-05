#include <iostream>
#include <vector>
#include "matplotlibcpp.h"
#include "csvparser.h"

using namespace std;

namespace plt = matplotlibcpp;

int main()
{
    CSVParser csvparserTrue("ceresTrue.csv");
    CSVParser csvparserNull("ceresNull.csv");
    CSVParser csvparserCauchy("ceresCauchy2.0.csv");
    CSVParser csvparserHuber("ceresHuber8.0.csv");
    CSVParser csvparserTukey("ceresTukey8.0.csv");
    CSVParser csvparserArctan("ceresArctan10.0.csv");

    vector<double> trueXji;
    vector<double> trueYji;
    vector<double> trueThetaji;

    vector<double> nullXji;
    vector<double> nullYji;
    vector<double> nullThetaji;

    vector<double> cauchyXji;
    vector<double> cauchyYji;
    vector<double> cauchyThetaji;

    vector<double> huberXji;
    vector<double> huberYji;
    vector<double> huberThetaji;

    vector<double> tukeyXji;
    vector<double> tukeyYji;
    vector<double> tukeyThetaji;

    vector<double> arctanXji;
    vector<double> arctanYji;
    vector<double> arctanThetaji;

    for (int i = 0; i < csvparserNull.ncol_; i++){
        trueXji.push_back(csvparserTrue.data_[i][0]);
        trueYji.push_back(csvparserTrue.data_[i][1]);
        trueThetaji.push_back(csvparserTrue.data_[i][2]);
    }
    for (int i = 0; i < csvparserNull.ncol_; i++){
        nullXji.push_back(csvparserNull.data_[i][0]);
        nullYji.push_back(csvparserNull.data_[i][1]);
        nullThetaji.push_back(csvparserNull.data_[i][2]);
    }
    for (int i = 0; i < csvparserNull.ncol_; i++){
        cauchyXji.push_back(csvparserCauchy.data_[i][0]);
        cauchyYji.push_back(csvparserCauchy.data_[i][1]);
        cauchyThetaji.push_back(csvparserCauchy.data_[i][2]);
    }
    for (int i = 0; i < csvparserNull.ncol_; i++){
        huberXji.push_back(csvparserHuber.data_[i][0]);
        huberYji.push_back(csvparserHuber.data_[i][1]);
        huberThetaji.push_back(csvparserHuber.data_[i][2]);
    }
    for (int i = 0; i < csvparserNull.ncol_; i++){
        tukeyXji.push_back(csvparserTukey.data_[i][0]);
        tukeyYji.push_back(csvparserTukey.data_[i][1]);
        tukeyThetaji.push_back(csvparserTukey.data_[i][2]);
    }
    for (int i = 0; i < csvparserNull.ncol_; i++){
        arctanXji.push_back(csvparserArctan.data_[i][0]);
        arctanYji.push_back(csvparserArctan.data_[i][1]);
        arctanThetaji.push_back(csvparserArctan.data_[i][2]);
    }

    plt::suptitle("Result");

    plt::subplot(3,1,1);
    plt::plot(trueXji, {{"label", "true"}});
    plt::plot(nullXji, {{"label", "null"}});
    plt::plot(cauchyXji, {{"label", "cauchy"}});
    plt::plot(huberXji, {{"label", "huber"}});
    plt::plot(tukeyXji, {{"label", "tukey"}});
    plt::plot(arctanXji, {{"label", "arctan"}});
    plt::grid(true);
    plt::legend();

    plt::subplot(3,1,2);
    plt::plot(trueYji, {{"label", "true"}});
    plt::plot(nullYji, {{"label", "null"}});
    plt::plot(cauchyYji, {{"label", "cauchy"}});
    plt::plot(huberYji, {{"label", "huber"}});
    plt::plot(tukeyYji, {{"label", "tukey"}});
    plt::plot(arctanYji, {{"label", "arctan"}});
    plt::grid(true);
    plt::legend();

    plt::subplot(3,1,3);
    plt::plot(trueThetaji, {{"label", "true"}});
    plt::plot(nullThetaji, {{"label", "null"}});
    plt::plot(cauchyThetaji, {{"label", "cauchy"}});
    plt::plot(huberThetaji, {{"label", "huber"}});
    plt::plot(tukeyThetaji, {{"label", "tukey"}});
    plt::plot(arctanThetaji, {{"label", "arctan"}});
    plt::grid(true);
    plt::legend();

    plt::show();

    return 0;
}