#include "ComputeSvdMetrics.h"

#include <Eigen/SVD>

#include <algorithm>

bool ComputeSvdMetrics(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
    double& conditionNumber,
    double& manipulability,
    std::vector<double>& singularValues)
{
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>> svd(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& sv = svd.singularValues();

    singularValues.clear();
    conditionNumber = 0.0;
    manipulability = 0.0;

    if (sv.size() == 0)
        return true;

    for (int i = 0; i < sv.size(); ++i)
        singularValues.push_back(sv[i]);
    std::sort(singularValues.begin(), singularValues.end(), std::greater<double>());

    const double sMin = singularValues.back();
    const double sMax = singularValues.front();

    conditionNumber = (sMin > 1e-10) ? (sMax / sMin) : 1e10;

    double w = 1.0;
    for (const double& s : singularValues)
        w *= s;
    manipulability = w;

    return (conditionNumber > 1000.0) || (sMin < 1e-8);
}
