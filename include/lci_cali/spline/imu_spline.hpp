#pragma once 
#include "Eigen/Core"
#include "lci_cali/continuous_se3.hpp"

namespace lci_cali{

template <int N>
class IMUSpline : public BSplineSE3<N>
{
private:

public:
    IMUSpline(double start_time, double end_time, double interval):
        BSplineSE3<N>(start_time, end_time, interval)
    {
                
    }
    ~IMUSpline(){}
};

}

