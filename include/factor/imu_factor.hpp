#pragma once

#include "basalt/spline/so3_spline.h"
#include "basalt/spline/spline_common.h"
#include "basalt/spline/ceres_spline_helper.h"
#include "sensor_data/imu_data.hpp"

namespace lci_cali{

class SplineMeasurementFactor: public basalt::CeresSplineHelper<4>
{
private:
    IMUData imu_data;
public:
    SplineMeasurementFactor(/* args */);
    ~SplineMeasurementFactor();
    template<class T>
    void operator()(T const* const* sKnots, T* residual) const{
        
    }
};

}

