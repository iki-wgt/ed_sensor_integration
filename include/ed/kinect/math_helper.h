#ifndef ED_SENSOR_INTEGRATION_MATH_HELPER_H_
#define ED_SENSOR_INTEGRATION_MATH_HELPER_H_

#include <geolib/datatypes.h>

namespace ed_sensor_integration
{
    namespace math_helper
    {
        double QuaternionToYaw(const geo::Quaternion& q);
    }
}

#endif
