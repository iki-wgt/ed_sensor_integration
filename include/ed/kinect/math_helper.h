#ifndef ED_SENSOR_INTEGRATION_MATH_HELPER_H_
#define ED_SENSOR_INTEGRATION_MATH_HELPER_H_

#include <geolib/datatypes.h>

namespace ed_sensor_integration
{
    namespace math_helper
    {
        double AngleBetweenTwoQuaternions(const geo::Quaternion& q1, const geo::Quaternion& q2);
        geo::Mat3 QuaternionToRotationMatrix(const geo::Quaternion& q1);
        double fmod(double value, double mod);
    }
}

#endif
