#include "ed/kinect/math_helper.h"

#include <cmath>
#include <geolib/datatypes.h>

namespace ed_sensor_integration
{
    namespace math_helper
    {
        double QuaternionToYaw(const geo::Quaternion& q)
        {
            double mag = sqrt(q.getZ() * q.getZ() + q.getW() * q.getW());
            return 2 * acos(q.getZ() / mag);
        }
    }
}
