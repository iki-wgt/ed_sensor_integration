#include "ed/kinect/math_helper.h"

#include <cmath>
#include <geolib/datatypes.h>

#include <iostream>

namespace ed_sensor_integration
{
    namespace math_helper
    {
        double AngleBetweenTwoQuaternions(const geo::Quaternion& q1, const geo::Quaternion& q2)
        {

            geo::Mat3 rotMat1 = QuaternionToRotationMatrix(q1);
            geo::Mat3 rotMat2 = QuaternionToRotationMatrix(q2);

            // rotate forward based on the quaternion rotation matrix.
            geo::Vec3 forward = geo::Vec3(1, 0, 0);
            geo::Vec3 forward1 = rotMat1 * forward;
            geo::Vec3 forward2 = rotMat2 * forward;
            // get angle between rotated vectors
            return atan2(forward1.x, forward1.y) - atan2(forward2.x, forward2.y);
        }

        geo::Mat3 QuaternionToRotationMatrix(const geo::Quaternion& q1)
        {
            //http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
            geo::Quaternion q1N = q1.normalized();
            return geo::Mat3(1 - 2 * q1N.y * q1N.y - 2 * q1N.z * q1N.z, \
                             2 * q1N.x * q1N.y - 2 * q1N.z * q1N.w, \
                             2 * q1N.x * q1N.z + 2 * q1N.y * q1N.w, \

                             2 * q1N.x * q1N.y + 2 * q1N.z * q1N.w, \
                             1 - 2 * q1N.x * q1N.x - 2 * q1N.z * q1N.z, \
                             2 * q1N.y * q1N.z - 2 * q1N.x * q1N.w, \

                             2 * q1N.x * q1N.z - 2 * q1N.y * q1N.w, \
                             2 * q1N.y * q1N.z + 2 * q1N.x * q1N.w, \
                             1 - 2 * q1N.x * q1N.x - 2 * q1N.y * q1N.y);
        }

        double fmod(double value, double mod)
        {
          if(value >=0)
          {
            if(value > mod)
            {
              return fmod(value-mod,mod);
            }
            else
            {
              return value;
            }
          }
          else
          {
            return fmod(value+mod,mod);
          }
        }
    }
}
