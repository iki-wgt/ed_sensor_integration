#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_CALC_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_CALC_H_

#include "ed_sensor_integration/properties/convex_hull.h"

namespace convex_hull
{

void create(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c, geo::Pose3D& pose);

void calculateEdgesAndNormals(ConvexHull& c);

bool collide(const ConvexHull& c1, const geo::Vector3& pos1,
             const ConvexHull& c2, const geo::Vector3& pos2,
             float xy_padding = 0, float z_padding = 0);

}

#endif