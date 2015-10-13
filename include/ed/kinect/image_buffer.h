#ifndef ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_
#define ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_

#include <queue>
#include <ros/callback_queue.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>

namespace rgbd
{
class Client;
}

namespace tf
{
class TransformListener;
}

class ImageBuffer
{

public:

    ImageBuffer();

    ~ImageBuffer();

    void initialize(const std::string& topic);

    bool nextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

private:

    std::string topic_;

    rgbd::Client* kinect_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;


};

#endif
