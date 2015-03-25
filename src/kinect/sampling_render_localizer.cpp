#include "sampling_render_localizer.h"

#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

namespace
{

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer_, std::vector<unsigned int>& pixels_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), num_pixels(0), pixels(pixels_)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0)
        {
            z_buffer.at<float>(y, x) = depth;
            pixels[num_pixels] = y * z_buffer.cols + x;
            ++num_pixels;
        }
        else if (depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat& z_buffer;
    unsigned int num_pixels;
    std::vector<unsigned int>& pixels;

};

}

// ----------------------------------------------------------------------------------------------------

SamplingRenderLocalizer::SamplingRenderLocalizer()
{
}

// ----------------------------------------------------------------------------------------------------

SamplingRenderLocalizer::~SamplingRenderLocalizer()
{
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D SamplingRenderLocalizer::localize(const geo::Pose3D& sensor_pose, const rgbd::Image& image, const ed::WorldModel& world, const std::set<ed::UUID>& loc_ids)
{
    // - - - - - - - - - - - - - - - - - -
    // Render world model based on pose calculated above

    rgbd::View view(image, 80);

    const geo::DepthCamera& rasterizer = view.getRasterizer();

    std::vector<unsigned int> pixels(view.getWidth() * view.getHeight());

    cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);

    cv::Mat depth_image(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
    for(int y = 0; y < view.getHeight(); ++y)
        for(int x = 0; x < view.getWidth(); ++x)
            depth_image.at<float>(y, x) = view.getDepth(x, y);

    SampleRenderResult res(model, pixels);

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && e->has_pose() && (loc_ids.empty() || loc_ids.find(e->id()) != loc_ids.end()))
        {
            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            // Render
            rasterizer.render(opt, res);

            if (res.num_pixels > 0)
                entities.push_back(e);
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try other poses and determine best scoring pose

    cv::Mat best_model;
    double min_error = 1e9; // TODO
    geo::Pose3D best_pose;


    for(double da = -0.1; da < 0.1; da += 0.05)
    {
        geo::Matrix3 m;
        m.setRPY(0, 0, da);

        for(double dx = -0.2; dx < 0.2; dx += 0.05)
        {
            for(double dy = -0.2; dy < 0.2; dy += 0.05)
            {

                geo::Pose3D test_pose;
                test_pose.t = sensor_pose.t + geo::Vector3(dx, dy, 0);
                test_pose.R = m * sensor_pose.R;

                // Render world
                cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
                SampleRenderResult res(model, pixels);
                for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
                {
                    const ed::EntityConstPtr& e = *it;

                    geo::Pose3D pose = test_pose.inverse() * e->pose();
                    geo::RenderOptions opt;
                    opt.setMesh(e->shape()->getMesh(), pose);                    

                    // Render
                    rasterizer.render(opt, res);
                }

                int n = 0;
                double total_error = 0;
                for(unsigned int i = 0; i < res.pixels.size(); ++i)
                {
                    unsigned int p_idx = res.pixels[i];

                    float ds = depth_image.at<float>(p_idx);

                    if (ds > 0) // TODO
                    {
                        float dm = model.at<float>(p_idx);
                        float err = std::min<float>(0.05, std::abs(dm - ds));
                        total_error += (err * err);
                        ++n;
                    }
                }

                if (n > 0)
                {
                    double avg_error = total_error / n;
                    if (avg_error < min_error)
                    {
                        min_error = avg_error;
                        best_pose = test_pose;
                        best_model = model;
                    }
                }


            }
        }
    }

    return best_pose;
}

