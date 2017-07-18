#ifndef BENDER_SELF_LASER_FILTER_H
#define BENDER_SELF_LASER_FILTER_H

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <uchile_laser_pipeline/SelfFilterConfig.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace uchile_laser_pipeline
{

class LaserScanSelfFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanSelfFilter();
    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

    // dynamic reconfigure server
    dynamic_reconfigure::Server<uchile_laser_pipeline::SelfFilterConfig> dyn_server;
    dynamic_reconfigure::Server<uchile_laser_pipeline::SelfFilterConfig>::CallbackType dyn_f;
    
  private:

    void getAngularLimits(float x, float y, float z, float r, float &theta_min, float &theta_max);

    void dynamicReconfigureCallback(uchile_laser_pipeline::SelfFilterConfig &config, uint32_t level);

    visualization_msgs::Marker genMarker(const std::string& target_frame, double radius, int marker_id);

    std::vector<std::string> _target_frames;
    std::string _l_hand_frame, _r_hand_frame;
    double _l_hand_radius, _r_hand_radius;
    double _l_hand_inflated_radius, _r_hand_inflated_radius;
    double _publish_markers;

    laser_geometry::LaserProjection projector_;
    
    // tf listener to transform scans into the box_frame
    tf::TransformListener tf_listener;

    ros::Publisher marker_pub;
};

}

#endif /* self_filter.h */
