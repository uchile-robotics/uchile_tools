#include "uchile_laser_pipeline/self_filter.h"

uchile_laser_pipeline::LaserScanSelfFilter::LaserScanSelfFilter() {

}

void uchile_laser_pipeline::LaserScanSelfFilter::dynamicReconfigureCallback(uchile_laser_pipeline::SelfFilterConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: {l,r} radius: %.2f, %.2f. Publish markers: %s", 
            config.l_radius,
            config.r_radius, 
            config.publish_markers ? "True" : "False");
    _publish_markers = config.publish_markers;
    _l_hand_inflated_radius = std::max(_l_hand_radius, config.l_radius);
    _r_hand_inflated_radius = std::max(_r_hand_radius, config.r_radius);
}

// TODO: wait for tfs a few seconds while configuring the pipeline
bool uchile_laser_pipeline::LaserScanSelfFilter::configure() {

    bool succeeded = true;
    if (!getParam("l_hand_link", _l_hand_frame)) {
        ROS_ERROR(
        "The 'l_hand_link' parameter is not set. Should be set to the tf frame used by the left hand.");
        succeeded = false;
    }
    if (!getParam("r_hand_link", _r_hand_frame)) {
        ROS_ERROR(
        "The 'r_hand_link' parameter is not set. Should be set to the tf frame used by the right hand.");
        succeeded = false;
    }
    if (!getParam("l_hand_radius", _l_hand_radius)) {
        ROS_ERROR(
        "The 'l_hand_radius' parameter is not set. Should be set to the left hand radius.");
        succeeded = false;
    }
    if (!getParam("r_hand_radius", _r_hand_radius)) {
        ROS_ERROR(
        "The 'r_hand_radius' parameter is not set. Should be set to the right hand radius.");
        succeeded = false;
    }
    if (_l_hand_radius < 0 || _l_hand_radius > 0.5) {    
        ROS_ERROR_STREAM("The 'l_hand_radius' value must be in range [0, 0.5] meters. Provided: " << _l_hand_radius);
        succeeded = false;
    }
    if (_r_hand_radius < 0 || _r_hand_radius > 0.5) {    
        ROS_ERROR_STREAM("The 'r_hand_radius' value must be in range [0, 0.5] meters. Provided: " << _r_hand_radius);
        succeeded = false;
    }
    _target_frames.push_back(_l_hand_frame);
    _target_frames.push_back(_r_hand_frame);
    _l_hand_inflated_radius = _l_hand_radius;
    _r_hand_inflated_radius = _r_hand_radius;
    _publish_markers = false;

    dyn_f = boost::bind(&uchile_laser_pipeline::LaserScanSelfFilter::dynamicReconfigureCallback, this, _1, _2);
    dyn_server.setCallback(dyn_f);

    ros::NodeHandle nh("~");
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 0);

    ros::Duration(3).sleep();
    return succeeded;
}

bool uchile_laser_pipeline::LaserScanSelfFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &output_scan) {

    ros::Time target_time = input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment);

    // point centered at required frame
    geometry_msgs::PointStamped link_point;
    link_point.header.stamp = target_time;

    // copy
    output_scan = input_scan;

    std::vector<double> _hand_radius;
    _hand_radius.push_back(_l_hand_inflated_radius);
    _hand_radius.push_back(_r_hand_inflated_radius);

    int marker_id = 0;
    std::string error_msg;
    std::vector<std::string>::const_iterator it;
    std::vector<double>::const_iterator r_it;
    visualization_msgs::MarkerArray marker_msg;
    for(it = _target_frames.begin(), r_it = _hand_radius.begin(); it != _target_frames.end(); ++it, ++r_it) {

        std::string target_frame = *it;
        float radius = (float)*r_it;

        link_point.header.frame_id = target_frame;

        bool success = tf_listener.waitForTransform(
            target_frame,
            input_scan.header.frame_id,
            target_time,
            ros::Duration(1.0),
            ros::Duration(0.01),
            &error_msg
        );
        if(!success){
            ROS_WARN_STREAM_DELAYED_THROTTLE(2.0 , "Could not get transform for frame '"
                    << target_frame.c_str()
                    << "'. Ignoring this frame. TF Error: "
                    << error_msg.c_str()
            );
            continue;
        }

        geometry_msgs::PointStamped link_point_transformed;
        try {
            tf_listener.transformPoint(input_scan.header.frame_id, link_point, link_point_transformed);
        }
        catch(tf::TransformException& ex) {
            ROS_WARN_STREAM_DELAYED_THROTTLE(2.0, "Ignoring frame: '"
                    << target_frame.c_str()
                    << "'. Transform Exception: " << ex.what());
            continue;
        }

        // link does not collides with the laser scan plane
        if (fabsf((float)link_point_transformed.point.z) > radius) {
            continue;
        }

        // angular limits
        float theta_min;
        float theta_max;
        getAngularLimits(
                (float) link_point_transformed.point.x,
                (float) link_point_transformed.point.y,
                (float) link_point_transformed.point.z,
                radius, theta_min, theta_max);

        // remove ranges which collide with our projection
        float cur_angle;
        std::vector<float>::iterator range_it;
        for (range_it = output_scan.ranges.begin(), cur_angle = output_scan.angle_min;
             range_it != output_scan.ranges.end();
             ++range_it, cur_angle += output_scan.angle_increment) {

            if (theta_min < cur_angle  && cur_angle < theta_max) {
                *range_it = output_scan.range_max + 1;
            }
        }

        marker_msg.markers.push_back(genMarker(target_frame, radius, marker_id));
        marker_id++;
    }
    marker_pub.publish(marker_msg);
    return true;
}

visualization_msgs::Marker uchile_laser_pipeline::LaserScanSelfFilter::genMarker(const std::string& target_frame, double radius, int marker_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(0.5);
    marker.ns = "inflated_hand";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2.0;
    marker.scale.y = radius * 2.0;
    marker.scale.z = radius * 2.0;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
}

/**
 * The algorithm works as follows:
 * Each considered joint/frame on 'target_frames' is inflated and
 * considered as a sphere centered at (0,0,0) with radius given by
 * 'hand_radius'.
 *
 * Then, the sphere center is transformed into the laser scan frame,
 * centered at (x,y,z). If the sphere is too high or too low, then
 * there can't be a collision.
 *
 * The goal is to remove/invalidate laser points which collides with
 * our sphere, so we project it into the laser X-Y plane, resulting
 * in a circle on (xm, ym), z=0 and radius rm.
 *
 * Then we compute the min/max angles of the scan sweep to be removed,
 * by finding the tangent lines to the projection which intersects
 * the laser center at (0,0)
 */
void uchile_laser_pipeline::LaserScanSelfFilter::getAngularLimits(
        float x, float y, float z, float r, float &theta_min, float &theta_max) {

    // sphere projection into the laser plane
    float xm = x;
    float ym = y;
    float rm = sqrtf(r*r - z*z);

    // angle from laser to projection center
    float theta_m = atan2f(ym, xm);

    // distance to the laser
    float d = sqrtf(xm*xm + ym*ym);
    // TODO: check whether d ~< rm, this means the link is colliding to the laser

    // length of tangent segment to the laser
    float td = sqrtf(d*d - rm*rm);
    td = fmaxf(td, 0.01); // min 1 [cm].

    // angle/2 between the 2 tangents
    float delta_theta = atanf(rm/td);

    // angular limits
    theta_min = theta_m - delta_theta;
    theta_max = theta_m + delta_theta;
}
