#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "srrg_semantic_extraction_trigger/semantic_extraction_trigger.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

namespace srrg_semantic_extraction_trigger_ros{

class SemanticExtractionTriggerNode{
public:
    SemanticExtractionTriggerNode(srrg_semantic_extraction_trigger::SemanticExtractionTrigger* trigger_=0);

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg);
    void occupancyGridSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg);

private:
    ros::NodeHandle _nh;
    float _raw_depth_scale;
    float _resolution;
    float _robot_climb_step;
    float _robot_height;
    bool _got_info;
    bool _got_map;

    cv::Mat _map_image;

    std::string _camera_info_topic;
    std::string _depth_image_topic;
    std::string _occupancy_grid_topic;

    std::string _base_frame_id;

    tf::TransformListener _robot_listener;
    tf::StampedTransform _robot_tf;

    ros::Subscriber _camera_info_subscriber;
    ros::Subscriber _depth_image_subscriber;
    ros::Subscriber _occupancy_grid_subscriber;

    srrg_semantic_extraction_trigger::SemanticExtractionTrigger* _trigger;

    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);

    tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T);

};

}
