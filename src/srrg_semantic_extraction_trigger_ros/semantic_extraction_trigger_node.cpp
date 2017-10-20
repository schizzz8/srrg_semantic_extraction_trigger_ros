#include "semantic_extraction_trigger_node.h"

namespace srrg_semantic_extraction_trigger_ros{

using namespace std;
using namespace srrg_core;
using namespace srrg_semantic_extraction_trigger;

SemanticExtractionTriggerNode::SemanticExtractionTriggerNode(SemanticExtractionTrigger *trigger_){
    if(!trigger_)
        _trigger = new SemanticExtractionTrigger();
    else
        _trigger = trigger_;

    _nh = ros::NodeHandle("~");

    _nh.param("raw_depth_scale",_raw_depth_scale,0.001f);
    _trigger->cloudGenerator()->setRawDepthScale(_raw_depth_scale);

    _nh.param("resolution",_resolution,0.05f);
    _trigger->structureAnalyzer()->setResolution(_resolution);

    _nh.param("robot_climb_step",_robot_climb_step,0.05f);
    _trigger->structureAnalyzer()->setRobotClimbStep(_robot_climb_step);

    _nh.param("robot_height",_robot_height,0.5f);
    _trigger->structureAnalyzer()->setRobotHeight(_robot_height);

    _nh.param<string>("camera_info_topic",_camera_info_topic,"/camera/depth/camera_info");
    _nh.param<string>("depth_image_topic",_depth_image_topic,"/camera/depth/image_raw");
    _nh.param<string>("occupancy_grid_topic",_occupancy_grid_topic,"/map");

    _nh.param<string>("base_frame_id",_base_frame_id,"/base_link");

    _camera_info_subscriber = _nh.subscribe(_camera_info_topic,
                                            1000,
                                            &SemanticExtractionTriggerNode::cameraInfoCallback,
                                            this);

    _depth_image_subscriber = _nh.subscribe(_depth_image_topic,
                                            1000,
                                            &SemanticExtractionTriggerNode::depthImageCallback,
                                            this);

    _occupancy_grid_subscriber = _nh.subscribe(_occupancy_grid_topic,
                                               1000,
                                               &SemanticExtractionTriggerNode::occupancyGridSubscriber,
                                               this);

    _got_info = false;
    _got_map = false;
}

void SemanticExtractionTriggerNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg){
    sensor_msgs::CameraInfo camerainfo;
    camerainfo.K = camera_info_msg->K;
    Eigen::Matrix3f K;
    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];

    _trigger->setK(K);

    ROS_INFO_STREAM("Wait for camera tf " << _base_frame_id << " - " << camera_info_msg->header.frame_id << " ...");
    tf::TransformListener camera_listener;
    tf::StampedTransform camera_tf;
    try {
        camera_listener.waitForTransform(_base_frame_id, camera_info_msg->header.frame_id, ros::Time(0), ros::Duration(10.0) );
        camera_listener.lookupTransform(_base_frame_id, camera_info_msg->header.frame_id, ros::Time(0), camera_tf);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    _trigger->setSensorOffset(tfTransform2eigen(camera_tf));
    ROS_INFO("got camera transform");

    _camera_info_subscriber.shutdown();
    _got_info=true;
    
}

void SemanticExtractionTriggerNode::depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_image_msg){
    if(!_got_info || !_got_map)
        return;

    cv_bridge::CvImageConstPtr depth_image;
    
    try {
        depth_image =  cv_bridge::toCvShare(depth_image_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", depth_image_msg->encoding.c_str());
    }

    _trigger->setDepthImage(depth_image->image);
    _trigger->generateCloud();
    _trigger->analyzeStructure();
    _trigger->extractClusters();

    try {
        _robot_listener.waitForTransform("/map", _base_frame_id, ros::Time(0), ros::Duration(10.0) );
        _robot_listener.lookupTransform("/map", _base_frame_id, ros::Time(0), _robot_tf);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    _trigger->processClusters(_map_image,tfTransform2eigen(_robot_tf));

    _depth_image_subscriber.shutdown();
    
}

void SemanticExtractionTriggerNode::occupancyGridSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg){
    int width = occupancy_grid_msg->info.width;
    int height = occupancy_grid_msg->info.height;

    cv::Mat temp_image = cv::Mat(height, width, CV_8U);

    _trigger->setMapResolution(occupancy_grid_msg->info.resolution);
    _trigger->setMapOrigin(Eigen::Vector3f(occupancy_grid_msg->info.origin.position.x,
                                           occupancy_grid_msg->info.origin.position.y,
                                           0));

    ROS_INFO("Occupancy grid received.");

    for (int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
        for (int j = 0; j < width; j++)
            switch (occupancy_grid_msg->data[i_rev*width + j]) {
            default:
            case -1:
                temp_image.data[i*width + j] = 150;
                break;
            case 0:
                temp_image.data[i*width + j] = 255;
                break;
            case 100:
                temp_image.data[i*width + j] = 0;
                break;
            }
    ROS_INFO("Image extracted from occupancy grid.");

    cv::cvtColor(temp_image,_map_image,CV_GRAY2RGB);
    _occupancy_grid_subscriber.shutdown();
    _got_map=true;
}


Eigen::Isometry3f SemanticExtractionTriggerNode::tfTransform2eigen(const tf::Transform& p){
    Eigen::Isometry3f iso;
    iso.translation().x()=p.getOrigin().x();
    iso.translation().y()=p.getOrigin().y();
    iso.translation().z()=p.getOrigin().z();
    Eigen::Quaternionf q;
    tf::Quaternion tq = p.getRotation();
    q.x()= tq.x();
    q.y()= tq.y();
    q.z()= tq.z();
    q.w()= tq.w();
    iso.linear()=q.toRotationMatrix();
    return iso;
}

tf::Transform SemanticExtractionTriggerNode::eigen2tfTransform(const Eigen::Isometry3f& T){
    Eigen::Quaternionf q(T.linear());
    Eigen::Vector3f t=T.translation();
    tf::Transform tft;
    tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    return tft;
}

}
