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

    _camera_info_subscriber = _nh.subscribe("camera_info_topic",
                                            1000,
                                            &SemanticExtractionTriggerNode::cameraInfoCallback,
                                            this);

    _depth_image_subscriber = _nh.subscribe("depth_image_topic",
                                            1000,
                                            &SemanticExtractionTriggerNode::depthImageCallback,
                                            this);

    _occupancy_grid_subscriber = _nh.subscribe("occupancy_grid_topic",
                                               1000,
                                               &SemanticExtractionTriggerNode::occupancyGridSubscriber,
                                               this);

    _got_info = false;
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
}


}
