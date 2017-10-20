#include <iostream>

#include <ros/ros.h>

#include "srrg_semantic_extraction_trigger_ros/semantic_extraction_trigger_node.h"

using namespace std;
using namespace srrg_core;
using namespace srrg_semantic_extraction_trigger;
using namespace srrg_semantic_extraction_trigger_ros;

int main(int argc, char** argv){

    ros::init(argc,argv,"srrg_semantic_extraction_node");

    ROS_INFO("Starting semantic extraction trigger!!!");

    SemanticExtractionTriggerNode trigger_node;

    ros::spin();

    return 0;
}
