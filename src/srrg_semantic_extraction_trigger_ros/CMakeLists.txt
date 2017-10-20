add_library(srrg_semantic_extraction_trigger_ros_library SHARED
  semantic_extraction_trigger_node.cpp semantic_extraction_trigger_node.h
)

target_link_libraries(srrg_semantic_extraction_trigger_ros_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
