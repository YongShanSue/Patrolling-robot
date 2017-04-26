FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmtypes_kinect_jar"
  "lcmtypes_kinect.jar"
  "../lcmtypes/java/kinect/segmentlist_t.class"
  "../lcmtypes/java/kinect/sensor_status_t.class"
  "../lcmtypes/java/kinect/frame_msg_t.class"
  "../lcmtypes/java/kinect/skeleton_msg_t.class"
  "../lcmtypes/java/kinect/depth_msg_t.class"
  "../lcmtypes/java/kinect/image_msg_t.class"
  "../lcmtypes/java/kinect/point3d_t.class"
  "../lcmtypes/java/kinect/pointcloud_t.class"
  "../lcmtypes/java/kinect/projected_point2d_t.class"
  "../lcmtypes/java/kinect/point2d_t.class"
  "../lcmtypes/java/kinect/cmd_msg_t.class"
  "../lcmtypes/java/kinect/link_msg_t.class"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmtypes_kinect_jar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
