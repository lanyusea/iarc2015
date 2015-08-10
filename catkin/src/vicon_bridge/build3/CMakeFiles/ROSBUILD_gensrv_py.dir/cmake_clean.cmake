FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/vicon_bridge/srv/__init__.py"
  "../src/vicon_bridge/srv/_viconGrabPose.py"
  "../src/vicon_bridge/srv/_viconCalibrateSegment.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
