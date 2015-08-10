FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/vicon_bridge/msg/__init__.py"
  "../src/vicon_bridge/msg/_Marker.py"
  "../src/vicon_bridge/msg/_TfDistortInfo.py"
  "../src/vicon_bridge/msg/_Markers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
