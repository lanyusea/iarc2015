FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/vicon_bridge/tf_distortConfig.h"
  "../docs/tf_distortConfig.dox"
  "../docs/tf_distortConfig-usage.dox"
  "../src/vicon_bridge/cfg/tf_distortConfig.py"
  "../docs/tf_distortConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
