FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "CMakeFiles/rospack_gencfg_real"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_gencfg_real.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
