FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o"
  "../lib/libmsvc_bridge.pdb"
  "../lib/libmsvc_bridge.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/msvc_bridge.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
