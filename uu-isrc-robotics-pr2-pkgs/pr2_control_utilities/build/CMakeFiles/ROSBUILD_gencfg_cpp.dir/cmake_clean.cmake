FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/pr2_control_utilities/planningConfig.h"
  "../docs/planningConfig.dox"
  "../docs/planningConfig-usage.dox"
  "../src/pr2_control_utilities/cfg/planningConfig.py"
  "../docs/planningConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
