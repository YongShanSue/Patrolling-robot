FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmtypes_lcm2ros_jar"
  "lcmtypes_lcm2ros.jar"
  "../lcmtypes/java/exlcm/example_t.class"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmtypes_lcm2ros_jar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
