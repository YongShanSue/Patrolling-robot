FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmtypes_wearnavi_jar"
  "lcmtypes_wearnavi.jar"
  "../lcmtypes/java/obstacle/tts_t.class"
  "../lcmtypes/java/obstacle/haptic_array_t.class"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmtypes_wearnavi_jar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
