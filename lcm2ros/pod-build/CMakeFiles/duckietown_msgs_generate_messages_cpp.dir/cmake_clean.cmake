FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/duckietown_msgs_generate_messages_cpp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/duckietown_msgs_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
