FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/std_msgs_generate_messages_lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/std_msgs_generate_messages_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
