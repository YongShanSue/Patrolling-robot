FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/clean_test_results_lcm2ros"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean_test_results_lcm2ros.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
