FILE(REMOVE_RECURSE
  "CMakeFiles/hoge.dir/hello.cpp.o"
  "../../../lib/libg2o_hoge.pdb"
  "../../../lib/libg2o_hoge.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/hoge.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
