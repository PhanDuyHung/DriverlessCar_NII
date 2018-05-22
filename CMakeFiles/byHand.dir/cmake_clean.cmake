file(REMOVE_RECURSE
  "bin/Release/byHand.pdb"
  "bin/Release/byHand"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/byHand.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
