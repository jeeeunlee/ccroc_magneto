FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (goldfarb ${headers} ${sources})

