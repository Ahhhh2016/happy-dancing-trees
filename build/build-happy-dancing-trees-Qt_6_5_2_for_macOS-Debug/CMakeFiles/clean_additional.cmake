# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/projects_raster_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/projects_raster_autogen.dir/ParseCache.txt"
  "projects_raster_autogen"
  )
endif()
