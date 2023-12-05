file(GLOB_RECURSE srcs "${PROJECT_SOURCE_DIR}/goma/*/*.cc")

add_library(goma ${srcs})
target_include_directories(goma PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(goma ${COLMAP_LIBRARIES} mpfr)
