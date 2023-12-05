add_executable(goma_main goma_main.cc image.cc two_views.cc)
set_target_properties(goma_main PROPERTIES OUTPUT_NAME goma)

target_link_libraries(goma_main ${COLMAP_LIBRARIES} mpfr goma)
