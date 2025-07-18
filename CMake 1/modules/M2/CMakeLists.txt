find_package(OpenCV REQUIRED)

aux_source_directory(src M2_SRC_LIST)

add_library(M2 ${M2_SRC_LIST})

target_include_directories(M2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(M2 PUBLIC ${kalman_INCLUDE_DIRS})

target_link_libraries(M2 PUBLIC A1 A2 ${OpenCV_LIBS})