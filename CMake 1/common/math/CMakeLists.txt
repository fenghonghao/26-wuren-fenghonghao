find_package(OpenCV REQUIRED)

aux_source_directory(src Math_SRC_LIST)

add_library(Math ${Math_SRC_LIST})

target_include_directories(Math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(Math PUBLIC ${OpenCV_LIBS})