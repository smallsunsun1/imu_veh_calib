add_subdirectory(csm)
add_subdirectory(data_selection)
add_subdirectory(egsl)

add_library(solver utils.cpp solver.cpp solveQyx.cpp)
target_link_directories(solver PUBLIC /usr/include)
target_link_libraries(solver PUBLIC csm data_selection egsl ceres)
add_executable(main main_node.cpp)
target_link_libraries(main PUBLIC solver)
