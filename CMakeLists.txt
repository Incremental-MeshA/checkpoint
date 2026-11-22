cmake_minimum_required(VERSION 3.16)
project(mesh_bench LANGUAGES CXX)
add_executable(planner src/main.cpp src/graph.cpp src/batch.cpp src/changed_edges.cpp
    algorithms/astar.cpp algorithms/dstar_lite.cpp algorithms/mpgaa.cpp)
target_compile_features(planner PRIVATE cxx_std_17)
if(MINGW)
    get_filename_component(mingw_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
    file(GLOB runtime_dlls "${mingw_bin}/libstdc++-6.dll" "${mingw_bin}/libgcc_s_*-1.dll" "${mingw_bin}/libwinpthread-1.dll")
    foreach(dll IN LISTS runtime_dlls)
        add_custom_command(TARGET planner POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${dll}" "$<TARGET_FILE_DIR:planner>" VERBATIM)
    endforeach()
endif()
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/build-$<CONFIG>.txt"
    CONTENT "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\n${CMAKE_CXX_COMPILER}\n$<CONFIG> ${CMAKE_CXX_FLAGS} $<$<CONFIG:Release>:${CMAKE_CXX_FLAGS_RELEASE}>\n")

