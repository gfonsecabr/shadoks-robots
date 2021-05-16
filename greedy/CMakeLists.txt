# The first time you compile it, set CPLEX_ROOT_DIR to /opt/ibm/ILOG/CPLEX_Studio1210,
# the folder containing cplex/ and concert/
#
# Source: https://github.com/alberto-santini/cplex-example

project(greedy)

cmake_minimum_required(VERSION 2.8.10)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -std=c++17 -DIL_STD -m64 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX   "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)
find_package(Cplex)
include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
set(SOURCE_FILES
	src/main.cpp
        src/instance.h             src/instance.cpp
        src/solution.h             src/solution.cpp
        src/solvergreedy.h         src/solvergreedy.cpp
        src/searcherpaths.h        src/searcherpaths.cpp
        src/searchermoves.h        src/searchermoves.cpp
        src/searcherpoints.h       src/searcherpoints.cpp
        src/searcherboundedpaths.h src/searcherboundedpaths.cpp
        src/finderpoints.h         src/finderpoints.cpp
        src/tools.h

)
add_executable(greedy ${SOURCE_FILES})

# I add the "-static" so that this works in the clusters
# You may need to compile this twice
#target_link_libraries(cgshop2021 PRIVATE dl -static)
#target_link_libraries(cgshop2021 PRIVATE ${CPLEX_LIBRARIES} -static)
target_link_libraries(greedy PRIVATE ${CPLEX_LIBRARIES})
target_link_libraries(greedy PRIVATE dl)

