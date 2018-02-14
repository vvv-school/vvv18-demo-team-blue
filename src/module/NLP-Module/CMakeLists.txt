# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(NLP-Module)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
 
# helpers defining certain macros (e.g. "yarp_install")
include(ICUBcontribHelpers)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# building 
include_directories(${YARP_INCLUDE_DIRS})
add_executable(NLP-Module main.cpp)
target_link_libraries(NLP-Module ${YARP_LIBRARIES})
install(TARGETS NLP-Module DESTINATION bin)

# icubcontrib_add_uninstall_target()



