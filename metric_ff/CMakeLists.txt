cmake_minimum_required(VERSION 2.8.3)
project(metric_ff)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES metric_ff
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# get the ${PYTHON_EXECUTABLE}
INCLUDE(FindPythonInterp)

#determine if we are using dist-packages on a Debian derviate or site-packages for most other distros
execute_process (
        COMMAND "${PYTHON_EXECUTABLE}" -c "from distutils.sysconfig import get_python_lib; site=get_python_lib(); site=site[site.rindex('/')+1:] ;print site"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

#get the python version for the correct path
execute_process (
        COMMAND "${PYTHON_EXECUTABLE}" -c "from distutils.sysconfig import get_python_version; print get_python_version()"
        OUTPUT_VARIABLE PYTHON_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

# use distutils to build the extension module
SET(PYTHON_EXECUTABLE python2)
add_custom_target(ff.so ALL
                COMMAND ${PYTHON_EXECUTABLE} setup.py build
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

# move the built extension module into the correct python site packages location
add_custom_target(ff_python_install ALL
                COMMAND ${PYTHON_EXECUTABLE} setup.py install --install-lib=${CATKIN_DEVEL_PREFIX}/lib/python${PYTHON_VERSION}/${PYTHON_SITE_PACKAGES} 
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

# copy built extension module library into lib folder for dynamic loading
add_custom_target(ff_ffp_install ALL
                COMMAND cp ${CATKIN_DEVEL_PREFIX}/lib/python${PYTHON_VERSION}/${PYTHON_SITE_PACKAGES}/ff.so ${CATKIN_DEVEL_PREFIX}/lib
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

# make sure the above commands run in the correct order
add_dependencies(ff.so ff_python_install)
add_dependencies(ff_ffp_install ff_python_install)
add_dependencies(ff.so ff_ffp_install)

## Declare a C++ executable
# creating an additional executable for testing
add_executable(${PROJECT_NAME} 
    main.c
    ff.h
    memory.c
    memory.h
    output.c
    output.h
    parse.c
    parse.h
    expressions.c
    expressions.h
    inst_pre.c
    inst_pre.h
    inst_easy.c
    inst_easy.h
    inst_hard.c
    inst_hard.h
    inst_final.c
    inst_final.h
    relax.c
    relax.h
    search.c
    search.h
    scan-fct_pddl.tab.c
    scan-ops_pddl.tab.c
)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES build)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS metric_ff_v2_1 metric_ff_v2_1_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_metric_ff_v2_1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
