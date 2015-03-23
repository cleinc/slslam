cmake_minimum_required(VERSION 2.8)
enable_testing()

# make release version
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_VERBOSE_MAKEFILE yes)

# user definitions, BRANCH_FACTOR: tree's branching factor, LEVEL: tree's number
# of hierarchy, DESC_DIM: descriptor dimension ex) SIFT=128, SURF=64 or 128
#add_definitions(-DBRANCH_FACTOR=40 -DLEVEL=3 -DDESC_DIM=72)
#add_definitions(-DBRANCH_FACTOR=50 -DLEVEL=3 -DDESC_DIM=72)

find_package(Ceres REQUIRED PATHS "/usr/local/share/Ceres")
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV 2 REQUIRED PATHS "/usr/local/opencv2.4/share/OpenCV")
include_directories(${OpenCV_INCLUDE_DIRS})

enable_testing()
if (APPLE)
  add_definitions(-DAPPLE)
endif (APPLE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${PROJECT_SOURCE_DIR}/cmake_modules/")

	# #########################################################
	# # FIND GLUT
	# #########################################################
	# find_package(GLUT REQUIRED)
	# include_directories(${GLUT_INCLUDE_DIRS})
	# link_directories(${GLUT_LIBS})
	# add_definitions(${GLUT_DEFINITIONS})
	# if(NOT GLUT_FOUND)
	#     message(ERROR " GLUT not found!")
	# endif(NOT GLUT_FOUND)
	# 
	# #########################################################
	# # FIND OPENGL
	# #########################################################
	# find_package(OpenGL REQUIRED)
	# include_directories(${OpenGL_INCLUDE_DIRS})
	# link_directories(${OpenGL_LIBS})
	# add_definitions(${OpenGL_DEFINITIONS})
	# if(NOT OPENGL_FOUND)
	#     message(ERROR " OPENGL not found!")
	# endif(NOT OPENGL_FOUND)

#find_package(GoogleLibs REQUIRED)
# Default locations to search for on various platforms.
LIST(APPEND SEARCH_LIBS /usr/lib)
LIST(APPEND SEARCH_LIBS /usr/local/lib)
LIST(APPEND SEARCH_LIBS /usr/local/opencv2.4/lib)
LIST(APPEND SEARCH_LIBS /opt/local/lib)

LIST(APPEND SEARCH_HEADERS /usr/include)
LIST(APPEND SEARCH_HEADERS /usr/local/include)
LIST(APPEND SEARCH_HEADERS /usr/local/opencv2.4/include)
LIST(APPEND SEARCH_HEADERS /usr/local/include/eigen3)

# Google libraries
message("-- Check for Google Log")
find_library(GLOG_LIB NAMES glog PATHS ${SEARCH_LIBS})
find_path(GLOG_INCLUDE NAMES glog/logging.h PATHS ${SEARCH_HEADERS})

message("-- Check for Google Flags")
find_library(GFLAGS_LIB NAMES gflags PATHS ${SEARCH_LIBS})
find_path(GFLAGS_INCLUDE NAMES gflags/gflags.h PATHS ${SEARCH_HEADERS})

find_path(EIGEN_INCLUDE NAMES Eigen/Core PATHS ${SEARCH_HEADERS})

set(GOOGLE_LIBRARIES ${GFLAGS_LIB} ${GLOG_LIB})

include_directories(${sl_slam_SOURCE_DIR}/src)
include_directories(${GLOG_INCLUDE})
include_directories(${GFLAGS_INCLUDE})
include_directories(${CERES_INCLUDE})
include_directories(${EIGEN_INCLUDE})
include_directories(.)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_executable(slslam_it3f cplot.cpp gc.cpp hrandom.cpp lba_problem.cpp main.cpp 
	po_problem.cpp slam.cpp)
set_target_properties (slslam_it3f PROPERTIES COMPILE_DEFINITIONS "IT3F")
target_link_libraries(slslam_it3f ${CERES_LIBRARIES} ${GOOGLE_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} -lGL -lGLU -lglfw )

add_executable(slslam_olympic4f cplot.cpp gc.cpp hrandom.cpp lba_problem.cpp main.cpp 
	po_problem.cpp slam.cpp)
set_target_properties (slslam_olympic4f PROPERTIES COMPILE_DEFINITIONS "OLYMPIC4F")
target_link_libraries(slslam_olympic4f ${CERES_LIBRARIES} ${GOOGLE_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} -lGL -lGLU -lglfw )

add_executable(slslam_myungdong cplot.cpp gc.cpp hrandom.cpp lba_problem.cpp main.cpp 
	po_problem.cpp slam.cpp)
set_target_properties (slslam_myungdong PROPERTIES COMPILE_DEFINITIONS "MYUNGDONG")
target_link_libraries(slslam_myungdong ${CERES_LIBRARIES} ${GOOGLE_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} -lGL -lGLU -lglfw )
