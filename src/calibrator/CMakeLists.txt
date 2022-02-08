# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set( CMAKE_BUILD_TYPE "Release" ) # release is important, or the g2o is very slow.
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_EXTENSIONS OFF)
# add_definitions(-std=c++11) # for qt creator

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/calibrator/cmake_modules )
# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# The library prefix
set(AT_LIBS apriltags ${OPENCV_LDFLAGS})#apriltag library

find_package( OpenCV REQUIRED)
find_package( Boost REQUIRED
  COMPONENTS
  system
  thread
  )
find_package( PCL QUIET REQUIRED)
find_package( Ceres REQUIRED )
find_package( Sophus REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

find_package( Qt5 REQUIRED COMPONENTS Core Gui Widgets )

find_package( VTK REQUIRED )

add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
#  ${CERES_INCLUDE_DIRS}
  thirdParty
  thirdParty/apriltags-cpp/src
  qt_calibration
  ${VTK_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  )

link_directories(
        ${VTK_LIBRARY_DIRS}
        )

#qt5_wrap_cpp( MOC mainwindows.h)
#qt5_wrap_ui( UIC mainwindows.ui)

file(GLOB UI_SRC
    "cqtopencvviewergl.cpp"
    "data_reader.cpp"
    "image_polygon.cpp"
    "imageviewer.cpp"
    "main.cpp"
    "mainwindow.cpp"
    "pointcloud_polygon.cpp"
    "pointcloudviewer.cpp"
)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CSPARSE_LIBRARY}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
    ${CERES_LIBRARIES}
    ${VTK_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
)
add_executable(lidar_camera_calibrator ${UI_SRC} )
target_link_libraries(lidar_camera_calibrator
              ${THIRD_PARTY_LIBS}
              ${AT_LIBS}
            )

#add_executable(single_tagtest single_tagtest.cpp)
#target_link_libraries(single_tagtest ${AT_LIBS})

# Include the apriltag subdirectories
add_subdirectory(thirdParty/apriltags-cpp)


