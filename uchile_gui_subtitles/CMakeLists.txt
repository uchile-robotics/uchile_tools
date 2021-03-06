##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(uchile_gui_subtitles)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    std_msgs
    uchile_msgs
    uchile_srvs
)
include_directories(${catkin_INCLUDE_DIRS})

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
    CATKIN_DEPENDS
        std_msgs
        uchile_msgs
        uchile_srvs
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.hpp)

qt4_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt4_wrap_cpp(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(gui_subtitles ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(gui_subtitles ${catkin_EXPORTED_TARGETS})
target_link_libraries(gui_subtitles ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS gui_subtitles RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
