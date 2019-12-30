cmake_minimum_required(VERSION 2.8.3)
project(car_demo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${catkin_LIBRARIES}
)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  mirvehicle_msgs
  
)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(gazebo 9 REQUIRED)
find_package(ignition-msgs1 REQUIRED)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros mirvehicle_msgs 
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})



add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})



add_library(ActorCollisionsPlugin SHARED plugins/ActorCollisionsPlugin.cc)
target_link_libraries(ActorCollisionsPlugin ${GAZEBO_LIBRARIES})


add_library(TrafficLightsGUIPlugin SHARED plugins/TrafficLightsGUIPlugin.cc)
target_link_libraries(TrafficLightsGUIPlugin ${guiSrc}
			${IGNITION-MSGS_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                       ${Qt5Widgets_LIBRARIES} 
                        ${GAZEBO_LIBRARIES})
add_dependencies(TrafficLightsGUIPlugin ${catkin_EXPORTED_TARGETS})

add_library(LensFlareVisualPlugin SHARED plugins/LensFlareVisualPlugin.cc)
target_link_libraries(TrafficLightsGUIPlugin ${guiSrc}
			${IGNITION-MSGS_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                       ${Qt5Widgets_LIBRARIES} 
                        ${GAZEBO_LIBRARIES})
add_dependencies(LensFlareVisualPlugin ${catkin_EXPORTED_TARGETS})

add_library(BloomVisualPlugin SHARED plugins/BloomVisualPlugin.cc)
target_link_libraries(TrafficLightsGUIPlugin ${guiSrc}
			${IGNITION-MSGS_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                       ${Qt5Widgets_LIBRARIES} 
                        ${GAZEBO_LIBRARIES})
add_dependencies(BloomVisualPlugin ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS nodes/joystick_translator_joystick
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS PriusHybridPlugin TrafficLightsGUIPlugin BloomVisualPlugin LensFlareVisualPlugin 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

