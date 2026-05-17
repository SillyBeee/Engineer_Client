cmake_minimum_required(VERSION 3.16)
project(URDF_MQTT_Viewer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

list(PREPEND CMAKE_PREFIX_PATH /opt/ros/jazzy)

find_package(OpenCV REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OGRE REQUIRED)
find_package(assimp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Threads REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(RUCKIG_LIBRARY ruckig
    PATHS /opt/ros/jazzy/lib/x86_64-linux-gnu
    REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OGRE_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    /opt/ros/jazzy/include
)

set(SOURCES
    src/main.cpp
    src/driver_mqtt.cpp
    src/driver_urdf_loader.cpp
    src/driver_urdf_renderer.cpp
    src/trajectory_planner.cpp
    src/logger.cpp
)

add_executable(urdf_mqtt_viewer ${SOURCES})

target_link_libraries(urdf_mqtt_viewer PRIVATE
    ${OpenCV_LIBS}
    PahoMqttCpp::paho-mqttpp3
    spdlog::spdlog
    ${OGRE_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${urdfdom_LIBRARIES}
    Threads::Threads
    ${OMPL_LIBRARIES}
    ${RUCKIG_LIBRARY}
    Eigen3::Eigen
)

add_executable(demo_plan
    src/demo_plan.cpp
    src/driver_urdf_loader.cpp
    src/driver_urdf_renderer.cpp
    src/trajectory_planner.cpp
    src/logger.cpp
)
target_link_libraries(demo_plan PRIVATE
    ${OpenCV_LIBS}
    spdlog::spdlog
    ${OGRE_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${urdfdom_LIBRARIES}
    Threads::Threads
    ${OMPL_LIBRARIES}
    ${RUCKIG_LIBRARY}
    Eigen3::Eigen
)
target_compile_definitions(demo_plan PRIVATE
    HAVE_OPENCV
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_include_directories(demo_plan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OGRE_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    /opt/ros/jazzy/include
)

target_compile_definitions(urdf_mqtt_viewer PRIVATE
    HAVE_OPENCV
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
