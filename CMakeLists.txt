cmake_minimum_required(VERSION 3.16)
project(URDF_MQTT_Viewer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OGRE REQUIRED)
find_package(assimp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OGRE_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
)

set(SOURCES
    src/main.cpp
    src/driver_mqtt.cpp
    src/driver_urdf_loader.cpp
    src/driver_urdf_renderer.cpp
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
)

target_compile_definitions(urdf_mqtt_viewer PRIVATE
    HAVE_OPENCV
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
