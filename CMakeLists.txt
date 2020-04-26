cmake_minimum_required(VERSION 3.5)
project(sick_safetyscanners)

## Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wpedantic")
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#SET(Boost_USE_STATIC_LIBS ON)           # link statically
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)  # or, link dynamically
find_package(Boost REQUIRED COMPONENTS system thread log log_setup)

set(
  msg_files
  "msg/ApplicationDataMsg.msg"
  "msg/ApplicationInputsMsg.msg"
  "msg/ApplicationOutputsMsg.msg"
  "msg/DataHeaderMsg.msg"
  "msg/DerivedValuesMsg.msg"
  "msg/ExtendedLaserScanMsg.msg"
  "msg/FieldMsg.msg"
  "msg/GeneralSystemStateMsg.msg"
  "msg/IntrusionDataMsg.msg"
  "msg/IntrusionDatumMsg.msg"
  "msg/MeasurementDataMsg.msg"
  "msg/MonitoringCaseMsg.msg"
  "msg/OutputPathsMsg.msg"
  "msg/RawMicroScanDataMsg.msg"
  "msg/ScanPointMsg.msg"

)

set(
  srv_files
  "srv/FieldData.srv"
)

set(action_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs std_msgs sensor_msgs
  ADD_LINTER_TESTS
)



###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
#  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/SickSafetyscanners.cpp
  src/cola2/ApplicationNameVariableCommand.cpp
  src/cola2/ChangeCommSettingsCommand.cpp
  src/cola2/CloseSession.cpp
  src/cola2/Cola2Session.cpp
  src/cola2/Command.cpp
  src/cola2/ConfigMetadataVariableCommand.cpp
  src/cola2/CreateSession.cpp
  src/cola2/DeviceNameVariableCommand.cpp
  src/cola2/DeviceStatusVariableCommand.cpp
  src/cola2/FieldGeometryVariableCommand.cpp
  src/cola2/FieldHeaderVariableCommand.cpp
  src/cola2/FieldSetsVariableCommand.cpp
  src/cola2/FindMeCommand.cpp
  src/cola2/FirmwareVersionVariableCommand.cpp
  src/cola2/MeasurementCurrentConfigVariableCommand.cpp
  src/cola2/MeasurementPersistentConfigVariableCommand.cpp
  src/cola2/MethodCommand.cpp
  src/cola2/MonitoringCaseTableHeaderVariableCommand.cpp
  src/cola2/MonitoringCaseVariableCommand.cpp
  src/cola2/OrderNumberVariableCommand.cpp
  src/cola2/ProjectNameVariableCommand.cpp
  src/cola2/RequiredUserActionVariableCommand.cpp
  src/cola2/SerialNumberVariableCommand.cpp
  src/cola2/StatusOverviewVariableCommand.cpp
  src/cola2/TypeCodeVariableCommand.cpp
  src/cola2/UserNameVariableCommand.cpp
  src/cola2/VariableCommand.cpp
  src/communication/AsyncTCPClient.cpp
  src/communication/AsyncUDPClient.cpp
  src/data_processing/ParseApplicationData.cpp
  src/data_processing/ParseApplicationNameData.cpp
  src/data_processing/ParseConfigMetadata.cpp
  src/data_processing/ParseData.cpp
  src/data_processing/ParseDataHeader.cpp
  src/data_processing/ParseDatagramHeader.cpp
  src/data_processing/ParseDerivedValues.cpp
  src/data_processing/ParseDeviceName.cpp
  src/data_processing/ParseDeviceStatus.cpp
  src/data_processing/ParseFieldGeometryData.cpp
  src/data_processing/ParseFieldHeaderData.cpp
  src/data_processing/ParseFieldSetsData.cpp
  src/data_processing/ParseFirmwareVersion.cpp
  src/data_processing/ParseGeneralSystemState.cpp
  src/data_processing/ParseIntrusionData.cpp
  src/data_processing/ParseMeasurementCurrentConfigData.cpp
  src/data_processing/ParseMeasurementData.cpp
  src/data_processing/ParseMeasurementPersistentConfigData.cpp
  src/data_processing/ParseMonitoringCaseData.cpp
  src/data_processing/ParseOrderNumber.cpp
  src/data_processing/ParseProjectName.cpp
  src/data_processing/ParseRequiredUserAction.cpp
  src/data_processing/ParseSerialNumber.cpp
  src/data_processing/ParseStatusOverview.cpp
  src/data_processing/ParseTCPPacket.cpp
  src/data_processing/ParseTypeCodeData.cpp
  src/data_processing/ParseUserNameData.cpp
  src/data_processing/TCPPacketMerger.cpp
  src/data_processing/UDPPacketMerger.cpp
  src/datastructure/ApplicationData.cpp
  src/datastructure/ApplicationInputs.cpp
  src/datastructure/ApplicationName.cpp
  src/datastructure/ApplicationOutputs.cpp
  src/datastructure/CommSettings.cpp
  src/datastructure/ConfigData.cpp
  src/datastructure/ConfigMetadata.cpp
  src/datastructure/Data.cpp
  src/datastructure/DatagramHeader.cpp
  src/datastructure/DataHeader.cpp
  src/datastructure/DerivedValues.cpp
  src/datastructure/DeviceName.cpp
  src/datastructure/DeviceStatus.cpp
  src/datastructure/FieldData.cpp
  src/datastructure/FieldSets.cpp
  src/datastructure/FirmwareVersion.cpp
  src/datastructure/GeneralSystemState.cpp
  src/datastructure/IntrusionData.cpp
  src/datastructure/IntrusionDatum.cpp
  src/datastructure/LatestTelegram.cpp
  src/datastructure/MeasurementData.cpp
  src/datastructure/MonitoringCaseData.cpp
  src/datastructure/OrderNumber.cpp
  src/datastructure/PacketBuffer.cpp
  src/datastructure/ParsedPacketBuffer.cpp
  src/datastructure/ProjectName.cpp
  src/datastructure/RequiredUserAction.cpp
  src/datastructure/ScanPoint.cpp
  src/datastructure/SerialNumber.cpp
  src/datastructure/StatusOverview.cpp
  src/datastructure/TypeCode.cpp
  src/datastructure/UserName.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}_node
  nodes/${PROJECT_NAME}_node.cpp
  src/SickSafetyscannersRos.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ament_cmake
  ament_index_cpp
  geometry_msgs
  std_msgs
  sensor_msgs
  diagnostic_updater
  diagnostic_msgs
)
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_core
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Mark other files for installation
install(
  DIRECTORY docs launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_ros_package_template.cpp
#  test/AlgorithmTest.cpp)
#  ament_lint_auto_find_test_dependencies()
#endif()

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#  endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
