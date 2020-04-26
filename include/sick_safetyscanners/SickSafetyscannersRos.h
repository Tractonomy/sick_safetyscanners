// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file SickSafetyscannersRos.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 * 
 * Conversion to ROS2 by Tractonomy Robotics BV, Kortrijk, Belgium
 * info@tractonomy.com
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
#define SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H


// ROS
//TODO Setup diagnostics updater
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// STD
#include <string>
#include <vector>

// Package
#include "sick_safetyscanners/msg/extended_laser_scan_msg.hpp"
#include "sick_safetyscanners/srv/field_data.hpp"
#include "sick_safetyscanners/msg/output_paths_msg.hpp"
#include "sick_safetyscanners/msg/raw_micro_scan_data_msg.hpp"
#include "sick_safetyscanners/SickSafetyscanners.h"
#include "sick_safetyscanners/datastructure/CommSettings.h"
#include "sick_safetyscanners/datastructure/FieldData.h"

#include <cmath>

namespace sick {

/*!
 * \brief Converts degrees to radians.
 * \param deg Degrees to convert.
 * \return To radians converted degrees.
 */
inline float degToRad(float deg)
{
  return deg * M_PI / 180.0f;
}

/*!
 * \brief Converts radians to degrees.
 * \param rad Input radians to convert
 * \return To degrees converted radians
 */
inline float radToDeg(float rad)
{
  return rad * 180.0f / M_PI;
}

/*!
 * \brief Converts a skip value into a "publish frequency" value
 * \param skip The number of scans to skip between each measured scan.  For a 25Hz laser, setting
 * 'skip' to 0 makes it publish at 25Hz, 'skip' to 1 makes it publish at 12.5Hz. \return "Publish
 * Frequency" ie. One out of every n_th scan will be published.  1 is publish every scan.  2 is
 * publish at half rate, and so on.
 */
inline uint16_t skipToPublishFrequency(int skip)
{
  return skip + 1;
}

typedef diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan> DiagnosedLaserScanPublisher;

/*!
 * \brief The SickSafetyscannersRos class
 *
 * Main class for the node to handle the ROS interfacing.
 */
class SickSafetyscannersRos : public rclcpp::Node
{
public:
  /*!
   * \brief Constructor of the SickSafetyscannersRos
   *
   * Constructor of the SickSafetyscannersRos, loads all parameters from the parameter server,
   * initialises the dynamic reconfiguration server. Furthermore initialises the ROS Publishers for
   * the different laserscan outputs.
   */
  SickSafetyscannersRos();

  /*!
   * \brief ~SickSafetyscannersRos
   * Destructor if the SickSafetyscanners ROS
   */
  virtual ~SickSafetyscannersRos();

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
  rclcpp::Publisher<sick_safetyscanners::msg::ExtendedLaserScanMsg>::SharedPtr m_extended_laser_scan_publisher;
  rclcpp::Publisher<sick_safetyscanners::msg::RawMicroScanDataMsg>::SharedPtr m_raw_data_publisher;
  rclcpp::Publisher<sick_safetyscanners::msg::OutputPathsMsg>::SharedPtr m_output_path_publisher;

  // Diagnostics
  diagnostic_updater::Updater m_diagnostic_updater;
  std::shared_ptr<DiagnosedLaserScanPublisher> m_diagnosed_laser_scan_publisher;
  sick_safetyscanners::msg::RawMicroScanDataMsg m_last_raw_data;
  void sensorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);

  rclcpp::Service<sick_safetyscanners::srv::FieldData>::SharedPtr m_field_service_server;

  bool m_initialised;

  std::shared_ptr<sick::SickSafetyscanners> m_device;

  sick::datastructure::CommSettings m_communication_settings;

  std::string m_frame_id;
  double m_time_offset;
  double m_range_min;
  double m_range_max;
  double m_frequency_tolerance      = 0.1;
  double m_expected_frequency       = 20.0;
  double m_timestamp_min_acceptable = -1.0;
  double m_timestamp_max_acceptable = 1.0;

  bool m_use_sick_angles;
  float m_angle_offset;
  bool m_use_pers_conf;

  /*!
   * @brief Reads and verifies the ROS parameters.
   * @return True if successful.
   */
  bool readParameters();
 
  /*!
   * \brief Function which is called when a new complete UDP Packet is received
   * \param data, the assortment of all data from the sensor
   */
  void receivedUDPPacket(const datastructure::Data& data);

  bool isInitialised();

  sensor_msgs::msg::LaserScan createLaserScanMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::ExtendedLaserScanMsg
  createExtendedLaserScanMessage(const sick::datastructure::Data& data);
  std::vector<bool>
  getMedianReflectors(const std::vector<sick::datastructure::ScanPoint> scan_points);
  sick_safetyscanners::msg::OutputPathsMsg
  createOutputPathsMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::RawMicroScanDataMsg
  createRawDataMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::DataHeaderMsg createDataHeaderMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::DerivedValuesMsg
  createDerivedValuesMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::GeneralSystemStateMsg
  createGeneralSystemStateMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::MeasurementDataMsg
  createMeasurementDataMessage(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners::msg::ScanPointMsg>
  createScanPointMessageVector(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::IntrusionDataMsg
  createIntrusionDataMessage(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners::msg::IntrusionDatumMsg>
  createIntrusionDatumMessageVector(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::ApplicationDataMsg
  createApplicationDataMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::ApplicationInputsMsg
  createApplicationInputsMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::msg::ApplicationOutputsMsg
  createApplicationOutputsMessage(const sick::datastructure::Data& data);
  void readTypeCodeSettings();
  void readPersistentConfig();

  void getFieldData(const std::shared_ptr<sick_safetyscanners::srv::FieldData::Request> req,
                    std::shared_ptr<sick_safetyscanners::srv::FieldData::Response> res);
};

} // namespace sick

#endif // SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
