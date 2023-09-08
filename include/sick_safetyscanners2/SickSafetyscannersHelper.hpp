// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2023, Lowpad, Bleskensgraaf, Netherlands*
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
 * \file SickSafetyscannersHelper.h
 *
 * \author  Rein Appeldoorn <rein.appeldoorn@lowpad.com>
 * \date    2023-09-07
 */
//----------------------------------------------------------------------

#pragma once

#include <sick_safetyscanners_base/SickSafetyscanners.h>

#include <sick_safetyscanners2_interfaces/srv/field_data.hpp>

#include <sick_safetyscanners2/utils/Conversions.h>
#include <sick_safetyscanners2/utils/MessageCreator.h>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace sick {

class SickSafetyscannersHelper
{
public:
  struct Config
  {
    void setupMsgCreator()
    {
      m_msg_creator = std::make_unique<sick::MessageCreator>(
          m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);
    }

    boost::asio::ip::address_v4 m_sensor_ip;
    boost::asio::ip::address_v4 m_interface_ip;
    std::string m_frame_id;
    double m_time_offset = 0.0;
    double m_range_min = 0.0;
    double m_range_max;
    double m_frequency_tolerance      = 0.1;
    double m_expected_frequency       = 20.0;
    double m_timestamp_min_acceptable = -1.0;
    double m_timestamp_max_acceptable = 1.0;
    double m_min_intensities          = 0.0; /*!< min intensities for laser points */
    bool m_use_sick_angles;
    float m_angle_offset = -90.0;
    bool m_use_pers_conf = false;

    sick::types::port_t m_tcp_port = 2122;

    sick::datastructure::CommSettings m_communications_settings;

    std::unique_ptr<sick::MessageCreator> m_msg_creator;
  };

  // Parameters
  static void initializeParameters(rclcpp::Node& node);
  static void loadParameters(rclcpp::Node& node, SickSafetyscannersHelper::Config &config);
  static bool handleParameterUpdate(const rclcpp::Logger& logger, const std::vector<rclcpp::Parameter>& parameters, SickSafetyscannersHelper::Config &config);

  // Methods Triggering COLA2 calls towards the sensor
  static bool getFieldData(const Config& config, sick::AsyncSickSafetyScanner& device, sick_safetyscanners2_interfaces::srv::FieldData::Response& response);
};
} // namespace sick
