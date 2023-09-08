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
 * \file SickSafetyscanners.h
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
class SickSafetyscanners {
public:
  /**
   * Sick safety scanners base class that implements the shared functionality
   * between the Node (Ros2) and LifCycle node (LifeCycle)
   *
   * @param logger Node log instance
   */
  explicit SickSafetyscanners(const rclcpp::Logger &logger);

protected:
  const rclcpp::Logger &m_logger;

  /**
   * Sick safety scanner configuration
   */
  struct Config {
    void setupMsgCreator() {
      m_msg_creator = std::make_unique<sick::MessageCreator>(
          m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset,
          m_min_intensities);
    }

    boost::asio::ip::address_v4 m_sensor_ip;
    boost::asio::ip::address_v4 m_interface_ip;
    std::string m_frame_id;
    double m_time_offset = 0.0;
    double m_range_min = 0.0;
    double m_range_max;
    double m_frequency_tolerance = 0.1;
    double m_expected_frequency = 20.0;
    double m_timestamp_min_acceptable = -1.0;
    double m_timestamp_max_acceptable = 1.0;
    double m_min_intensities = 0.0; /*!< min intensities for laser points */
    bool m_use_sick_angles;
    float m_angle_offset = -90.0;
    bool m_use_pers_conf = false;

    sick::types::port_t m_tcp_port = 2122;

    sick::datastructure::CommSettings m_communications_settings;

    std::unique_ptr<sick::MessageCreator> m_msg_creator;
  };

  Config m_config;

  /**
   * Initialize parameters for this node
   *
   * This method is a template function because the node and lifecycle node
   * are not sharing the same implementation but have the same interface.
   *
   * @tparam NodeT Templated node input
   * @param node Node
   */
  template <typename NodeT> static void initializeParameters(NodeT &node) {
    node.template declare_parameter<std::string>("frame_id", "scan");
    node.template declare_parameter<std::string>("sensor_ip", "192.168.1.11");
    node.template declare_parameter<std::string>("host_ip", "192.168.1.9");
    node.template declare_parameter<std::string>("interface_ip", "0.0.0.0");
    node.template declare_parameter<int>("host_udp_port", 0);
    node.template declare_parameter<int>("channel", 0);
    node.template declare_parameter<bool>("channel_enabled", true);
    node.template declare_parameter<int>("skip", 0);
    node.template declare_parameter<double>("angle_start", 0.0);
    node.template declare_parameter<double>("angle_end", 0.0);
    node.template declare_parameter<double>("time_offset", 0.0);
    node.template declare_parameter<bool>("general_system_state", true);
    node.template declare_parameter<bool>("derived_settings", true);
    node.template declare_parameter<bool>("measurement_data", true);
    node.template declare_parameter<bool>("intrusion_data", true);
    node.template declare_parameter<bool>("application_io_data", true);
    node.template declare_parameter<bool>("use_persistent_config", false);
    node.template declare_parameter<float>("min_intensities", 0.f);
  }

  /**
   * Load the initial parameters
   *
   * This method is a template function because the node and lifecycle node
   * are not sharing the same implementation but have the same interface.
   *
   * @tparam NodeT Templated node input
   * @param node Node
   */
  template <typename NodeT> void loadParameters(NodeT &node) {
    node.template get_parameter<std::string>("frame_id", m_config.m_frame_id);
    RCLCPP_INFO(m_logger, "frame_id: %s", m_config.m_frame_id.c_str());

    std::string sensor_ip;
    node.template get_parameter<std::string>("sensor_ip", sensor_ip);
    RCLCPP_INFO(m_logger, "sensor_ip: %s", sensor_ip.c_str());
    m_config.m_sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip);

    std::string interface_ip;
    node.template get_parameter<std::string>("interface_ip", interface_ip);
    RCLCPP_INFO(m_logger, "interface_ip: %s", interface_ip.c_str());
    m_config.m_interface_ip =
        boost::asio::ip::address_v4::from_string(interface_ip);

    std::string host_ip;
    node.template get_parameter<std::string>("host_ip", host_ip);
    RCLCPP_INFO(m_logger, "host_ip: %s", host_ip.c_str());
    // TODO check if valid IP?
    m_config.m_communications_settings.host_ip =
        boost::asio::ip::address_v4::from_string(host_ip);

    int host_udp_port;
    node.template get_parameter<int>("host_udp_port", host_udp_port);
    RCLCPP_INFO(m_logger, "host_udp_port: %i", host_udp_port);
    m_config.m_communications_settings.host_udp_port = host_udp_port;

    int channel;
    node.template get_parameter<int>("channel", channel);
    RCLCPP_INFO(m_logger, "channel: %i", channel);
    m_config.m_communications_settings.channel = channel;

    bool enabled;
    node.template get_parameter<bool>("channel_enabled", enabled);
    RCLCPP_INFO(m_logger, "channel_enabled: %s", btoa(enabled).c_str());
    m_config.m_communications_settings.enabled = enabled;

    int skip;
    node.template get_parameter<int>("skip", skip);
    RCLCPP_INFO(m_logger, "skip: %i", skip);
    m_config.m_communications_settings.publishing_frequency =
        skipToPublishFrequency(skip);

    float angle_start;
    node.template get_parameter<float>("angle_start", angle_start);
    RCLCPP_INFO(m_logger, "angle_start: %f", angle_start);

    float angle_end;
    node.template get_parameter<float>("angle_end", angle_end);
    RCLCPP_INFO(m_logger, "angle_end: %f", angle_end);

    // Included check before calculations to prevent rounding errors while
    // calculating
    if (angle_start == angle_end) {
      m_config.m_communications_settings.start_angle = sick::radToDeg(0);
      m_config.m_communications_settings.end_angle = sick::radToDeg(0);
    } else {
      m_config.m_communications_settings.start_angle =
          sick::radToDeg(angle_start) - m_config.m_angle_offset;
      m_config.m_communications_settings.end_angle =
          sick::radToDeg(angle_end) - m_config.m_angle_offset;
    }

    node.template get_parameter<double>("time_offset", m_config.m_time_offset);
    RCLCPP_INFO(m_logger, "time_offset: %f", m_config.m_time_offset);

    // Features
    bool general_system_state;
    node.template get_parameter<bool>("general_system_state",
                                      general_system_state);
    RCLCPP_INFO(m_logger, "general_system_state: %s",
                btoa(general_system_state).c_str());

    bool derived_settings;
    node.template get_parameter<bool>("derived_settings", derived_settings);
    RCLCPP_INFO(m_logger, "derived_settings: %s",
                btoa(derived_settings).c_str());

    bool measurement_data;
    node.template get_parameter<bool>("measurement_data", measurement_data);
    RCLCPP_INFO(m_logger, "measurement_data: %s",
                btoa(measurement_data).c_str());

    bool intrusion_data;
    node.template get_parameter<bool>("intrusion_data", intrusion_data);
    RCLCPP_INFO(m_logger, "intrusion_data: %s", btoa(intrusion_data).c_str());

    bool application_io_data;
    node.template get_parameter<bool>("application_io_data",
                                      application_io_data);
    RCLCPP_INFO(m_logger, "application_io_data: %s",
                btoa(application_io_data).c_str());

    m_config.m_communications_settings.features =
        sick::SensorDataFeatures::toFeatureFlags(
            general_system_state, derived_settings, measurement_data,
            intrusion_data, application_io_data);

    node.template get_parameter<bool>("use_persistent_config",
                                      m_config.m_use_pers_conf);
    RCLCPP_INFO(m_logger, "use_persistent_config: %s",
                btoa(m_config.m_use_pers_conf).c_str());

    node.template get_parameter<double>("min_intensities",
                                        m_config.m_min_intensities);
    RCLCPP_INFO(m_logger, "min_intensities: %f", m_config.m_min_intensities);
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      m_param_callback;
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Device and Communication
  std::unique_ptr<sick::AsyncSickSafetyScanner> m_device;

  /**
   * Setup the device communication
   * @param callback Callback that fires when a new UDP packet is received
   */
  void setupCommunication(
      std::function<void(const sick::datastructure::Data &)> callback);

  /**
   * Start the sensor communication by receiving UDP packets
   */
  void startCommunication();

  /**
   * Stop the sensor communication
   */
  void stopCommunication();

  // Methods Triggering COLA2 calls towards the sensor
  bool getFieldData(
      const std::shared_ptr<
          sick_safetyscanners2_interfaces::srv::FieldData::Request>
          request,
      std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response>
          response);

  void readPersistentConfig();
  void readTypeCodeSettings();
};
} // namespace sick
