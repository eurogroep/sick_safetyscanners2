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

#include <sick_safetyscanners2/SickSafetyscannersHelper.hpp>

namespace sick {
void SickSafetyscannersHelper::initializeParameters(rclcpp::Node& node)
{
  node.declare_parameter<std::string>("frame_id", "scan");
  node.declare_parameter<std::string>("sensor_ip", "192.168.1.11");
  node.declare_parameter<std::string>("host_ip", "192.168.1.9");
  node.declare_parameter<std::string>("interface_ip", "0.0.0.0");
  node.declare_parameter<int>("host_udp_port", 0);
  node.declare_parameter<int>("channel", 0);
  node.declare_parameter<bool>("channel_enabled", true);
  node.declare_parameter<int>("skip", 0);
  node.declare_parameter<double>("angle_start", 0.0);
  node.declare_parameter<double>("angle_end", 0.0);
  node.declare_parameter<double>("time_offset", 0.0);
  node.declare_parameter<bool>("general_system_state", true);
  node.declare_parameter<bool>("derived_settings", true);
  node.declare_parameter<bool>("measurement_data", true);
  node.declare_parameter<bool>("intrusion_data", true);
  node.declare_parameter<bool>("application_io_data", true);
  node.declare_parameter<bool>("use_persistent_config", false);
  node.declare_parameter<float>("min_intensities", 0.f);
}

void SickSafetyscannersHelper::loadParameters(rclcpp::Node& node, SickSafetyscannersHelper::Config &config)
{
  auto node_logger = node.get_logger();

  node.get_parameter<std::string>("frame_id", config.m_frame_id);
  RCLCPP_INFO(node_logger, "frame_id: %s", config.m_frame_id.c_str());

  std::string sensor_ip;
  node.get_parameter<std::string>("sensor_ip", sensor_ip);
  RCLCPP_INFO(node_logger, "sensor_ip: %s", sensor_ip.c_str());
  config.m_sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip);

  std::string interface_ip;
  node.get_parameter<std::string>("interface_ip", interface_ip);
  RCLCPP_INFO(node_logger, "interface_ip: %s", interface_ip.c_str());
  config.m_interface_ip = boost::asio::ip::address_v4::from_string(interface_ip);

  std::string host_ip;
  node.get_parameter<std::string>("host_ip", host_ip);
  RCLCPP_INFO(node_logger, "host_ip: %s", host_ip.c_str());
  // TODO check if valid IP?
  config.m_communications_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip);

  int host_udp_port;
  node.get_parameter<int>("host_udp_port", host_udp_port);
  RCLCPP_INFO(node_logger, "host_udp_port: %i", host_udp_port);
  config.m_communications_settings.host_udp_port = host_udp_port;

  int channel;
  node.get_parameter<int>("channel", channel);
  RCLCPP_INFO(node_logger, "channel: %i", channel);
  config.m_communications_settings.channel = channel;

  bool enabled;
  node.get_parameter<bool>("channel_enabled", enabled);
  RCLCPP_INFO(node_logger, "channel_enabled: %s", btoa(enabled).c_str());
  config.m_communications_settings.enabled = enabled;

  int skip;
  node.get_parameter<int>("skip", skip);
  RCLCPP_INFO(node_logger, "skip: %i", skip);
  config.m_communications_settings.publishing_frequency = skipToPublishFrequency(skip);

  float angle_start;
  node.get_parameter<float>("angle_start", angle_start);
  RCLCPP_INFO(node_logger, "angle_start: %f", angle_start);

  float angle_end;
  node.get_parameter<float>("angle_end", angle_end);
  RCLCPP_INFO(node_logger, "angle_end: %f", angle_end);

  // Included check before calculations to prevent rounding errors while calculating
  if (angle_start == angle_end)
  {
    config.m_communications_settings.start_angle = sick::radToDeg(0);
    config.m_communications_settings.end_angle   = sick::radToDeg(0);
  }
  else
  {
    config.m_communications_settings.start_angle = sick::radToDeg(angle_start) - config.m_angle_offset;
    config.m_communications_settings.end_angle   = sick::radToDeg(angle_end) - config.m_angle_offset;
  }


  node.get_parameter<double>("time_offset", config.m_time_offset);
  RCLCPP_INFO(node_logger, "time_offset: %f", config.m_time_offset);

  // Features
  bool general_system_state;
  node.get_parameter<bool>("general_system_state", general_system_state);
  RCLCPP_INFO(node_logger, "general_system_state: %s", btoa(general_system_state).c_str());

  bool derived_settings;
  node.get_parameter<bool>("derived_settings", derived_settings);
  RCLCPP_INFO(node_logger, "derived_settings: %s", btoa(derived_settings).c_str());

  bool measurement_data;
  node.get_parameter<bool>("measurement_data", measurement_data);
  RCLCPP_INFO(node_logger, "measurement_data: %s", btoa(measurement_data).c_str());

  bool intrusion_data;
  node.get_parameter<bool>("intrusion_data", intrusion_data);
  RCLCPP_INFO(node_logger, "intrusion_data: %s", btoa(intrusion_data).c_str());

  bool application_io_data;
  node.get_parameter<bool>("application_io_data", application_io_data);
  RCLCPP_INFO(node_logger, "application_io_data: %s", btoa(application_io_data).c_str());

  config.m_communications_settings.features = sick::SensorDataFeatures::toFeatureFlags(
    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);

  node.get_parameter<bool>("use_persistent_config", config.m_use_pers_conf);
  RCLCPP_INFO(node_logger, "use_persistent_config: %s", btoa(config.m_use_pers_conf).c_str());

  node.get_parameter<double>("min_intensities", config.m_min_intensities);
  RCLCPP_INFO(node_logger, "min_intensities: %f", config.m_min_intensities);
}

bool SickSafetyscannersHelper::handleParameterUpdate(const rclcpp::Logger& logger, const std::vector<rclcpp::Parameter>& parameters, SickSafetyscannersHelper::Config& config)
{
  bool update_sensor_config = false;

  for (const auto& param : parameters)
  {
    std::stringstream ss;
    ss << "{" << param.get_name() << ", " << param.value_to_string() << "}";
    RCLCPP_INFO(logger, "Got parameter: '%s'", ss.str().c_str());

    if (param.get_name() == "frame_id")
    {
      config.m_frame_id = param.value_to_string();
    }
    else if (param.get_name() == "host_ip")
    {
      config.m_communications_settings.host_ip =
        boost::asio::ip::address_v4::from_string(param.value_to_string());
      update_sensor_config = true;
    }
    else if (param.get_name() == "host_udp_port")
    {
      config.m_communications_settings.host_udp_port = param.as_int();
      update_sensor_config                    = true;
    }
    else if (param.get_name() == "channel")
    {
      config.m_communications_settings.channel = param.as_int();
      update_sensor_config              = true;
    }
    else if (param.get_name() == "channel_enabled")
    {
      config.m_communications_settings.enabled = param.as_bool();
      update_sensor_config              = true;
    }
    else if (param.get_name() == "skip")
    {
      config.m_communications_settings.publishing_frequency = skipToPublishFrequency(param.as_int());
      update_sensor_config                           = true;
    }
    else if (param.get_name() == "angle_start")
    {
      config.m_communications_settings.start_angle = sick::radToDeg(param.as_double()) - config.m_angle_offset;
      update_sensor_config = true;
    }
    else if (param.get_name() == "angle_end")
    {
      config.m_communications_settings.end_angle   = sick::radToDeg(param.as_double()) - config.m_angle_offset;
      update_sensor_config = true;
    }
    else if (param.get_name() == "time_offset")
    {
      config.m_time_offset = param.as_double();
    }
    else if (param.get_name() == "general_system_state")
    {
      // TODO improve
      config.m_communications_settings.features =
        (config.m_communications_settings.features & ~(1UL << 0)) | (param.as_bool() << 0);
      update_sensor_config = true;
    }
    else if (param.get_name() == "derived_settings")
    {
      config.m_communications_settings.features =
        (config.m_communications_settings.features & ~(1UL << 1)) | (param.as_bool() << 1);
      update_sensor_config = true;
    }
    else if (param.get_name() == "measurement_data")
    {
      config.m_communications_settings.features =
        (config.m_communications_settings.features & ~(1UL << 2)) | (param.as_bool() << 2);
      update_sensor_config = true;
    }
    else if (param.get_name() == "intrusion_data")
    {
      config.m_communications_settings.features =
        (config.m_communications_settings.features & ~(1UL << 3)) | (param.as_bool() << 3);
      update_sensor_config = true;
    }
    else if (param.get_name() == "application_io_data")
    {
      config.m_communications_settings.features =
        (config.m_communications_settings.features & ~(1UL << 4)) | (param.as_bool() << 4);
      update_sensor_config = true;
    }
    else if (param.get_name() == "min_intensities")
    {
      config.m_min_intensities = param.as_double();
    }
    else
    {
      throw std::runtime_error("Parameter is not dynamic reconfigurable");
    }
  }

  config.setupMsgCreator();

  return update_sensor_config;
}

bool SickSafetyscannersHelper::getFieldData(const Config& config, sick::AsyncSickSafetyScanner& device, sick_safetyscanners2_interfaces::srv::FieldData::Response& response)
{
  std::vector<sick::datastructure::FieldData> fields;
  device.requestFieldData(fields);

  for (size_t i = 0; i < fields.size(); i++)
  {
    sick::datastructure::FieldData field = fields.at(i);
    sick_safetyscanners2_interfaces::msg::Field field_msg;

    field_msg.start_angle        = degToRad(field.getStartAngle() + config.m_angle_offset);
    field_msg.angular_resolution = degToRad(field.getAngularBeamResolution());
    field_msg.protective_field   = field.getIsProtectiveField();

    std::vector<uint16_t> ranges = field.getBeamDistances();
    for (size_t j = 0; j < ranges.size(); j++)
    {
      field_msg.ranges.push_back(static_cast<float>(ranges.at(j)) * 1e-3);
    }

    response.fields.push_back(field_msg);
  }

  datastructure::DeviceName device_name;
  device.requestDeviceName(device_name);
  response.device_name = device_name.getDeviceName();

  std::vector<sick::datastructure::MonitoringCaseData> monitoring_cases;
  device.requestMonitoringCases(monitoring_cases);

  for (const auto& monitoring_case : monitoring_cases)
  {
    sick_safetyscanners2_interfaces::msg::MonitoringCase monitoring_case_msg;

    monitoring_case_msg.monitoring_case_number = monitoring_case.getMonitoringCaseNumber();
    std::vector<uint16_t> mon_fields           = monitoring_case.getFieldIndices();
    std::vector<bool> mon_fields_valid         = monitoring_case.getFieldsValid();
    for (size_t j = 0; j < mon_fields.size(); j++)
    {
      monitoring_case_msg.fields.push_back(mon_fields.at(j));
      monitoring_case_msg.fields_valid.push_back(mon_fields_valid.at(j));
    }
    response.monitoring_cases.push_back(monitoring_case_msg);
  }

  return true;
}
} // namespace sick
