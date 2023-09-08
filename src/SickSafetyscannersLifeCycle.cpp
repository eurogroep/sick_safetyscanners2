// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2020, SICK AG, Waldkirch
*  Copyright (C) 2020, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file SickSafetyscannersLifeCycle.cpp
 *
 * \authors  Soma gallai<soma.gallai@cm-robotics.com>  Erwin Lejeune <erwin.lejeune@cm-robotics.com>
 * \date    2021-05-27
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscannersLifeCycle.hpp>

namespace sick {


SickSafetyscannersLifeCycle::SickSafetyscannersLifeCycle(const std::string& node_name,
                                                         bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(
      node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  RCLCPP_INFO(this->get_logger(), "Initializing SickSafetyscannersLifeCycle ");

  // read parameters!
  SickSafetyscannersHelper::initializeParameters(*this);
  SickSafetyscannersHelper::loadParameters(*this, m_config);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_configure()...");

  // Dynamic Parameter Change client
  m_param_callback = add_on_set_parameters_callback(
    std::bind(&SickSafetyscannersLifeCycle::parametersCallback, this, std::placeholders::_1));

  // TODO diagnostics

  // init publishers and services
  m_laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  m_extended_laser_scan_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>("extended_scan",
                                                                                    1);
  m_output_paths_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>("output_paths", 1);
  m_raw_data_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::RawMicroScanData>("raw_data", 1);

  m_field_data_service = this->create_service<sick_safetyscanners2_interfaces::srv::FieldData>(
    "field_data",
    std::bind(&SickSafetyscannersLifeCycle::getFieldData,
              this,
              std::placeholders::_1,
              std::placeholders::_2));


  // Bind callback
  std::function<void(const sick::datastructure::Data&)> callback =
    std::bind(&SickSafetyscannersLifeCycle::receiveUDPPaket, this, std::placeholders::_1);

  // Create a sensor instance
  if (m_config.m_communications_settings.host_ip.is_multicast())
  {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port, m_config.m_communications_settings, m_config.m_interface_ip, callback);
  }
  else
  {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port, m_config.m_communications_settings, callback);
  }

  RCLCPP_INFO(this->get_logger(), "Communication to Sensor set up");

  // Read sensor specific configurations
  readTypeCodeSettings();

  if (m_config.m_use_pers_conf)
  {
    readPersistentConfig();
  }

  m_config.setupMsgCreator();

  RCLCPP_INFO(this->get_logger(), "Node Configured");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_activate()...");
  // Start async receiving and processing of sensor data
  m_device->run();
  m_device->changeSensorSettings(m_config.m_communications_settings);

  m_laser_scan_publisher->on_activate();
  m_extended_laser_scan_publisher->on_activate();
  m_output_paths_publisher->on_activate();
  m_raw_data_publisher->on_activate();

  RCLCPP_INFO(this->get_logger(), "Node activated, device is running...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
  m_device->stop();
  m_laser_scan_publisher->on_deactivate();
  m_extended_laser_scan_publisher->on_deactivate();
  m_output_paths_publisher->on_deactivate();
  m_raw_data_publisher->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Node deactivated, device stopped...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
  m_laser_scan_publisher.reset();
  m_extended_laser_scan_publisher.reset();
  m_output_paths_publisher.reset();
  m_raw_data_publisher.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
  m_laser_scan_publisher.reset();
  m_extended_laser_scan_publisher.reset();
  m_output_paths_publisher.reset();
  m_raw_data_publisher.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SickSafetyscannersLifeCycle::readTypeCodeSettings()
{
  RCLCPP_INFO(this->get_logger(), "Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_config.m_communications_settings.e_interface_type = type_code.getInterfaceType();
  m_config.m_range_min                                = 0.1;
  m_config.m_range_max                                = type_code.getMaxRange();
}

void SickSafetyscannersLifeCycle::readPersistentConfig()
{
  RCLCPP_INFO(this->get_logger(), "Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_config.m_communications_settings.start_angle = config_data.getStartAngle();
  m_config.m_communications_settings.end_angle   = config_data.getEndAngle();
}

rcl_interfaces::msg::SetParametersResult
SickSafetyscannersLifeCycle::parametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful         = true;
  result.reason             = "";
  bool update_sensor_config = SickSafetyscannersHelper::handleParameterUpdate(this->get_logger(), parameters, m_config);

  if (update_sensor_config)
  {
    m_device->changeSensorSettings(m_config.m_communications_settings);
  }

  return result;
}


void SickSafetyscannersLifeCycle::receiveUDPPaket(const sick::datastructure::Data& data)
{
  if (!m_config.m_msg_creator)
  {
    RCLCPP_WARN(get_logger(),
                "Received UDPP packet before all objects were instantiated, ignoring this packet.");
    return;
  }

  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    auto scan = m_config.m_msg_creator->createLaserScanMsg(data, this->now());
    m_laser_scan_publisher->publish(scan);

    sick_safetyscanners2_interfaces::msg::ExtendedLaserScan extended_scan =
        m_config.m_msg_creator->createExtendedLaserScanMsg(data, this->now());

    m_extended_laser_scan_publisher->publish(extended_scan);

    auto output_paths = m_config.m_msg_creator->createOutputPathsMsg(data);
    m_output_paths_publisher->publish(output_paths);
  }

  auto raw_msg = m_config.m_msg_creator->createRawDataMsg(data);
  m_raw_data_publisher->publish(raw_msg);
}


bool SickSafetyscannersLifeCycle::getFieldData(
  const std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Request> request,
  std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response> response)
{
  // Suppress warning of unused request variable due to empty request fields
  (void)request;
  return SickSafetyscannersHelper::getFieldData(m_config, *m_device, *response);
}


} // namespace sick
