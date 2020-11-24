// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <regex>

#include <nodelet/NodeletUnload.h>


using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

std::string api_version_to_string(int version)
{
	std::ostringstream ss;
	if (version / 10000 == 0)
		ss << version;
	else
		ss << (version / 10000) << "." << (version % 10000) / 100 << "." << (version % 100);
	return ss.str();
}

RealSenseNodeFactory::RealSenseNodeFactory() :
	_is_alive(true),
	_initialized(false)
{
	rs2_error* e = nullptr;
	std::string running_librealsense_version(api_version_to_string(rs2_get_api_version(&e)));
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Built with LibRealSense v%s", RS2_API_VERSION_STR);
	ROS_INFO_STREAM("Running with LibRealSense v" << running_librealsense_version);
	if (RS2_API_VERSION_STR != running_librealsense_version)
	{
		ROS_WARN("***************************************************");
		ROS_WARN("** running with a different librealsense version **");
		ROS_WARN("** than the one the wrapper was compiled with!   **");
		ROS_WARN("***************************************************");
	}

	auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
	tryGetLogSeverity(severity);
	if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	rs2::log_to_console(severity);
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
	try
	{
		_initialized = false;
		_data_monitor_timer.stop();
		_is_alive = false;
		if (_query_thread.joinable())
		{
			_query_thread.join();
		}

		_realSenseNode.reset();
		if (_device)
		{
			_device.hardware_reset();
			_device = rs2::device();
		}
	}
	catch (const rs2::error& e)
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
	}
}

std::string RealSenseNodeFactory::parse_usb_port(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
	if (!_device)
	{
		if (0 == list.size())
		{
			ROS_WARN("No RealSense devices were found!");
		}
		else
		{
			bool found = false;
      		ROS_INFO_STREAM(" ");
			for (size_t count = 0; count < list.size(); count++)
			{
				rs2::device dev;
				try
				{
					dev = list[count];
				}
				catch(const std::exception& ex)
				{
					ROS_WARN_STREAM("Device " << count+1 << "/" << list.size() << " failed with exception: " << ex.what());
					continue;
				}
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);
				std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
				std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
				ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
				std::vector<std::string> results;
				ROS_INFO_STREAM("Device with name " << name << " was found.");
				std::string port_id = parse_usb_port(pn);
				if (port_id.empty())
				{
					std::stringstream msg;
					msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
					if (_usb_port_id.empty())
					{
						ROS_WARN_STREAM(msg.str());
					}
					else
					{
						ROS_ERROR_STREAM(msg.str());
						ROS_ERROR_STREAM("Please use serial number instead of usb port.");
					}
				}
				else
				{
					ROS_INFO_STREAM("Device with port number " << port_id << " was found.");					
				}
				bool found_device_type(true);
				if (!_device_type.empty())
				{
					std::smatch match_results;
					std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
					found_device_type = std::regex_search(name, match_results, device_type_regex);
				}

				if ((_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
				{
					_device = dev;
					_serial_no = sn;
					found = true;
					break;
				}
			}
			if (!found)
			{
				// T265 could be caught by another node.
				std::string msg ("The requested device with ");
				bool add_and(false);
				if (!_serial_no.empty())
				{
					msg += "serial number " + _serial_no;
					add_and = true;
				}
				if (!_usb_port_id.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "usb port id " + _usb_port_id;
					add_and = true;
				}
				if (!_device_type.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "device name containing " + _device_type;
				}
				msg += " is NOT found. Will Try again.";
				ROS_ERROR_STREAM(msg);
			}
			else
			{
				if (_device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
				{
					std::string usb_type = _device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
					ROS_INFO_STREAM("Device USB type: " << usb_type);
					if (usb_type.find("2.") != std::string::npos)
					{
						ROS_WARN_STREAM("Device " << _serial_no << " is connected using a " << usb_type << " port. Reduced performance is expected.");
					}
				}
			}
		}
	}

	bool remove_tm2_handle(_device && RS_T265_PID != std::stoi(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	if (_device && _initial_reset)
	{
		_initial_reset = false;
		try
		{
			ROS_INFO("Resetting device...");
			_device.hardware_reset();
			_device = rs2::device();
			
		}
		catch(const std::exception& ex)
		{
			ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
		}
	}
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
        // 20201103 @130s is afraid the behavior might have changed in https://github.com/plusone-robotics/realsense/pull/28
        // Upstream may request maintaining backward compatiblity.
	if (_initialized)
	{
		if (info.was_removed(_device))
		{
			ROS_ERROR("The device has been disconnected!");
			ROS_ERROR("Shutting down ...");
			shutdown();
		}
	}
}

bool RealSenseNodeFactory::toggle_sensor_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data)
    ROS_INFO_STREAM("toggling sensor : ON");
  else
    ROS_INFO_STREAM("toggling sensor : OFF");
  _realSenseNode->toggleSensors(req.data);
  res.success=true;
  return true;
}

void RealSenseNodeFactory::onInit()
{
	auto nh = getNodeHandle();
	auto privateNh = getPrivateNodeHandle();

	privateNh.param("initial_reset", _initial_reset, false);
	initialize();
}

void RealSenseNodeFactory::initCallback(const ros::WallTimerEvent &ignored)
{
	initialize();
}

void RealSenseNodeFactory::initialize()
{
	_device = rs2::device();
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		privateNh.param("serial_no", _serial_no, std::string(""));
    	privateNh.param("usb_port_id", _usb_port_id, std::string(""));
    	privateNh.param("device_type", _device_type, std::string(""));
    toggle_sensor_srv = nh.advertiseService("enable", &RealSenseNodeFactory::toggle_sensor_callback, this);

                // Setting timeout condition for respawning node.
		std::string timeout_action;
		privateNh.param("timeout_action", timeout_action, std::string("warn"));
		if (timeout_action == "warn")
		{
			_timeout_action = TIMEOUT_WARN;
		}
		else if (timeout_action == "reset")
		{
			_timeout_action = TIMEOUT_RESET;
		}
		else if (timeout_action == "shutdown")
		{
			_timeout_action = TIMEOUT_SHUTDOWN;
		}
		else
		{
			_timeout_action = TIMEOUT_WARN;
			ROS_WARN("Invalid timeout_action (warn|reset|shutdown): %s", timeout_action.c_str());
		}

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));

		if (!rosbag_filename.empty())
		{
			{
				ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
				auto pipe = std::make_shared<rs2::pipeline>();
				rs2::config cfg;
				cfg.enable_device_from_file(rosbag_filename.c_str(), false);
				cfg.enable_all_streams();
				pipe->start(cfg); //File will be opened in read mode at this point
				_device = pipe->get_active_profile().get_device();
				_serial_no = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			}
			if (_device)
			{
				StartDevice();
			}
		}
		else
		{
			_is_alive = true;
			_query_thread = std::thread([=]()
						{
							ROS_DEBUG("Waiting for device...");

							std::chrono::milliseconds timespan(6000);
							while (_is_alive && !_device)
							{
								ROS_DEBUG("Checking for device...");
								getDevice(_ctx.query_devices());
								if (_device)
								{
									ROS_DEBUG("starting device...");
									std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
									_ctx.set_devices_changed_callback(change_device_callback_function);
									StartDevice();
								}
								else
								{
									std::this_thread::sleep_for(timespan);
								}
							}
						});

			if (!_shutdown_srv)
			{
				_shutdown_srv = privateNh.advertiseService("shutdown", &RealSenseNodeFactory::handleShutdown, this);
			}
			if (!_reset_srv)
			{
				_reset_srv = privateNh.advertiseService("reset", &RealSenseNodeFactory::handleReset, this);
			}
		}
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		exit(1);
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		exit(1);
	}
}

void RealSenseNodeFactory::dataMonitor(const ros::TimerEvent &e)
{
	if (!_realSenseNode || _data_timeout <= 0.0)
	{
		return;
	}

	ros::Time timeout = e.current_real - ros::Duration(_data_timeout);

	std::vector<std::string> stale_topics;
	if (!std::dynamic_pointer_cast<BaseRealSenseNode>(_realSenseNode)->checkTopics(timeout, stale_topics))
	{
		if (_timeout_action == TIMEOUT_WARN)
		{
			std::string stale_topic_string;
			for (const auto& topic: stale_topics)
			{
				stale_topic_string += topic + ", ";
			}
			ROS_WARN_THROTTLE(1.0, "Realsense data timed out. Stale topics: %s", stale_topic_string.c_str());
		}
		else if (_timeout_action == TIMEOUT_RESET)
		{
			ROS_ERROR("Realsense data timed out. Resetting driver.");
			ROS_ERROR("The following topics were stale:");
			for (const auto& topic: stale_topics)
			{
				ROS_ERROR("  %s", topic.c_str());
			}
			reset();
		}
		else if (_timeout_action == TIMEOUT_SHUTDOWN)
		{
			ROS_ERROR("Realsense data timed out. Shutting down driver.");
			ROS_ERROR("The following topics were stale:");
			for (const auto& topic: stale_topics)
			{
				ROS_ERROR("  %s", topic.c_str());
			}
			shutdown();
		}
	}
}

void RealSenseNodeFactory::StartDevice()
{
	if (_realSenseNode) _realSenseNode.reset();
	try
	{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();
	// TODO
	std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch(pid)
	{
	case SR300_PID:
	case SR300v2_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS455_PID:
	case RS465_PID:
	case RS_USB2_PID:
	case RS_L515_PID_PRE_PRQ:
	case RS_L515_PID:
		_realSenseNode = std::shared_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	case RS_T265_PID:
		_realSenseNode = std::shared_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
		break;
	default:
		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
		ros::shutdown();
		exit(1);
	}
	assert(_realSenseNode);
	_realSenseNode->publishTopics();
	_realSenseNode->registerDynamicReconfigCb(nh);

	_initialized = true;

	_data_timeout = privateNh.param("data_timeout", 0.0);
	if (_data_timeout > 0.0)
	{
		ROS_INFO("starting data monitor ...");
		_data_monitor_timer = nh.createTimer(ros::Duration(_data_timeout), &RealSenseNodeFactory::dataMonitor, this, false);
	}
	else
	{
		ROS_INFO("not monitoring data.");
	}
  }
	catch (const rs2::error& e)
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
	}
}

bool RealSenseNodeFactory::shutdown()
{
	_initialized = false;
	_data_monitor_timer.stop();

	std::string manager_name = ros::this_node::getName();
	std::string unload_service = manager_name + "/unload_nodelet";

	if (ros::service::waitForService(unload_service, 0.1))
	{
		nodelet::NodeletUnload srv;
		srv.request.name = getName();
		if (!ros::service::call(unload_service, srv) || !srv.response.success)
		{
			ROS_WARN("Failed to unload nodelet, requesting shutdown ...");
			ros::requestShutdown();
		}
	}
	else
	{
		ROS_WARN("Failed to find unload nodelet service, requesting shutdown ...");
		ros::requestShutdown();
	}

	return true;
}

bool RealSenseNodeFactory::reset()
{
	if (!_initialized)
	{
		return false;
	}

	_initialized = false;

	_data_monitor_timer.stop();

	_is_alive = false;
	if (_query_thread.joinable())
	{
		_query_thread.join();
	}

	try
	{
	_realSenseNode.reset();
		if (_device)
		{
			_device.hardware_reset();
			_device = rs2::device();
		}
	}
	catch (const rs2::error& e)
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
	}

	_init_timer = getNodeHandle().createWallTimer(ros::WallDuration(1.0), &RealSenseNodeFactory::initCallback, this, true);
	return true;
}

bool RealSenseNodeFactory::handleShutdown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	return shutdown();
}

bool RealSenseNodeFactory::handleReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	return reset();
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
	static const char* severity_var_name = "LRS_LOG_LEVEL";
	auto content = getenv(severity_var_name);

	if (content)
	{
		std::string content_str(content);
		std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

		for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
		{
			auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
			std::transform(current.begin(), current.end(), current.begin(), ::toupper);
			if (content_str == current)
			{
				severity = (rs2_log_severity)i;
				break;
			}
		}
	}
}
