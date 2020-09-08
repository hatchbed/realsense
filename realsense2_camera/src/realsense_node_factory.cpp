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
#include <sys/time.h>
#include <chrono>

#include <nodelet/NodeletUnload.h>


using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory() :
	_is_alive(true),
	_initialized(false)
{
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

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

rs2::device RealSenseNodeFactory::getDevice()
{
	rs2::device retDev;
	auto list = _ctx.query_devices();
	if (0 == list.size())
	{
		ROS_WARN("No RealSense devices were found!");
	}
	else
	{
		bool found = false;
		for (auto&& dev : list)
		{
			auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			ROS_DEBUG_STREAM("Device with serial number " << sn << " was found.");
			if (_serial_no.empty() || sn == _serial_no)
			{
				retDev = dev;
				_serial_no = sn;
				found = true;
				break;
			}
		}
		if (!found)
		{
			ROS_ERROR_STREAM("The requested device with serial number " << _serial_no << " is NOT found!");
		}
	}

	bool remove_tm2_handle(retDev && RS_T265_PID != std::stoi(retDev.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	return retDev;
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
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

void RealSenseNodeFactory::onInit()
{
	auto nh = getNodeHandle();
	auto privateNh = getPrivateNodeHandle();

	privateNh.param("initial_reset", _initial_reset, false);
	_init_timer = nh.createWallTimer(ros::WallDuration(1.0), &RealSenseNodeFactory::initialize, this, true);
}

void RealSenseNodeFactory::initialize(const ros::WallTimerEvent &ignored)
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
			ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
			auto pipe = std::make_shared<rs2::pipeline>();
			rs2::config cfg;
			cfg.enable_device_from_file(rosbag_filename.c_str(), false);
			cfg.enable_all_streams();
			pipe->start(cfg); //File will be opened in read mode at this point
			_device = pipe->get_active_profile().get_device();
			_realSenseNode = std::shared_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
			_realSenseNode->publishTopics();
			_realSenseNode->registerDynamicReconfigCb(nh);
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
								_device = getDevice();
								if (_device)
								{
									ROS_DEBUG("got device");
									if (_initial_reset)
									{
										ROS_DEBUG("Resetting device...");
										_initial_reset = false;
										_device.hardware_reset();
										std::this_thread::sleep_for(std::chrono::seconds(10));
										_device = getDevice();
									}
									if (_device)
									{
										ROS_DEBUG("starting device...");
										std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
										_ctx.set_devices_changed_callback(change_device_callback_function);
										StartDevice();
									}
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
		throw;
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		throw;
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
	case RS_USB2_PID:
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

	_init_timer = getNodeHandle().createWallTimer(ros::WallDuration(1.0), &RealSenseNodeFactory::initialize, this, true);
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
