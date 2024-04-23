/**
 * @brief Automatic dependent surveillance-broadcast Vehicle plugin
 * @file open_drone_id.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/BasicID.h>
#include <mavros_msgs/OperatorID.h>
#include <mavros_msgs/SelfID.h>
#include <mavros_msgs/System.h>
#include <mavros_msgs/SystemUpdate.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Open Drone ID plugin
 *
 * Send Open Drone ID information to vehicle.
 */
class OpenDroneIDPlugin : public plugin::PluginBase {
public:
	OpenDroneIDPlugin() : PluginBase(),
		open_drone_id_nh("~open_drone_id")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		basic_id_sub = open_drone_id_nh.subscribe("basic_id", 10, &OpenDroneIDPlugin::basic_id_cb, this);
		operator_id_sub = open_drone_id_nh.subscribe("operator_id", 10, &OpenDroneIDPlugin::operator_id_cb, this);
		self_id_sub = open_drone_id_nh.subscribe("self_id", 10, &OpenDroneIDPlugin::self_id_cb, this);
		system_sub = open_drone_id_nh.subscribe("system", 10, &OpenDroneIDPlugin::system_cb, this);
		system_update_sub = open_drone_id_nh.subscribe("system_update", 10, &OpenDroneIDPlugin::system_update_cb, this);

		test_timer = open_drone_id_nh.createTimer(ros::Duration(1.0), &OpenDroneIDPlugin::test_timer_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return { };
	}

private:
	ros::NodeHandle open_drone_id_nh;

	ros::Subscriber basic_id_sub;
	ros::Subscriber operator_id_sub;
	ros::Subscriber self_id_sub;
	ros::Subscriber system_sub;
	ros::Subscriber system_update_sub;

	ros::Timer test_timer;

	void basic_id_cb(const mavros_msgs::BasicID::ConstPtr &msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

		ROS_INFO("Receiving basic ID message");

		basic_id.id_type = msg->id_type;
		basic_id.ua_type = msg->ua_type;

		size_t length = std::min(basic_id.uas_id.size(), msg->uas_id.size());
		std::memcpy(basic_id.uas_id.data(), msg->uas_id.data(), length);

		UAS_FCU(m_uas)->send_message_ignore_drop(basic_id);
	}

	void operator_id_cb(const mavros_msgs::OperatorID::ConstPtr &msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID operator_id{};

		UAS_FCU(m_uas)->send_message_ignore_drop(operator_id);
	}

	void self_id_cb(const mavros_msgs::SelfID::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SELF_ID self_id{};

		UAS_FCU(m_uas)->send_message_ignore_drop(self_id);
	}

	void system_cb(const mavros_msgs::System::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM system{};

		UAS_FCU(m_uas)->send_message_ignore_drop(system);
	}

	void system_update_cb(const mavros_msgs::SystemUpdate::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE system_update{};

		UAS_FCU(m_uas)->send_message_ignore_drop(system_update);
	}

	void test_timer_cb(const ros::TimerEvent&) {
		ROS_INFO("Open Drone ID plugin working");

		{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

		basic_id.ua_type = (uint8_t)mavlink::common::MAV_ODID_UA_TYPE::HELICOPTER_OR_MULTIROTOR;
		basic_id.id_type = (uint8_t)mavlink::common::MAV_ODID_ID_TYPE::SERIAL_NUMBER;

		std::array<uint8_t, 20UL> uas_id;

		uas_id[0] = 54;
		uas_id[1] = 57;
		basic_id.uas_id = uas_id;

		UAS_FCU(m_uas)->send_message_ignore_drop(basic_id);
		}


		{
		mavlink::common::msg::OPEN_DRONE_ID_LOCATION location{};

		UAS_FCU(m_uas)->send_message_ignore_drop(location);
		}

		{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM system{};

		UAS_FCU(m_uas)->send_message_ignore_drop(system);
		}

		{
		mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID operator_id{};

		UAS_FCU(m_uas)->send_message_ignore_drop(operator_id);
		}

		{
		mavlink::common::msg::OPEN_DRONE_ID_SELF_ID self_id{};

		UAS_FCU(m_uas)->send_message_ignore_drop(self_id);
		}
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OpenDroneIDPlugin, mavros::plugin::PluginBase)
