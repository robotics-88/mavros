/**
 * @brief Automatic dependent surveillance-broadcast Vehicle plugin
 * @file remote_id.cpp
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

#include <mavros_msgs/RemoteID.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief ARemote ID plugin
 *
 * Send remote ID information to vehicle.
 */
class RemoteIDPlugin : public plugin::PluginBase {
public:
	RemoteIDPlugin() : PluginBase(),
		remote_id_nh("~remote_id")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		remote_id_sub = remote_id_nh.subscribe("send", 10, &RemoteIDPlugin::remote_id_cb, this);

		test_timer = remote_id_nh.createTimer(ros::Duration(1.0), &RemoteIDPlugin::test_timer_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&RemoteIDPlugin::handle_remote_id)
		};
	}

private:
	ros::NodeHandle remote_id_nh;

	ros::Publisher remote_id_pub;
	ros::Subscriber remote_id_sub;

	ros::Timer test_timer;

	void handle_remote_id(const mavlink::mavlink_message_t *msg, mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS &remote_id)
	{
		return;
	}

	void remote_id_cb(const mavros_msgs::RemoteID::ConstPtr &msg)
	{
		
		{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

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

	void test_timer_cb(const ros::TimerEvent&) {
		ROS_INFO("Remote ID plugin working");

		{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

		basic_id.ua_type = (uint8_t)mavlink::common::MAV_ODID_UA_TYPE::AEROPLANE;
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
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RemoteIDPlugin, mavros::plugin::PluginBase)
