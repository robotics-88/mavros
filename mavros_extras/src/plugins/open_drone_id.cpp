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

	void basic_id_cb(const mavros_msgs::BasicID::ConstPtr &msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

		basic_id.id_type = msg->id_type;
		basic_id.ua_type = msg->ua_type;

		size_t length = std::min(basic_id.uas_id.size(), msg->uas_id.size());
		std::memcpy(basic_id.uas_id.data(), msg->uas_id.data(), length);

		UAS_FCU(m_uas)->send_message_ignore_drop(basic_id);
	}

	void operator_id_cb(const mavros_msgs::OperatorID::ConstPtr &msg)
	{
		mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID operator_id{};

		operator_id.operator_id_type = msg->operator_id_type;

		size_t length = std::min(operator_id.operator_id.size(), msg->operator_id.size());
		std::memcpy(operator_id.operator_id.data(), msg->operator_id.data(), length);

		UAS_FCU(m_uas)->send_message_ignore_drop(operator_id);
	}

	void self_id_cb(const mavros_msgs::SelfID::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SELF_ID self_id{};
		self_id.description_type = msg->description_type;

		size_t length = std::min(self_id.description.size(), msg->description.size());
		std::memcpy(self_id.description.data(), msg->description.data(), length);

		UAS_FCU(m_uas)->send_message_ignore_drop(self_id);
	}

	void system_cb(const mavros_msgs::System::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM system{};

		system.operator_location_type = msg->operator_location_type;
		system.classification_type = msg->classification_type;
		system.operator_latitude = msg->operator_latitude;
		system.operator_longitude = msg->operator_longitude;
		system.area_count = msg->area_count;
		system.area_radius = msg->area_radius;
		system.area_ceiling = msg->area_ceiling;
		system.area_floor = msg->area_floor;
		system.category_eu = msg->category_eu;
		system.class_eu = msg->class_eu;
		system.operator_altitude_geo = msg->operator_altitude_geo;
		system.timestamp = msg->timestamp;

		UAS_FCU(m_uas)->send_message_ignore_drop(system);
	}

	void system_update_cb(const mavros_msgs::SystemUpdate::ConstPtr &msg) 
	{
		mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE system_update{};

		UAS_FCU(m_uas)->send_message_ignore_drop(system_update);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OpenDroneIDPlugin, mavros::plugin::PluginBase)
