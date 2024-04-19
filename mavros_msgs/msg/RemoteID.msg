# Remote ID message

std_msgs/Header header


# Basic ID

uint8 id_type
uint8 MAV_ODID_ID_TYPE_NONE = 0
uint8 MAV_ODID_ID_TYPE_SERIAL_NUMBER = 1
uint8 MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID = 2
uint8 MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID = 3
uint8 MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID  = 4


uint8 ua_type
uint8[] uas_id

# Location