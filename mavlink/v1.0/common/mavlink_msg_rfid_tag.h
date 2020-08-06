// MESSAGE RFID_TAG PACKING

#define MAVLINK_MSG_ID_RFID_TAG 154

typedef struct __mavlink_rfid_tag_t
{
 uint64_t timestamp; /*< time stamp*/
 int32_t RSSI; /*< RSSI*/
 float x; /*< x position of UAV when reading tag*/
 float y; /*< x position of UAV when reading tag*/
 float z; /*< x position of UAV when reading tag*/
 float yaw; /*< x position of UAV when reading tag*/
 uint16_t position_tag; /*<  position tag of UAV when reading tag*/
 char tag_ID[40]; /*< Tag ID*/
 uint8_t ReadCount; /*< read count*/
} mavlink_rfid_tag_t;

#define MAVLINK_MSG_ID_RFID_TAG_LEN 71
#define MAVLINK_MSG_ID_154_LEN 71

#define MAVLINK_MSG_ID_RFID_TAG_CRC 26
#define MAVLINK_MSG_ID_154_CRC 26

#define MAVLINK_MSG_RFID_TAG_FIELD_TAG_ID_LEN 40

#define MAVLINK_MESSAGE_INFO_RFID_TAG { \
	"RFID_TAG", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rfid_tag_t, timestamp) }, \
         { "RSSI", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_rfid_tag_t, RSSI) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rfid_tag_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rfid_tag_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rfid_tag_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rfid_tag_t, yaw) }, \
         { "position_tag", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_rfid_tag_t, position_tag) }, \
         { "tag_ID", NULL, MAVLINK_TYPE_CHAR, 40, 30, offsetof(mavlink_rfid_tag_t, tag_ID) }, \
         { "ReadCount", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_rfid_tag_t, ReadCount) }, \
         } \
}


/**
 * @brief Pack a rfid_tag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tag_ID Tag ID
 * @param timestamp time stamp
 * @param RSSI RSSI
 * @param ReadCount read count
 * @param x x position of UAV when reading tag
 * @param y x position of UAV when reading tag
 * @param z x position of UAV when reading tag
 * @param yaw x position of UAV when reading tag
 * @param position_tag  position tag of UAV when reading tag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rfid_tag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *tag_ID, uint64_t timestamp, int32_t RSSI, uint8_t ReadCount, float x, float y, float z, float yaw, uint16_t position_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RFID_TAG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, RSSI);
	_mav_put_float(buf, 12, x);
	_mav_put_float(buf, 16, y);
	_mav_put_float(buf, 20, z);
	_mav_put_float(buf, 24, yaw);
	_mav_put_uint16_t(buf, 28, position_tag);
	_mav_put_uint8_t(buf, 70, ReadCount);
	_mav_put_char_array(buf, 30, tag_ID, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RFID_TAG_LEN);
#else
	mavlink_rfid_tag_t packet;
	packet.timestamp = timestamp;
	packet.RSSI = RSSI;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.position_tag = position_tag;
	packet.ReadCount = ReadCount;
	mav_array_memcpy(packet.tag_ID, tag_ID, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RFID_TAG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
}

/**
 * @brief Pack a rfid_tag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tag_ID Tag ID
 * @param timestamp time stamp
 * @param RSSI RSSI
 * @param ReadCount read count
 * @param x x position of UAV when reading tag
 * @param y x position of UAV when reading tag
 * @param z x position of UAV when reading tag
 * @param yaw x position of UAV when reading tag
 * @param position_tag  position tag of UAV when reading tag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rfid_tag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *tag_ID,uint64_t timestamp,int32_t RSSI,uint8_t ReadCount,float x,float y,float z,float yaw,uint16_t position_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RFID_TAG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, RSSI);
	_mav_put_float(buf, 12, x);
	_mav_put_float(buf, 16, y);
	_mav_put_float(buf, 20, z);
	_mav_put_float(buf, 24, yaw);
	_mav_put_uint16_t(buf, 28, position_tag);
	_mav_put_uint8_t(buf, 70, ReadCount);
	_mav_put_char_array(buf, 30, tag_ID, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RFID_TAG_LEN);
#else
	mavlink_rfid_tag_t packet;
	packet.timestamp = timestamp;
	packet.RSSI = RSSI;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.position_tag = position_tag;
	packet.ReadCount = ReadCount;
	mav_array_memcpy(packet.tag_ID, tag_ID, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RFID_TAG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
}

/**
 * @brief Encode a rfid_tag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rfid_tag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rfid_tag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rfid_tag_t* rfid_tag)
{
	return mavlink_msg_rfid_tag_pack(system_id, component_id, msg, rfid_tag->tag_ID, rfid_tag->timestamp, rfid_tag->RSSI, rfid_tag->ReadCount, rfid_tag->x, rfid_tag->y, rfid_tag->z, rfid_tag->yaw, rfid_tag->position_tag);
}

/**
 * @brief Encode a rfid_tag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rfid_tag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rfid_tag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rfid_tag_t* rfid_tag)
{
	return mavlink_msg_rfid_tag_pack_chan(system_id, component_id, chan, msg, rfid_tag->tag_ID, rfid_tag->timestamp, rfid_tag->RSSI, rfid_tag->ReadCount, rfid_tag->x, rfid_tag->y, rfid_tag->z, rfid_tag->yaw, rfid_tag->position_tag);
}

/**
 * @brief Send a rfid_tag message
 * @param chan MAVLink channel to send the message
 *
 * @param tag_ID Tag ID
 * @param timestamp time stamp
 * @param RSSI RSSI
 * @param ReadCount read count
 * @param x x position of UAV when reading tag
 * @param y x position of UAV when reading tag
 * @param z x position of UAV when reading tag
 * @param yaw x position of UAV when reading tag
 * @param position_tag  position tag of UAV when reading tag
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rfid_tag_send(mavlink_channel_t chan, const char *tag_ID, uint64_t timestamp, int32_t RSSI, uint8_t ReadCount, float x, float y, float z, float yaw, uint16_t position_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RFID_TAG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, RSSI);
	_mav_put_float(buf, 12, x);
	_mav_put_float(buf, 16, y);
	_mav_put_float(buf, 20, z);
	_mav_put_float(buf, 24, yaw);
	_mav_put_uint16_t(buf, 28, position_tag);
	_mav_put_uint8_t(buf, 70, ReadCount);
	_mav_put_char_array(buf, 30, tag_ID, 40);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, buf, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, buf, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
#else
	mavlink_rfid_tag_t packet;
	packet.timestamp = timestamp;
	packet.RSSI = RSSI;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.position_tag = position_tag;
	packet.ReadCount = ReadCount;
	mav_array_memcpy(packet.tag_ID, tag_ID, sizeof(char)*40);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, (const char *)&packet, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, (const char *)&packet, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RFID_TAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rfid_tag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *tag_ID, uint64_t timestamp, int32_t RSSI, uint8_t ReadCount, float x, float y, float z, float yaw, uint16_t position_tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, RSSI);
	_mav_put_float(buf, 12, x);
	_mav_put_float(buf, 16, y);
	_mav_put_float(buf, 20, z);
	_mav_put_float(buf, 24, yaw);
	_mav_put_uint16_t(buf, 28, position_tag);
	_mav_put_uint8_t(buf, 70, ReadCount);
	_mav_put_char_array(buf, 30, tag_ID, 40);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, buf, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, buf, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
#else
	mavlink_rfid_tag_t *packet = (mavlink_rfid_tag_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->RSSI = RSSI;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->yaw = yaw;
	packet->position_tag = position_tag;
	packet->ReadCount = ReadCount;
	mav_array_memcpy(packet->tag_ID, tag_ID, sizeof(char)*40);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, (const char *)packet, MAVLINK_MSG_ID_RFID_TAG_LEN, MAVLINK_MSG_ID_RFID_TAG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RFID_TAG, (const char *)packet, MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RFID_TAG UNPACKING


/**
 * @brief Get field tag_ID from rfid_tag message
 *
 * @return Tag ID
 */
static inline uint16_t mavlink_msg_rfid_tag_get_tag_ID(const mavlink_message_t* msg, char *tag_ID)
{
	return _MAV_RETURN_char_array(msg, tag_ID, 40,  30);
}

/**
 * @brief Get field timestamp from rfid_tag message
 *
 * @return time stamp
 */
static inline uint64_t mavlink_msg_rfid_tag_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field RSSI from rfid_tag message
 *
 * @return RSSI
 */
static inline int32_t mavlink_msg_rfid_tag_get_RSSI(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field ReadCount from rfid_tag message
 *
 * @return read count
 */
static inline uint8_t mavlink_msg_rfid_tag_get_ReadCount(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field x from rfid_tag message
 *
 * @return x position of UAV when reading tag
 */
static inline float mavlink_msg_rfid_tag_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y from rfid_tag message
 *
 * @return x position of UAV when reading tag
 */
static inline float mavlink_msg_rfid_tag_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from rfid_tag message
 *
 * @return x position of UAV when reading tag
 */
static inline float mavlink_msg_rfid_tag_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from rfid_tag message
 *
 * @return x position of UAV when reading tag
 */
static inline float mavlink_msg_rfid_tag_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field position_tag from rfid_tag message
 *
 * @return  position tag of UAV when reading tag
 */
static inline uint16_t mavlink_msg_rfid_tag_get_position_tag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Decode a rfid_tag message into a struct
 *
 * @param msg The message to decode
 * @param rfid_tag C-struct to decode the message contents into
 */
static inline void mavlink_msg_rfid_tag_decode(const mavlink_message_t* msg, mavlink_rfid_tag_t* rfid_tag)
{
#if MAVLINK_NEED_BYTE_SWAP
	rfid_tag->timestamp = mavlink_msg_rfid_tag_get_timestamp(msg);
	rfid_tag->RSSI = mavlink_msg_rfid_tag_get_RSSI(msg);
	rfid_tag->x = mavlink_msg_rfid_tag_get_x(msg);
	rfid_tag->y = mavlink_msg_rfid_tag_get_y(msg);
	rfid_tag->z = mavlink_msg_rfid_tag_get_z(msg);
	rfid_tag->yaw = mavlink_msg_rfid_tag_get_yaw(msg);
	rfid_tag->position_tag = mavlink_msg_rfid_tag_get_position_tag(msg);
	mavlink_msg_rfid_tag_get_tag_ID(msg, rfid_tag->tag_ID);
	rfid_tag->ReadCount = mavlink_msg_rfid_tag_get_ReadCount(msg);
#else
	memcpy(rfid_tag, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RFID_TAG_LEN);
#endif
}
