// MESSAGE VFR_HUD PACKING

#define MAVLINK_MSG_ID_VFR_HUD 74

typedef struct __mavlink_vfr_hud_t
{
 uint8_t rotor1; /*< rotor1 pwm*/
 uint8_t rotor2; /*< rotor2 pwm*/
 uint8_t rotor3; /*< rotor3 pwm*/
 uint8_t rotor4; /*< rotor4 pwm*/
 uint8_t rotor5; /*< rotor5 pwm*/
 uint8_t rotor6; /*< rotor6 pwm*/
 uint8_t rotor7; /*< rotor7 pwm*/
 uint8_t rotor8; /*< rotor8 pwm*/
} mavlink_vfr_hud_t;

#define MAVLINK_MSG_ID_VFR_HUD_LEN 8
#define MAVLINK_MSG_ID_74_LEN 8

#define MAVLINK_MSG_ID_VFR_HUD_CRC 111
#define MAVLINK_MSG_ID_74_CRC 111



#define MAVLINK_MESSAGE_INFO_VFR_HUD { \
	"VFR_HUD", \
	8, \
	{  { "rotor1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_vfr_hud_t, rotor1) }, \
         { "rotor2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_vfr_hud_t, rotor2) }, \
         { "rotor3", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_vfr_hud_t, rotor3) }, \
         { "rotor4", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_vfr_hud_t, rotor4) }, \
         { "rotor5", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_vfr_hud_t, rotor5) }, \
         { "rotor6", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_vfr_hud_t, rotor6) }, \
         { "rotor7", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_vfr_hud_t, rotor7) }, \
         { "rotor8", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_vfr_hud_t, rotor8) }, \
         } \
}


/**
 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rotor1 rotor1 pwm
 * @param rotor2 rotor2 pwm
 * @param rotor3 rotor3 pwm
 * @param rotor4 rotor4 pwm
 * @param rotor5 rotor5 pwm
 * @param rotor6 rotor6 pwm
 * @param rotor7 rotor7 pwm
 * @param rotor8 rotor8 pwm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vfr_hud_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t rotor1, uint8_t rotor2, uint8_t rotor3, uint8_t rotor4, uint8_t rotor5, uint8_t rotor6, uint8_t rotor7, uint8_t rotor8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VFR_HUD_LEN];
	_mav_put_uint8_t(buf, 0, rotor1);
	_mav_put_uint8_t(buf, 1, rotor2);
	_mav_put_uint8_t(buf, 2, rotor3);
	_mav_put_uint8_t(buf, 3, rotor4);
	_mav_put_uint8_t(buf, 4, rotor5);
	_mav_put_uint8_t(buf, 5, rotor6);
	_mav_put_uint8_t(buf, 6, rotor7);
	_mav_put_uint8_t(buf, 7, rotor8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VFR_HUD_LEN);
#else
	mavlink_vfr_hud_t packet;
	packet.rotor1 = rotor1;
	packet.rotor2 = rotor2;
	packet.rotor3 = rotor3;
	packet.rotor4 = rotor4;
	packet.rotor5 = rotor5;
	packet.rotor6 = rotor6;
	packet.rotor7 = rotor7;
	packet.rotor8 = rotor8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VFR_HUD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
}

/**
 * @brief Pack a vfr_hud message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rotor1 rotor1 pwm
 * @param rotor2 rotor2 pwm
 * @param rotor3 rotor3 pwm
 * @param rotor4 rotor4 pwm
 * @param rotor5 rotor5 pwm
 * @param rotor6 rotor6 pwm
 * @param rotor7 rotor7 pwm
 * @param rotor8 rotor8 pwm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vfr_hud_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t rotor1,uint8_t rotor2,uint8_t rotor3,uint8_t rotor4,uint8_t rotor5,uint8_t rotor6,uint8_t rotor7,uint8_t rotor8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VFR_HUD_LEN];
	_mav_put_uint8_t(buf, 0, rotor1);
	_mav_put_uint8_t(buf, 1, rotor2);
	_mav_put_uint8_t(buf, 2, rotor3);
	_mav_put_uint8_t(buf, 3, rotor4);
	_mav_put_uint8_t(buf, 4, rotor5);
	_mav_put_uint8_t(buf, 5, rotor6);
	_mav_put_uint8_t(buf, 6, rotor7);
	_mav_put_uint8_t(buf, 7, rotor8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VFR_HUD_LEN);
#else
	mavlink_vfr_hud_t packet;
	packet.rotor1 = rotor1;
	packet.rotor2 = rotor2;
	packet.rotor3 = rotor3;
	packet.rotor4 = rotor4;
	packet.rotor5 = rotor5;
	packet.rotor6 = rotor6;
	packet.rotor7 = rotor7;
	packet.rotor8 = rotor8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VFR_HUD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
}

/**
 * @brief Encode a vfr_hud struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vfr_hud_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vfr_hud_t* vfr_hud)
{
	return mavlink_msg_vfr_hud_pack(system_id, component_id, msg, vfr_hud->rotor1, vfr_hud->rotor2, vfr_hud->rotor3, vfr_hud->rotor4, vfr_hud->rotor5, vfr_hud->rotor6, vfr_hud->rotor7, vfr_hud->rotor8);
}

/**
 * @brief Encode a vfr_hud struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vfr_hud_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vfr_hud_t* vfr_hud)
{
	return mavlink_msg_vfr_hud_pack_chan(system_id, component_id, chan, msg, vfr_hud->rotor1, vfr_hud->rotor2, vfr_hud->rotor3, vfr_hud->rotor4, vfr_hud->rotor5, vfr_hud->rotor6, vfr_hud->rotor7, vfr_hud->rotor8);
}

/**
 * @brief Send a vfr_hud message
 * @param chan MAVLink channel to send the message
 *
 * @param rotor1 rotor1 pwm
 * @param rotor2 rotor2 pwm
 * @param rotor3 rotor3 pwm
 * @param rotor4 rotor4 pwm
 * @param rotor5 rotor5 pwm
 * @param rotor6 rotor6 pwm
 * @param rotor7 rotor7 pwm
 * @param rotor8 rotor8 pwm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vfr_hud_send(mavlink_channel_t chan, uint8_t rotor1, uint8_t rotor2, uint8_t rotor3, uint8_t rotor4, uint8_t rotor5, uint8_t rotor6, uint8_t rotor7, uint8_t rotor8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VFR_HUD_LEN];
	_mav_put_uint8_t(buf, 0, rotor1);
	_mav_put_uint8_t(buf, 1, rotor2);
	_mav_put_uint8_t(buf, 2, rotor3);
	_mav_put_uint8_t(buf, 3, rotor4);
	_mav_put_uint8_t(buf, 4, rotor5);
	_mav_put_uint8_t(buf, 5, rotor6);
	_mav_put_uint8_t(buf, 6, rotor7);
	_mav_put_uint8_t(buf, 7, rotor8);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, buf, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, buf, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
#else
	mavlink_vfr_hud_t packet;
	packet.rotor1 = rotor1;
	packet.rotor2 = rotor2;
	packet.rotor3 = rotor3;
	packet.rotor4 = rotor4;
	packet.rotor5 = rotor5;
	packet.rotor6 = rotor6;
	packet.rotor7 = rotor7;
	packet.rotor8 = rotor8;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, (const char *)&packet, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, (const char *)&packet, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_VFR_HUD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vfr_hud_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t rotor1, uint8_t rotor2, uint8_t rotor3, uint8_t rotor4, uint8_t rotor5, uint8_t rotor6, uint8_t rotor7, uint8_t rotor8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, rotor1);
	_mav_put_uint8_t(buf, 1, rotor2);
	_mav_put_uint8_t(buf, 2, rotor3);
	_mav_put_uint8_t(buf, 3, rotor4);
	_mav_put_uint8_t(buf, 4, rotor5);
	_mav_put_uint8_t(buf, 5, rotor6);
	_mav_put_uint8_t(buf, 6, rotor7);
	_mav_put_uint8_t(buf, 7, rotor8);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, buf, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, buf, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
#else
	mavlink_vfr_hud_t *packet = (mavlink_vfr_hud_t *)msgbuf;
	packet->rotor1 = rotor1;
	packet->rotor2 = rotor2;
	packet->rotor3 = rotor3;
	packet->rotor4 = rotor4;
	packet->rotor5 = rotor5;
	packet->rotor6 = rotor6;
	packet->rotor7 = rotor7;
	packet->rotor8 = rotor8;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, (const char *)packet, MAVLINK_MSG_ID_VFR_HUD_LEN, MAVLINK_MSG_ID_VFR_HUD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, (const char *)packet, MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE VFR_HUD UNPACKING


/**
 * @brief Get field rotor1 from vfr_hud message
 *
 * @return rotor1 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field rotor2 from vfr_hud message
 *
 * @return rotor2 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field rotor3 from vfr_hud message
 *
 * @return rotor3 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field rotor4 from vfr_hud message
 *
 * @return rotor4 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field rotor5 from vfr_hud message
 *
 * @return rotor5 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field rotor6 from vfr_hud message
 *
 * @return rotor6 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field rotor7 from vfr_hud message
 *
 * @return rotor7 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field rotor8 from vfr_hud message
 *
 * @return rotor8 pwm
 */
static inline uint8_t mavlink_msg_vfr_hud_get_rotor8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a vfr_hud message into a struct
 *
 * @param msg The message to decode
 * @param vfr_hud C-struct to decode the message contents into
 */
static inline void mavlink_msg_vfr_hud_decode(const mavlink_message_t* msg, mavlink_vfr_hud_t* vfr_hud)
{
#if MAVLINK_NEED_BYTE_SWAP
	vfr_hud->rotor1 = mavlink_msg_vfr_hud_get_rotor1(msg);
	vfr_hud->rotor2 = mavlink_msg_vfr_hud_get_rotor2(msg);
	vfr_hud->rotor3 = mavlink_msg_vfr_hud_get_rotor3(msg);
	vfr_hud->rotor4 = mavlink_msg_vfr_hud_get_rotor4(msg);
	vfr_hud->rotor5 = mavlink_msg_vfr_hud_get_rotor5(msg);
	vfr_hud->rotor6 = mavlink_msg_vfr_hud_get_rotor6(msg);
	vfr_hud->rotor7 = mavlink_msg_vfr_hud_get_rotor7(msg);
	vfr_hud->rotor8 = mavlink_msg_vfr_hud_get_rotor8(msg);
#else
	memcpy(vfr_hud, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_VFR_HUD_LEN);
#endif
}
