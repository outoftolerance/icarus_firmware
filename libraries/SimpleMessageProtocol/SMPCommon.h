/**
 * @brief      Defines different message types that can be sent
 */
enum MESSAGE_TYPES : uint8_t {
	/**
	 * Report Messages
	 */
    MESSAGE_TYPE_HEARTBEAT = 0,
    MESSAGE_TYPE_REPORT_TELEMETRY,

    /**
     * Command Messages
     */
	MESSAGE_TYPE_COMMAND_ARM,
    MESSAGE_TYPE_COMMAND_DISARM,
    MESSAGE_TYPE_COMMAND_SET_STATE,
    MESSAGE_TYPE_COMMAND_SET_TRACKER_LOCATION,
    MESSAGE_TYPE_COMMAND_SET_TRACKER_TARGET_LOCATION,
    MESSAGE_TYPE_COMMAND_SET_TRACKER_POSE,
    MESSAGE_TYPE_COMMAND_SET_REPORT_RATE,
    MESSAGE_TYPE_COMMAND_REQUEST_REPORT,

    /**
     * Protocol Level Messages
     */
    MESSAGE_TYPE_PROTO_ACK,
    MESSAGE_TYPE_PROTO_NACK
};

enum NODE_TYPES : uint8_t {
    NODE_TYPE_GROUNDSTATION = 0,
    NODE_TYPE_BALLOON,
    NODE_TYPE_TRACKER
};

/**
 * @brief Union for float data
 */
typedef union
{
    float value;
    uint8_t bytes[4];
} FloatUnion_t;

/**
 * @brief Union for uint16 data
 */
typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} UInt16Union_t;

/**
 * @brief Union for uint8 data
 */
typedef union
{
    uint8_t value;
    uint8_t bytes[1];
} UInt8Union_t;