#define MESSAGE_PROTO_NACK_PAYLOAD_LENGTH 1
#define MESSAGE_PROTO_NACK_PAYLOAD_BYTE_LENGTH 1

typedef struct smpMessageProtoNack
{
    UInt8Union_t command;
};

static inline void smpMessageProtoNackEncode(uint8_t node_id, uint8_t node_type, smpMessageProtoNack& nack, hdlcMessage& message)
{
    message.node_id = node_id;
    message.node_type = node_type;
    message.command = MESSAGE_TYPES::MESSAGE_TYPE_PROTO_NACK;
    message.length = MESSAGE_PROTO_NACK_PAYLOAD_BYTE_LENGTH;

    int data_position = 0;
    int bytes = 0;

    for (bytes = 0; bytes < sizeof(nack.command); bytes++)
    {
        message.payload[data_position + bytes] = nack.command.bytes[bytes];
    }
}

static inline void smpMessageProtoNackDecode(hdlcMessage& message, smpMessageProtoNack& nack)
{
    int data_position = 0;
    int bytes = 0;

    for (bytes = 0; bytes < sizeof(nack.command); bytes++)
    {
        nack.command.bytes[bytes] = message.payload[data_position + bytes];
    }
}