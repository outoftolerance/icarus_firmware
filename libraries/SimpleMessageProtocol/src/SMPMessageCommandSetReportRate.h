#define MESSAGE_COMMAND_SET_REPORT_RATE_PAYLOAD_LENGTH 2
#define MESSAGE_COMMAND_SET_REPORT_RATE_PAYLOAD_BYTE_LENGTH 5

typedef struct smpMessageCommandSetReportRate
{
    UInt8Union_t report;   /*< MESSAGE_TYPE id rate being set */
    FloatUnion_t rate;      /*< Desired message rate in Hz */
};

static inline void smpMessageCommandSetReportRateEncode(uint8_t node_id, uint8_t node_type, smpMessageCommandSetReportRate& command, hdlcMessage& message)
{
    message.node_id = node_id;
    message.node_type = node_type;
    message.command = MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_REPORT_RATE;
    message.length = MESSAGE_COMMAND_SET_REPORT_RATE_PAYLOAD_BYTE_LENGTH;

    int data_position = 0;
    int bytes = 0;

    for (bytes = 0; bytes < sizeof(command.report.value); bytes++)
    {
        message.payload[data_position + bytes] = command.report.bytes[bytes];
    }

    data_position += sizeof(command.report.value);

    for (bytes = 0; bytes < sizeof(command.rate.value); bytes++)
    {
        message.payload[data_position + bytes] = command.rate.bytes[bytes];
    }
}

static inline void smpMessageCommandSetReportRateDecode(hdlcMessage& message, smpMessageCommandSetReportRate& command)
{
    int data_position = 0;
    int bytes = 0;

    for (bytes = 0; bytes < sizeof(command.report.value); bytes++)
    {
        command.report.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(command.report.value);

    for (bytes = 0; bytes < sizeof(command.rate.value); bytes++)
    {
        command.rate.bytes[bytes] = message.payload[data_position + bytes];
    }
}