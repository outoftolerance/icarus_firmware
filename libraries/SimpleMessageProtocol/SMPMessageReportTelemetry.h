#define MESSAGE_REPORT_TELEMETRY_PAYLOAD_LENGTH 15
#define MESSAGE_REPORT_TELEMETRY_PAYLOAD_BYTE_LENGTH 60

typedef struct smpMessageReportTelemetry
{
    FloatUnion_t latitude;
    FloatUnion_t longitude;
    FloatUnion_t altitude;
    FloatUnion_t altitude_ellipsoid;
    FloatUnion_t altitude_relative;
    FloatUnion_t altitude_barometric;
    FloatUnion_t velocity_horizontal;
    FloatUnion_t velocity_vertical;
    FloatUnion_t roll;
    FloatUnion_t pitch;
    FloatUnion_t yaw;
    FloatUnion_t heading;
    FloatUnion_t course;
};

static inline void smpMessageReportTelemetryEncode(uint8_t node_id, uint8_t node_type, smpMessageReportTelemetry& telemetry, hdlcMessage& message)
{
    message.node_id = node_id;
    message.node_type = node_type;
    message.command = MESSAGE_TYPES::MESSAGE_TYPE_REPORT_TELEMETRY;
    message.length = MESSAGE_REPORT_TELEMETRY_PAYLOAD_BYTE_LENGTH;

    int data_position = 0;
    int bytes = 0;

    for (bytes = 0; bytes < sizeof(telemetry.latitude); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.latitude.bytes[bytes];
    }

    data_position += sizeof(telemetry.latitude);

    for (bytes = 0; bytes < sizeof(telemetry.longitude); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.longitude.bytes[bytes];
    }

    data_position += sizeof(telemetry.longitude);

    for (bytes = 0; bytes < sizeof(telemetry.altitude); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.altitude.bytes[bytes];
    }

    data_position += sizeof(telemetry.altitude);

    for (bytes = 0; bytes < sizeof(telemetry.altitude_ellipsoid); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.altitude_ellipsoid.bytes[bytes];
    }

    data_position += sizeof(telemetry.altitude_ellipsoid);

    for (bytes = 0; bytes < sizeof(telemetry.altitude_relative); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.altitude_relative.bytes[bytes];
    }

    data_position += sizeof(telemetry.altitude_relative);

    for (bytes = 0; bytes < sizeof(telemetry.altitude_barometric); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.altitude_barometric.bytes[bytes];
    }

    data_position += sizeof(telemetry.altitude_barometric);

    for (bytes = 0; bytes < sizeof(telemetry.velocity_horizontal); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.velocity_horizontal.bytes[bytes];
    }

    data_position += sizeof(telemetry.velocity_horizontal);

    for (bytes = 0; bytes < sizeof(telemetry.velocity_vertical); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.velocity_vertical.bytes[bytes];
    }

    data_position += sizeof(telemetry.velocity_vertical);

    for (bytes = 0; bytes < sizeof(telemetry.roll); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.roll.bytes[bytes];
    }

    data_position += sizeof(telemetry.roll);

    for (bytes = 0; bytes < sizeof(telemetry.pitch); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.pitch.bytes[bytes];
    }

    data_position += sizeof(telemetry.pitch);

    for (bytes = 0; bytes < sizeof(telemetry.yaw); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.yaw.bytes[bytes];
    }

    data_position += sizeof(telemetry.yaw);

    for (bytes = 0; bytes < sizeof(telemetry.heading); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.heading.bytes[bytes];
    }

    data_position += sizeof(telemetry.heading);

    for (bytes = 0; bytes < sizeof(telemetry.course); bytes++)
    {
        message.payload[data_position + bytes] = telemetry.course.bytes[bytes];
    }

    data_position += sizeof(telemetry.course);
}

static inline void smpMessageReportTelemetryDecode(hdlcMessage& message, smpMessageReportTelemetry& telemetry)
{
    int data_position = 0;
    int bytes = 0;

    //Lat
    for (bytes = 0; bytes < sizeof(telemetry.latitude); bytes++)
    {
        telemetry.latitude.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.latitude);

    //Lon
    for (bytes = 0; bytes < sizeof(telemetry.longitude); bytes++)
    {
        telemetry.longitude.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.longitude);

    //Alt
    for (bytes = 0; bytes < sizeof(telemetry.altitude); bytes++)
    {
        telemetry.altitude.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.altitude);

    //Alt Ellipsoid
    for (bytes = 0; bytes < sizeof(telemetry.altitude_ellipsoid); bytes++)
    {
        telemetry.altitude_ellipsoid.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.altitude_ellipsoid);

    //Alt Rel
    for (bytes = 0; bytes < sizeof(telemetry.altitude_relative); bytes++)
    {
        telemetry.altitude_relative.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.altitude_relative);

    //Alt Baro
    for (bytes = 0; bytes < sizeof(telemetry.altitude_barometric); bytes++)
    {
        telemetry.altitude_barometric.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.altitude_barometric);

    //Vel Hor
    for (bytes = 0; bytes < sizeof(telemetry.velocity_horizontal); bytes++)
    {
        telemetry.velocity_horizontal.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.velocity_horizontal);

    //Vel Vert
    for (bytes = 0; bytes < sizeof(telemetry.velocity_vertical); bytes++)
    {
        telemetry.velocity_vertical.bytes[bytes] = message.payload[data_position + bytes];
    }


    data_position += sizeof(telemetry.velocity_vertical);

    //Roll
    for (bytes = 0; bytes < sizeof(telemetry.roll); bytes++)
    {
        telemetry.roll.bytes[bytes] = message.payload[data_position + bytes];
    }


    data_position += sizeof(telemetry.roll);

    //Pitch
    for (bytes = 0; bytes < sizeof(telemetry.pitch); bytes++)
    {
        telemetry.pitch.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.pitch);

    //Yaw
    for (bytes = 0; bytes < sizeof(telemetry.yaw); bytes++)
    {
        telemetry.yaw.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.yaw);

    //Heading
    for (bytes = 0; bytes < sizeof(telemetry.heading); bytes++)
    {
        telemetry.heading.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.heading);

    //Course
    for (bytes = 0; bytes < sizeof(telemetry.course); bytes++)
    {
        telemetry.course.bytes[bytes] = message.payload[data_position + bytes];
    }

    data_position += sizeof(telemetry.course);
}