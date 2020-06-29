#include <ParticleConfiguration.h>

#include <SimpleUtils.h>
#include <SimpleLog.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>

Stream& logging_stream = Serial;                            /**< Logging output stream, this is of type Serial_ */
Stream& icarus_stream = Serial1;                            /**< Icarus stream, this is of type HardwareSerial */

SimpleHDLC icarus(icarus_stream, &handleMessageCallback);   /**< HDLC messaging object, linked to message callback */
SimpleLog logger(logging_stream, LOG_LEVELS::INFO);         /**< Log object */

SimpleUtils::TelemetryStruct current_telemetry;             /**< Current telemetry */

void setup() 
{
    //Start debug serial port
    logger.init();
    logger.event(LOG_LEVELS::INFO, "Particle systems starting...");

    //Start radio modem Serial port
    logger.event(LOG_LEVELS::INFO, "Starting Icarus serial port...");
    static_cast<HardwareSerial&>(icarus_stream).begin(57600);
    logger.event(LOG_LEVELS::INFO, "Done!");

    logger.event(LOG_LEVELS::INFO, "Finished initialisation, starting program!");
}

void loop() 
{
    //Set defaults
    bool cellular_enabled = true;
    current_telemetry.altitude = 0.0;

    while(1)
    {
        //Get messages from serial interface
        icarus.receive();

        //Check current altitude
        if(current_telemetry.altitude <= AIRPLANE_MODE_ALTITUDE)
        {
            logger.event(LOG_LEVELS::INFO, "Lower than Airplane Mode altitude.");
            if(!cellular_enabled)
            {
                logger.event(LOG_LEVELS::INFO, "Enabling cellular module!");

                Cellular.on();
                cellular_enabled = true;
            }
        }
        else
        {
            logger.event(LOG_LEVELS::INFO, "Enabling cellular module!");
            
            if(cellular_enabled)
            {
                logger.event(LOG_LEVELS::INFO, "Disabling cellular module!");

                Cellular.off();
                cellular_enabled = false;
            }
        }
    }
}

void handleMessageCallback(hdlcMessage message)
{
    logger.event(LOG_LEVELS::DEBUG, "Received a message!");

    switch(message.command)
    {
        case MESSAGE_TYPES::MESSAGE_TYPE_HEARTBEAT:
            logger.event(LOG_LEVELS::DEBUG, "Received message: Heartbeat.");
            handleMessageHeartbeat(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_REPORT_TELEMETRY:
            logger.event(LOG_LEVELS::DEBUG, "Received message: Position Report.");
            handleMessageTelemetryReport(message);
            break;
    }
}

void handleMessageHeartbeat(hdlcMessage& message)
{
    logger.event(LOG_LEVELS::DEBUG, "Received heartbeat message!");
}

void handleMessageTelemetryReport(hdlcMessage& message)
{
    logger.event(LOG_LEVELS::INFO, "Received telemetry report message");

    //Decode and parse for target location
    smpMessageReportTelemetry report;
    smpMessageReportTelemetryDecode(message, report);

    //Extract altitude and other telemetry
    current_telemetry.latitude = report.latitude.value;
    current_telemetry.longitude = report.longitude.value;
    current_telemetry.altitude = report.altitude.value;
    current_telemetry.altitude_ellipsoid = report.altitude_ellipsoid.value;
    current_telemetry.altitude_relative = report.altitude_relative.value;
    current_telemetry.altitude_barometric = report.altitude_barometric.value;
    current_telemetry.velocity_horizontal = report.velocity_horizontal.value;
    current_telemetry.velocity_vertical = report.velocity_vertical.value;
    current_telemetry.roll = report.roll.value;
    current_telemetry.pitch = report.pitch.value;
    current_telemetry.yaw = report.yaw.value;
    current_telemetry.heading = report.heading.value;
    current_telemetry.course = report.course.value;

    //Check if connected
    if(Cellular.ready() && Particle.connected())
    {
        //Publish location report
        Particle.publish("TELEMETRY_REPORT", "yo", LOCATION_REPORT_TTL, PRIVATE);
    }
}