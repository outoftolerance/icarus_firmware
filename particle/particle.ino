#include <ParticleConfiguration.h>

#include <SimpleUtils.h>
#include <SimpleLog.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>

Stream& logging_stream = Serial;                            /**< Logging output stream, this is of type Serial_ */
Stream& icarus_stream = Serial1;                            /**< Icarus stream, this is of type HardwareSerial */

SimpleHDLC icarus(icarus_stream, &handleMessageCallback);   /**< HDLC messaging object, linked to message callback */
SimpleLog logger(logging_stream, &rtc, LOG_LEVELS::INFO);   /**< Log object */

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
    SimpleUtils::TelemetryStruct current_telemetry;                 /**< Current telemetry */

    bool cellular_enabled = true;

    current_telemetry.altitude = 0.0;

    while(1)
    {
        icarus.receive();

        //Check current altitude
        if(current_telemetry.altitude <= AIRPLANE_MODE_ALTITUDE)
        {
            if(!cellular_enabled)
            {
                Cellular.on();
                cellular_enabled = true;
            }
        }
        else
        {
            if(cellular_enabled)
            {
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
    logger.event(LOG_LEVELS::DEBUG, "Ignoring heartbeat message.");
}

void handleMessageTelemetryReport(hdlcMessage& message)
{
    logger.event(LOG_LEVELS::INFO, "Received telemetry report message");

    //Check if connected
    if(cellular_enabled && Cellular.ready() && Particle.connected())
    {
        //Publish location report
        Particle.publish("TELEMETRY_REPORT", "yo", LOCATION_REPORT_TTL, PRIVATE);
    }
}