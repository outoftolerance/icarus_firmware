#include "wiring_private.h" // For ATSAMD M0 pinPeripheral() function
#include <Wire.h>
#include <Math.h>
#include <PID_v1.h>

#include <Adafruit_PWMServoDriver.h>
#include <TinyGPS++.h>
#include <SimpleServo.h>
#include <Timer.h>
#include <Log.h>
#include <Telemetry.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>
#include <TinyGPS++.h>

#define PAN_SERVO_CHANNEL 5
#define TILT_SERVO_CHANNEL 4

#define PAN_SERVO_PWM_MIN  150  // this is the 'minimum' pulse length count (out of 4096) Left
#define PAN_SERVO_PWM_MAX  600  // this is the 'maximum' pulse length count (out of 4096) Right
#define TILT_SERVO_PWM_MIN  260 // Vertical
#define TILT_SERVO_PWM_MAX  470 // Horizontal

/*
 * Creating some new Serial ports using M0 SERCOM for peripherals
 *
 * Notes for Adafruit Feather M0 pre-defined Serial ports:
 *     - Serial goes to USB port interface (PA24, PA25)
 *     - Serial1 is broken out on the board and uses pins 1/PA10 (TX), 0/PA11 (RX)
 *     - Serial5 is on pins 30/PB22 (TX), 31/PB23 (RX) but not exposed on the board
 * 
 * We are adding the following:
 *     - Serial2 on pins 10/PA18 (TX), 11/PA16 (RX)
 *     - Serial3 on pins 4/PA08 (TX), 3/PA09 (RX)
 */
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);    /**< Creating a second serial port using SERCOM1 */

/*
 * @brief Handler function for SERCOM1 (serial port 2)
 */
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

Stream& logging_output_stream = Serial;             /**< Logging output stream, this is of type Serial_ */
Stream& command_input_stream = Serial;              /**< Message and command interface stream */
Stream& gps_input_stream = Serial1;                 /**< GPS device input stream, this is of type HardwareSerial */
Stream& radio_input_output_stream = Serial2;        /**< Radio input output stream, this is of type HardwareSerial */

SimpleHDLC usb(command_input_stream, &handleMessageCallback);                               /**< HDLC messaging object, linked to message callback */
SimpleHDLC radio(radio_input_output_stream, &handleMessageCallback);                        /**< HDLC messaging object, linked to message callback */

Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();                           /**< Adafruit servo driver object */
SimpleServo tilt_servo(TILT_SERVO_PWM_MAX, TILT_SERVO_PWM_MIN, TILT_SERVO_CHANNEL, &servo_driver);
SimpleServo pan_servo(PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX, PAN_SERVO_CHANNEL, &servo_driver);

Log logger(logging_output_stream, LOG_LEVELS::INFO);                                        /**< Log object */
Telemetry telemetry(&gps_input_stream);                                                     /**< Telemetry object */
uint8_t node_id_ = 2;
uint8_t node_type_ = NODE_TYPES::NODE_TYPE_TRACKER;

uint8_t target_node_id = 1;
SimpleUtils::TelemetryStruct target_location;

Timer timer_execution_led;            /**< Timer sets interval between run led blinks */
Timer timer_telemetry_check;          /**< Timer sets interval between telemetry checks */
Timer timer_servo_update_delay;       /**< Timer sets interval between position updates */

/**
 * @brief System setup function
 * @details Initialises all system componenets at start-up
 */
void setup() {
    //Sleep until debug can connect
    //while(!Serial);

    //Setup pin modes
    pinMode(LED_BUILTIN, OUTPUT);

    //Start logger
    logger.init();
    logger.event(LOG_LEVELS::INFO, "HAB Tracker systems starting...");

    //Start command interface over USB
    logger.event(LOG_LEVELS::INFO, "Starting USB serial interface...");
    //static_cast<HardwareSerial&>(command_input_stream).begin(57600);
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Start radio modem Serial port
    logger.event(LOG_LEVELS::INFO, "Starting radio modem serial port...");
    static_cast<HardwareSerial&>(radio_input_output_stream).begin(57600);
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Initialise the telemetry system
    logger.event(LOG_LEVELS::INFO, "Initialising Telemetry subsystem...");
    if(!telemetry.init())
    {
        logger.event(LOG_LEVELS::FATAL, "Failed to initialise Telemetry subsystem!");
        stop();
    }
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Initialize PWM driver and servos
    logger.event(LOG_LEVELS::INFO, "Starting PWM Driver...");
    servo_driver.begin();
    servo_driver.setPWMFreq(SERVO_PWM_FREQUENCY);
    logger.event(LOG_LEVELS::INFO, "Done!");

    logger.event(LOG_LEVELS::INFO, "Finished initialisation, starting program!");
}

/**
 * @brief Main program loop
 * @details Called after setup() function, loops inifiteley, everything happens here
 */
void loop() {
    timer_execution_led.setInterval(1000);
    timer_telemetry_check.setInterval(5);
    
    SimpleUtils::TelemetryStruct current_telemetry;                      /**< Current telemetry */

    timer_execution_led.start();
    timer_telemetry_check.start();
    timer_servo_update_delay.start();

    double target_distance, tilt_setpoint, pan_setpoint;

    tilt_servo.start();
    pan_servo.start();

	while(1)
	{
		//Get messages from HDLC interfaces
        usb.receive();
        radio.receive();

        //Telemetry Update
        if(timer_telemetry_check.check())
        {
            //Get latest telemetry
            logger.event(LOG_LEVELS::DEBUG, "Getting update from Telemetry subsystem.");

            if(!telemetry.get(current_telemetry))
            {
                logger.event(LOG_LEVELS::ERROR, "Failed to get update from Telemetry subsystem!");
            }
            else
            {
                logger.event(LOG_LEVELS::DEBUG, "Telemetry updated completed.");
                timer_telemetry_check.reset();
            }
        }

		//Calculate bearing and azimuth to target
        target_distance = TinyGPSPlus::distanceBetween(current_telemetry.latitude, current_telemetry.longitude, target_location.latitude, target_location.longitude);
        tilt_setpoint = calcAzimuthTo(current_telemetry.altitude, target_location.altitude, target_distance);
        pan_setpoint = TinyGPSPlus::courseTo(current_telemetry.latitude, current_telemetry.longitude, target_location.latitude, target_location.longitude);

		//Move servos until at target
        tilt_servo.setTargetAngle(tilt_setpoint);
        tilt_servo.move();
        
        pan_servo.setTargetAngle(pan_setpoint);
        pan_servo.move();

        //Execution LED indicator blinkies
        if(timer_execution_led.check())
        {
            if(digitalRead(LED_BUILTIN) == HIGH)
            {
                digitalWrite(LED_BUILTIN, LOW);
            }
            else
            {
                digitalWrite(LED_BUILTIN, HIGH);
            }

            //Send Heartbeat message
            sendHeartbeat(0);

            //Print a bunch of debug information
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Latitude    ", current_telemetry.latitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Longitude   ", current_telemetry.longitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Altitude    ", current_telemetry.altitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Course      ", current_telemetry.course);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Roll        ", current_telemetry.roll);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Pitch       ", current_telemetry.pitch);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Yaw         ", current_telemetry.yaw);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Heading     ", current_telemetry.heading);
            logger.event(LOG_LEVELS::DEBUG, "Current Baro Altitude   ", current_telemetry.altitude_barometric);
            logger.event(LOG_LEVELS::DEBUG, "Current Baro Pressure   ", current_telemetry.pressure);
            logger.event(LOG_LEVELS::DEBUG, "Current Baro Temperature", current_telemetry.temperature);

            logger.event(LOG_LEVELS::DEBUG, "Current Target Latitude  ", target_location.latitude);
            logger.event(LOG_LEVELS::DEBUG, "Current Target Longitude ", target_location.longitude);
            logger.event(LOG_LEVELS::DEBUG, "Current Target Altitude   ", target_location.altitude);

            logger.event(LOG_LEVELS::DEBUG, "Current Target Distance   ", target_distance);
            logger.event(LOG_LEVELS::DEBUG, "Current Tilt Setpoint     ", tilt_setpoint);
            logger.event(LOG_LEVELS::DEBUG, "Current Pan Setpoint      ", pan_setpoint);

            timer_execution_led.reset();
        }
	}
}

float calcAzimuthTo(float base_altitude, float target_altitude, float target_horizontal_distance)
{
	float altitude_difference = target_altitude - base_altitude;

	return atan2(altitude_difference, target_horizontal_distance) * 180 / M_PI;
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
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_TRACKER_TARGET_LOCATION:
            logger.event(LOG_LEVELS::DEBUG, "Received a command to set tracker target.");
            handleMessageSetTrackerTargetLocation(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_TRACKER_POSE:
            logger.event(LOG_LEVELS::DEBUG, "Received a command to set tracker pose.");
            handleMessageSetTrackerPose(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_REQUEST_REPORT:
            logger.event(LOG_LEVELS::DEBUG, "Received a command to request a report.");
            handleMessageRequestReport(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_PROTO_ACK:
            logger.event(LOG_LEVELS::DEBUG, "Received acknowledgement.");
            handleMessageProtoAck(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_PROTO_NACK:
            logger.event(LOG_LEVELS::DEBUG, "Received non-acknowledgement.");
            handleMessageProtoNack(message);
            break;
    }
}

void handleMessageHeartbeat(hdlcMessage& message)
{
    logger.event(LOG_LEVELS::DEBUG, "Ignoring heartbeat message.");
}

void handleMessageTelemetryReport(hdlcMessage& message)
{
    //Is this a telemetry message from the target?
    if(message.node_id == target_node_id)
    {
        logger.event(LOG_LEVELS::INFO, "Received telemetry from target! Updating target location.");

        //Decode and parse for target location
        smpMessageReportTelemetry report;
        smpMessageReportTelemetryDecode(message, report);

        target_location.latitude = report.latitude.value;
        target_location.longitude = report.longitude.value;
        target_location.altitude = report.altitude.value;
    }
    else
    {
        logger.event(LOG_LEVELS::WARNING, "Ignoring telemetry report message.");
    }
}

void handleMessageSetTrackerTargetLocation(hdlcMessage message)
{
    target_location.latitude = 0;
    target_location.longitude = 0;
    target_location.altitude = 0;
}

void handleMessageSetTrackerPose(hdlcMessage message)
{
    
}

void handleMessageRequestReport(hdlcMessage message)
{
    
}

void handleMessageProtoAck(hdlcMessage& message)
{

}

void handleMessageProtoNack(hdlcMessage& message)
{

}

void sendHeartbeat(uint8_t state)
{
    hdlcMessage message;
    smpMessageHeartbeat heartbeat;

    heartbeat.state.value = state;

    smpMessageHeartbeatEncode(node_id_, node_type_, heartbeat, message);

    radio.send(message);
}

void sendReportTelemetry(SimpleUtils::TelemetryStruct& telemetry)
{
    hdlcMessage message;
    smpMessageReportTelemetry telemetry_report;

    telemetry_report.latitude.value = telemetry.latitude;
    telemetry_report.longitude.value = telemetry.longitude;
    telemetry_report.altitude.value = telemetry.altitude;
    telemetry_report.altitude_ellipsoid.value = telemetry.altitude_ellipsoid;
    telemetry_report.altitude_relative.value = telemetry.altitude_relative;
    telemetry_report.altitude_barometric.value = telemetry.altitude_barometric;
    telemetry_report.velocity_horizontal.value = telemetry.velocity_horizontal;
    telemetry_report.velocity_vertical.value = telemetry.velocity_vertical;
    telemetry_report.roll.value = telemetry.roll;
    telemetry_report.pitch.value = telemetry.pitch;
    telemetry_report.heading.value = telemetry.heading;
    telemetry_report.course.value = telemetry.course;

    smpMessageReportTelemetryEncode(node_id_, node_type_, telemetry_report, message);

    usb.send(message);
}

void stop()
{
    while(1)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50); 
    }
}
