#include "wiring_private.h" // For ATSAMD M0 pinPeripheral() function
#include <Wire.h>
#include <Math.h>

#include <TrackerConfiguration.h>
#include <Telemetry.h>

#include <SimpleTimer.h>
#include <SimpleLog.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>

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

Stream& logging_output_stream = Serial;                     /**< Logging output stream, this is of type Serial_ */

SimpleLog logger(logging_output_stream, LOG_LEVELS::INFO);  /**< Log object */
Telemetry telemetry(&logger, NULL, false);                  /**< Telemetry object */

SimpleTimer timer_execution_led;                            /**< Timer sets interval between run led blinks */
SimpleTimer timer_telemetry_update;                         /**< Timer sets interval between telemetry updates */
SimpleTimer timer_telemetry_check;                          /**< Timer sets interval between telemetry checks */
SimpleTimer timer_stop_log_interval;                        /**< Timer sets interval between log output in stop function */

/**
 * @brief System setup function
 * @details Initialises all system componenets at start-up
 */
void setup() {
    //Sleep until debug can connect
    while(!Serial);

    //Setup pin modes
    pinMode(LED_BUILTIN, OUTPUT);

    //Start logger
    logger.init();
    logger.event(LOG_LEVELS::INFO, "HAB Tracker systems starting...");

    //Start command interface over USB
    logger.event(LOG_LEVELS::INFO, "Starting USB serial interface...");
    //static_cast<HardwareSerial&>(command_input_stream).begin(57600);
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Initialise the telemetry system
    logger.event(LOG_LEVELS::INFO, "Initialising Telemetry subsystem...");
    if(!telemetry.init())
    {
        logger.event(LOG_LEVELS::FATAL, "Failed to initialise Telemetry subsystem!");
        stop("Failed to initialise Telemetry subsystem!");
    }
    logger.event(LOG_LEVELS::INFO, "Done!");

    logger.event(LOG_LEVELS::INFO, "Finished initialisation, starting program!");
}

/**
 * @brief Main program loop
 * @details Called after setup() function, loops inifiteley, everything happens here
 */
void loop() {
    SimpleUtils::TelemetryStruct current_telemetry;             /**< Current telemetry */

    timer_execution_led.setInterval(500);                       /**< LED blink interval */
    timer_telemetry_update.setInterval(SENSOR_UPDATE_INTERVAL); /**< Telemetry update interval */
    timer_telemetry_check.setInterval(SENSOR_UPDATE_INTERVAL);  /**< Telemetry check interval */

    // Start the program timers
    timer_execution_led.start();
    timer_telemetry_update.start();
    timer_telemetry_check.start();

    // Program loop
	while(1)
	{
        // Update telemetry
        if(timer_telemetry_update.check())
        {
            telemetry.update();
            timer_telemetry_update.reset();
        }

        // Get latest telemetry
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

                SimpleUtils::AxisData accel;
                SimpleUtils::AxisData gyro;
                SimpleUtils::AxisData mag;

                telemetry.getAccelerometerRaw(accel);
                telemetry.getGyroscopeRaw(gyro);
                telemetry.getMagnetometerRaw(mag);

                // 'Raw' values to match expectation of MotionCal
                Serial.print("Raw:");
                Serial.print(int(accel.x*8192/9.8)); Serial.print(",");
                Serial.print(int(accel.y*8192/9.8)); Serial.print(",");
                Serial.print(int(accel.z*8192/9.8)); Serial.print(",");
                Serial.print(int(gyro.x*57.29577793F*16)); Serial.print(",");
                Serial.print(int(gyro.y*57.29577793F*16)); Serial.print(",");
                Serial.print(int(gyro.z*57.29577793F*16)); Serial.print(",");
                Serial.print(int(mag.x*10)); Serial.print(",");
                Serial.print(int(mag.y*10)); Serial.print(",");
                Serial.print(int(mag.z*10)); Serial.println("");

                // unified data
                Serial.print("Uni:");
                Serial.print(accel.x); Serial.print(",");
                Serial.print(accel.y); Serial.print(",");
                Serial.print(accel.z); Serial.print(",");
                Serial.print(gyro.x, 4); Serial.print(",");
                Serial.print(gyro.y, 4); Serial.print(",");
                Serial.print(gyro.z, 4); Serial.print(",");
                Serial.print(mag.x); Serial.print(",");
                Serial.print(mag.y); Serial.print(",");
                Serial.print(mag.z); Serial.println("");

                timer_telemetry_check.reset();
            }
        }

        // Execution LED indicator blinkies
        if(timer_execution_led.check())
        {
            // Blink LED
            if(digitalRead(LED_BUILTIN) == HIGH)
            {
                digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(LED_STATUS_B, LOW);
            }
            else
            {
                digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(LED_STATUS_B, HIGH);
            }

            timer_execution_led.reset();
        }
	}
}

void stop(const char stop_reason[])
{
    timer_stop_log_interval.setInterval(1000);  /**< Interval to print log messages in stop function */
    timer_stop_log_interval.start(); /**< Start the timer */

    while(1)
    {
        if(timer_stop_log_interval.check())
        {
            logger.event(LOG_LEVELS::FATAL, stop_reason);
            timer_stop_log_interval.reset();
        }

        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50); 
    }
}
