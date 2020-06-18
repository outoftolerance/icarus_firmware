#ifndef Telemetry_h
#define Telemetry_h

#include <SimpleUtils.h>
#include <Timer.h>
#include <RTClib.h>
#include <Buffer.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Imu.h>
#include <Adafruit_BME280.h>
#include <SimpleAHRS.h>
#include <SimpleUtils.h>
#include <Log.h>

#define GPS_SERIAL_BAUD 9600
#define GPS_SERIAL_BUFFER_SIZE 64
#define SENSOR_UPDATE_INTERVAL 10   /*< Interval between sensor updates in ms*/
#define SEALEVELPRESSURE_HPA (1013.25)

/**
 * @brief      Telemetry class definition. Class interacts with all sensors and gives access to sensor data in a useful way
 */
class Telemetry
{
    public:
        /*
         * @brief      Telemetry class constructor without GPS
         */
        Telemetry(Log* logger);

        /**
         * @brief      Telemetry class constructor with GPS
         * @param      gps_stream  Pointer to the Stream object for the GPS serial port
         */
        Telemetry(Log* logger, Stream* gps_stream);

        /**
         * @brief      Initialises all variables and objects to their default value/state
         */
        bool init();

        /**
         * @brief      Updates all sensors and filters
         */
        void update();

        /**
         * @brief      Returns current telemetry of system
         * @param      Pointer to variable to output data to
         * @return     Boolean success/fail indicator
         */
        bool get(SimpleUtils::TelemetryStruct& telemetry);

        /**
         * @brief      Gets raw accelerometer data
         * @param      Pointer to variable to output data to
         * @return     Boolean success/fail indicator
         */
        bool getAccelerometerRaw(SimpleUtils::AxisData& accelerometer);

        /**
         * @brief      Gets the raw gyroscope data
         * @param      Pointer to variable to output data to
         * @return     Boolean success/fail indicator
         */
        bool getGyroscopeRaw(SimpleUtils::AxisData& gyroscope);

        /**
         * @brief      Gets the raw magnetometer data
         * @param      Pointer to variable to output data to
         * @return     Boolean success/fail indicator
         */
        bool getMagnetometerRaw(SimpleUtils::AxisData& magnetometer);

        /**
         * @brief      Gets the raw barometer data
         * @param      Pointer to variable to output data to
         * @return     Boolean success/fail indicator
         */
        bool getBarometerRaw(float& data);

        /**
         * @brief      Gets the latest gps serial chars
         * @param      string  Pointer to the output string
         * @return     Number of chars returned
         */
        int getGpsString(char string[], const unsigned int string_length);

        /**
         * @brief      Gets the gps fix status
         *
         * @return     The gps fix status, true if fix, false if not
         */
        bool getGpsFixStatus();

        /**
         * @brief      Returns the horizontal precision of the GPS
         *
         * @return     The gps hdop in meters floating point
         */
        float getGpsHdop();

        /**
         * @brief      Gets the date and time from GPS unit
         *
         * @return     The gps date time as seconds since unix epoch
         */
        uint32_t getGpsUnixTime();

        /**
         * @brief      Resets the base altitude to current altitude
         *
         * @return     True on successful update
         */
        bool resetBaseAltitude();

    private:
        Log* logger_;                                   /**< Log object */
        TinyGPSPlus gps_;                               /**< Defines Tiny GPS object */
        Stream* gps_serial_;                            /**< Defines Stream object for GPS device serial port */
        Buffer* gps_serial_buffer_;                     /**< Buffer to store received GPS serial data in for sending out to other devices */
        int gps_fix_pin_;                               /**< Pin that senses GPS fix status */
        bool gps_fix_status_;                           /**< Current fix status of the GPS unit */
        bool altitude_base_is_set_;                     /**< Indicates if the base altitude is set or not yet */
        float altitude_base_;                           /**< Altitude the system was initialized at, used to calculate relative altitude */
        float altitude_gps_previous_;                   /**< Previous GPS altitude, used for vertical velocity estimation */
        long millis_altitude_gps_previous_;             /**< The timestamp of the previous GPS altitude reading */

        Timer sensor_update_timer_;
        Imu imu_;
        Adafruit_BME280 baro_;
        SimpleAHRS ahrs_;
};

#endif
