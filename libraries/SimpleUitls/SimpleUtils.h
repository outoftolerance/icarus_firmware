#ifndef SimpleUtils_h
#define SimpleUtils_h

class SimpleUtils
{
    public:
        /**
         * @brief Structure for complete telemetry output.
         */
        typedef struct TelemetryStruct
        {
            float latitude;             /**< Latitude in decimal degrees */
            float longitude;            /**< Longitude in decimal degrees */
            float altitude;             /**< Altitude in meters from GPS MSL Geoid */
            float altitude_ellipsoid;   /**< Altitude in meters from GPS WGS84 Ellipsoid */
            float altitude_relative;    /**< Altitude in meters from GPS relative to boot GPS MSL geoid altitude */
            float altitude_barometric;  /**< Altitude in meters from barometer */
            float velocity_horizontal;  /**< Velocity horizontally along course vector */
            float velocity_vertical;    /**< Velocity vertically */
            float roll;                 /**< Roll in radians */
            float pitch;                /**< Pitch in radians */
            float yaw;                  /**< Yaw in radians */ 
            float heading;              /**< Magnetic heading in degrees */
            float course;               /**< Direction of travel in degrees */
            float temperature;          /**< Temperature in degrees C */
            float pressure;             /**< Pressure in pascals */
            float humidity;             /**< Humidity in % */
            float hdop;                 /**< GPS HDOP */
            float fix;                  /**< GPS fix status */
        } TelemetryStruct;
        
        /**
         * @brief Structure for axis related data (e.g. acceleration, velocity, gyro, mag, etc...)
         */
        typedef struct AxisData
        {
            float x;
            float y;
            float z;
        } AxisData;

        /**
         * @brief Structure for quaternion related data
         */
        typedef struct QuatData
        {
            float w;
            float x;
            float y;
            float z;
        } QuatData;
};

#endif