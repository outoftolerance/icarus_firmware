#include "Telemetry.h"

/*------------------------------Constructor Methods------------------------------*/
Telemetry::Telemetry(Log* logger):
    logger_(logger)
{
    gps_serial_ = NULL;
    gps_fix_status_ = false;
    altitude_base_is_set_ = false;
    altitude_base_ = 0;
    sensor_update_timer_.setInterval(SENSOR_UPDATE_INTERVAL);
}

Telemetry::Telemetry(Log* logger, Stream* gps_serial) :
    gps_serial_(gps_serial),
    logger_(logger)
{
    gps_serial_buffer_ = new Buffer(GPS_SERIAL_BUFFER_SIZE);
    gps_fix_status_ = false;
    altitude_base_is_set_ = false;
    altitude_base_ = 0;
    sensor_update_timer_.setInterval(SENSOR_UPDATE_INTERVAL);
}

/*------------------------------Public Methods------------------------------*/

bool Telemetry::init()
{
    //Initialise the GPS
    if(gps_serial_ != NULL)
    {
        static_cast<HardwareSerial&>(*gps_serial_).begin(GPS_SERIAL_BAUD);
    }

    //Initialise each sensor
    if(!imu_.begin())
    {
        logger_->event(LOG_LEVELS::DEBUG, "Failed to initialise IMU!");
        return false;
    }

    if(!baro_.begin()) {
        logger_->event(LOG_LEVELS::DEBUG, "Failed to initialise Barometer!");
        return false;
    }

    //Initialise the AHRS filter
    if(!ahrs_.begin((float)(1.0/(SENSOR_UPDATE_INTERVAL))))
    {
        logger_->event(LOG_LEVELS::DEBUG, "Failed to initialise AHRS!");
        return false;
    }

    //Start the sensor update timer
    if(!sensor_update_timer_.start())
    {
        logger_->event(LOG_LEVELS::DEBUG, "Failed to initialise Sensor Update Timer!");
        return false;
    }

    //Everything initialized correctly
    return true;
}

void Telemetry::update()
{
    if(sensor_update_timer_.check())
    {
        imu_.update();

        SimpleUtils::AxisData accel;
        SimpleUtils::AxisData gyro;
        SimpleUtils::AxisData mag;

        imu_.getAccelerometer(accel);
        imu_.getGyroscope(gyro);
        imu_.getMagnetometer(mag);

        ahrs_.update(accel, gyro, mag);

        sensor_update_timer_.reset();
    }

    if(gps_serial_ != NULL)
    {
        char c;

        while(gps_serial_->available())
        {
            c = gps_serial_->read();
            gps_.encode(c);
            gps_serial_buffer_->push(c);
        }

        //Detect for first transition of GPS fix
        if(!gps_fix_status_ && gps_.fix.value() > 0 && !altitude_base_is_set_)
        {
            altitude_base_ = (float)gps_.altitude.meters();
            altitude_base_is_set_ = true;

            gps_fix_status_ = true;
        }
    }
    else if(!altitude_base_is_set_)
    {
        altitude_base_ = baro_.readAltitude(SEALEVELPRESSURE_HPA);
        altitude_base_is_set_ = true;
    }
}

bool Telemetry::get(SimpleUtils::TelemetryStruct& telemetry)
{
    //Grab data not dependent on GPS
    telemetry.roll = ahrs_.getRoll();
    telemetry.pitch = ahrs_.getPitch();
    telemetry.yaw = ahrs_.getYaw();
    telemetry.heading = ahrs_.getYawDegrees();
    telemetry.temperature = baro_.readTemperature();
    telemetry.pressure = baro_.readPressure();
    telemetry.humidity = baro_.readHumidity();
    telemetry.altitude_barometric = baro_.readAltitude(SEALEVELPRESSURE_HPA);

    //Assign to output struct
    if(gps_serial_ != NULL)
    {
        telemetry.latitude = (float)gps_.location.lat();
        telemetry.longitude = (float)gps_.location.lng();
        telemetry.altitude = (float)gps_.altitude.meters();
        telemetry.altitude_ellipsoid = (float)gps_.altitude_ellipsoid.meters();
        telemetry.course = (float)gps_.course.deg();
        telemetry.velocity_vertical = ((float)gps_.altitude.meters() - altitude_gps_previous_) / (float)(millis() - millis_altitude_gps_previous_)/1000.0;
        telemetry.velocity_horizontal = (float)gps_.speed.mps();
        telemetry.hdop = (float)gps_.hdop.hdop();
        telemetry.fix = (float)gps_.fix.value();

        altitude_gps_previous_ = telemetry.altitude;
        millis_altitude_gps_previous_ = millis();

        if(altitude_base_is_set_)
        {
            telemetry.altitude_relative = telemetry.altitude - altitude_base_;
        }
        else
        {
            telemetry.altitude_relative = 0.0;
        }
    }
    else
    {
        telemetry.latitude = 0.0;
        telemetry.longitude = 0.0;
        telemetry.altitude = 0.0;
        telemetry.altitude_ellipsoid = 0.0;
        telemetry.course = 0.0;
        telemetry.velocity_vertical = 0.0;
        telemetry.velocity_horizontal = 0.0;
        telemetry.hdop = 0.0;
        telemetry.fix = 0.0;

        if(altitude_base_is_set_)
        {
            telemetry.altitude_relative = telemetry.altitude_barometric - altitude_base_;
        }
        else
        {
            telemetry.altitude_relative = 0;
        }
    }

    return true;
}

bool Telemetry::getAccelerometerRaw(SimpleUtils::AxisData& accelerometer)
{
    return false;
}

bool Telemetry::getGyroscopeRaw(SimpleUtils::AxisData& gyroscope)
{
    return false;
}

bool Telemetry::getMagnetometerRaw(SimpleUtils::AxisData& magnetometer)
{
    return false;
}

bool Telemetry::getBarometerRaw(float& data)
{
    return false;
}

int Telemetry::getGpsString(char string[], const unsigned int string_length)
{
    int i = 0;
    
    if(gps_serial_ != NULL)
    {
        while(gps_serial_buffer_->available() && i < string_length)
        {
            string[i] = gps_serial_buffer_->pop();
            i++;
        }
    }

    return i;
}

bool Telemetry::getGpsFixStatus()
{
    return gps_fix_status_;
}

float Telemetry::getGpsHdop()
{
    if(gps_serial_ != NULL)
    {
        return (float)gps_.hdop.hdop();
    }
    else
    {
        return 0;
    }
}

uint32_t Telemetry::getGpsUnixTime()
{
    DateTime current_time(
        gps_.date.year(),
        gps_.date.month(),
        gps_.date.day(),
        gps_.time.hour(),
        gps_.time.minute(),
        gps_.time.second()
        );

    return current_time.unixtime();
}

bool Telemetry::resetBaseAltitude()
{
    if(gps_serial_ != NULL)
    {
        altitude_base_ = gps_.altitude.meters();
    }
    else
    {
        altitude_base_ = baro_.readAltitude(SEALEVELPRESSURE_HPA);   
    }

    return true;
}