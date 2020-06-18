#include "Imu.h"

Imu::Imu()
{
    
}

bool Imu::begin()
{
    //Start gyroscope
    if(!gyroscope_.begin())
    {
        Serial.println("Failed to initialise Gyroscope!");
        return false;
    }

    //Start accel and mag
    if(!accelerometer_magnetometer_.begin(ACCEL_RANGE_4G))
    {
        Serial.println("Failed to initialise Acceleromter/Magnetometer!");
        return false;
    }

    return true;
}

bool Imu::update()
{
    gyroscope_.getEvent(&gyroscope_data_);
    accelerometer_magnetometer_.getEvent(&accelerometer_data_, &magnetometer_data_);

    return true;
}

bool Imu::getAccelerometer(SimpleUtils::AxisData& accel)
{
    accel.x = accelerometer_data_.acceleration.x;
    accel.y = accelerometer_data_.acceleration.y;
    accel.z = accelerometer_data_.acceleration.z;

    return true;
}

bool Imu::getGyroscope(SimpleUtils::AxisData& gyro)
{
    gyro.x = gyroscope_data_.gyro.x;
    gyro.y = gyroscope_data_.gyro.y;
    gyro.z = gyroscope_data_.gyro.z;

    return true;
}

bool Imu::getMagnetometer(SimpleUtils::AxisData& mag)
{
    mag.x = magnetometer_data_.magnetic.x;
    mag.y = magnetometer_data_.magnetic.y;
    mag.z = magnetometer_data_.magnetic.z;

    return true;
}