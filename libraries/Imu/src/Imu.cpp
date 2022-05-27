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
    if(!accelerometer_magnetometer_.begin())
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
    // Apply gyro zero-rate error compensation
    float gx = gyroscope_data_.gyro.x + gyro_zero_offsets_[0];
    float gy = gyroscope_data_.gyro.y + gyro_zero_offsets_[1];
    float gz = gyroscope_data_.gyro.z + gyro_zero_offsets_[2];

    gyro.x = gx;
    gyro.y = gy;
    gyro.z = gz;

    return true;
}

bool Imu::getMagnetometer(SimpleUtils::AxisData& mag)
{
    // Apply mag offset compensation (base values in uTesla)
    float x = magnetometer_data_.magnetic.x - mag_offsets_[0];
    float y = magnetometer_data_.magnetic.y - mag_offsets_[1];
    float z = magnetometer_data_.magnetic.z - mag_offsets_[2];

    // Apply mag soft iron error compensation
    float mx = x * mag_softiron_matrix_[0][0] + y * mag_softiron_matrix_[0][1] + z * mag_softiron_matrix_[0][2];
    float my = x * mag_softiron_matrix_[1][0] + y * mag_softiron_matrix_[1][1] + z * mag_softiron_matrix_[1][2];
    float mz = x * mag_softiron_matrix_[2][0] + y * mag_softiron_matrix_[2][1] + z * mag_softiron_matrix_[2][2];

    mag.x = mx;
    mag.y = my;
    mag.z = mz;

    return true;
}