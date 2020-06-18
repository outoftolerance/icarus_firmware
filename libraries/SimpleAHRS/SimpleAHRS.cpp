#include <SimpleAHRS.h>

SimpleAHRS::SimpleAHRS()
{

}

bool SimpleAHRS::begin(float sample_rate)
{
	sample_rate_ = sample_rate;
	ahrs_.begin(sample_rate_);

	return true;
}

bool SimpleAHRS::update(const SimpleUtils::AxisData& accel, const SimpleUtils::AxisData& gyro, const SimpleUtils::AxisData& mag)
{
	// Apply mag offset compensation (base values in uTesla)
    float x = mag.x - mag_offsets_[0];
    float y = mag.y - mag_offsets_[1];
    float z = mag.z - mag_offsets_[2];

    // Apply mag soft iron error compensation
    float temp_mx = x * mag_softiron_matrix_[0][0] + y * mag_softiron_matrix_[0][1] + z * mag_softiron_matrix_[0][2];
    float temp_my = x * mag_softiron_matrix_[1][0] + y * mag_softiron_matrix_[1][1] + z * mag_softiron_matrix_[1][2];
    float temp_mz = x * mag_softiron_matrix_[2][0] + y * mag_softiron_matrix_[2][1] + z * mag_softiron_matrix_[2][2];

    // Apply gyro zero-rate error compensation
    float temp_gx = gyro.x + gyro_zero_offsets_[0];
    float temp_gy = gyro.y + gyro_zero_offsets_[1];
    float temp_gz = gyro.z + gyro_zero_offsets_[2];

	ahrs_.update(temp_gx, temp_gy, temp_gz, accel.x, accel.y, accel.z, temp_mx, temp_my, temp_mz);
}

float SimpleAHRS::getRoll()
{
	return ahrs_.getRollRadians();
}

float SimpleAHRS::getPitch()
{
	return ahrs_.getPitchRadians();
}

float SimpleAHRS::getYaw()
{
	return ahrs_.getYawRadians();
}

float SimpleAHRS::getRollDegrees()
{
	return ahrs_.getRoll();
}

float SimpleAHRS::getPitchDegrees()
{
	return ahrs_.getPitch();
}

float SimpleAHRS::getYawDegrees()
{
	return ahrs_.getYaw();
}

bool SimpleAHRS::getQuaternion(SimpleUtils::QuatData& quat)
{
	float w, x, y, z;
	
	ahrs_.getQuaternion(&w, &x, &y, &z);

	quat.w = w;
	quat.z = x;
	quat.y = y;
	quat.z = z;

	return true;
}

bool SimpleAHRS::getAccelerationNED(SimpleUtils::AxisData& accel)
{
	return false;
}