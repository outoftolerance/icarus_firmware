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
	//Filter needs gyro in deg/s not rad/s
	float gx = gyro.x * 57.2958F;
	float gy = gyro.y * 57.2958F;
	float gz = gyro.z * 57.2958F;

	ahrs_.update(gx, gy, gz, accel.x, accel.y, accel.z, mag.x, mag.y, mag.z);
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