#ifndef SimpleAHRS_h
#define SimpleAHRS_h

#include <SimpleUtils.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>

#include <SimpleLog.h>

class SimpleAHRS {
	public:
		SimpleAHRS();
		/**
		 * @brief      Begins the AHRS filter
		 *
		 * @param[in]  sample_rate  The rate in Hz that the sensors are sampled at
		 *
		 * @return     True/False if init was successful
		 */
		bool begin(float sample_rate);

		bool update(const SimpleUtils::AxisData& accel, const SimpleUtils::AxisData& gyro, const SimpleUtils::AxisData& mag);
		float getRoll();
		float getPitch();
		float getYaw();
		float getRollDegrees();
		float getPitchDegrees();
		float getYawDegrees();
		bool getQuaternion(SimpleUtils::QuatData& quat);
		bool getAccelerationNED(SimpleUtils::AxisData& accel);

	private:
		Adafruit_NXPSensorFusion ahrs_;
		//Adafruit_Madgwick ahrs_;
		//Adafruit_Mahony ahrs_;
		float sample_rate_;
};

#endif