#ifndef SimpleAHRS_h
#define SimpleAHRS_h

#include <SimpleUtils.h>
#include <Madgwick.h>

class SimpleAHRS {
	public:
		SimpleAHRS();
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
		Madgwick ahrs_;
		float sample_rate_;

		//Calibration parameters
        float mag_offsets_[3]            = { 0.93F, -7.47F, -35.23F };      /*< Offsets applied to raw x/y/z mag values */
        float mag_softiron_matrix_[3][3] = { {  0.986,  -0.035,  0.032 },
                                             {  -0.035,  0.907,  0.037 },
                                             {  0.032,   0.037,  1.122 } }; /*< Soft iron error compensation matrix */
        float mag_field_strength_        = 44.43F;                          /*< Strength of magnetic field recorded */
        float gyro_zero_offsets_[3]      = { 0.0F, 0.0F, 0.0F };            /*< Offsets applied to compensate for gyro zero-drift error for x/y/z */
};

#endif