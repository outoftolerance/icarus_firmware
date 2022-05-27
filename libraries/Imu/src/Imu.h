#ifndef Imu_h
#define Imu_h

#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>

#include <SimpleUtils.h>

class Imu
{
	public:
		Imu();
		bool begin();
		bool update();

		bool getAccelerometer(SimpleUtils::AxisData& accel);
        bool getGyroscope(SimpleUtils::AxisData& gyro);
        bool getMagnetometer(SimpleUtils::AxisData& mag);

	private:
		Adafruit_FXAS21002C gyroscope_ = Adafruit_FXAS21002C(0x0021002C);
        Adafruit_FXOS8700 accelerometer_magnetometer_ = Adafruit_FXOS8700(0x8700A, 0x8700B);

        sensors_event_t gyroscope_data_;
        sensors_event_t accelerometer_data_;
        sensors_event_t magnetometer_data_;

        //Calibration parameters
        float mag_offsets_[3]            = { 13.45F, -1.37F, 32.93F };      /*< Offsets applied to raw x/y/z mag values */
        float mag_softiron_matrix_[3][3] = { {  1.077,  -0.018,  0.054 },
                                             {  0.018,  0.965,  0.050 },
                                             {  0.054,   0.050,  0.968 } }; /*< Soft iron error compensation matrix */
        float mag_field_strength_        = 47.72F;                          /*< Strength of magnetic field recorded */
        float gyro_zero_offsets_[3]      = { 0.0F, 0.0F, 0.0F };            /*< Offsets applied to compensate for gyro zero-drift error for x/y/z */
};

#endif