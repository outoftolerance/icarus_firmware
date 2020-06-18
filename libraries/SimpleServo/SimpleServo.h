#ifndef SIMPLESERVO_H
#define SIMPLESERVO_H

#include <Timer.h>
#include <Adafruit_PWMServoDriver.h>

#define DEFAULT_STEP_SIZE 1
#define DEFAULT_UPDATE_DELAY 25

#define SERVO_PWM_FREQUENCY 60

class SimpleServo {
    public:
        /**
         * @brief 
         * @param 
         * @param 
         * @param 
         * @param 
         */
        SimpleServo(int min_pwm, int max_pwm, int servo_channel_, Adafruit_PWMServoDriver* driver);

        /**
         * @brief Start the servo control
         */
        bool start();

        /**
         * @brief Stop the servo control
         */
        bool stop();

        /**
         * @brief Do a movement update
         */
        bool move();

        /**
         * @brief 
         */
        float getCurrentAngle();

        /**
         * @brief 
         */
        float getTargetAngle();

        /**
         * @brief 
         * @param 
         */
        bool setTargetAngle(float angle);

        /**
         * @brief 
         * @param 
         */
        bool setTargetMicroseconds(int microseconds);

        /**
         * @brief 
         */
        bool goToMax();

        /**
         * @brief 
         */
        bool goToMin();

        /**
         * @brief 
         * @param 
         */
        bool setStepSize(int size);

        /**
         * @brief 
         * @param 
         */
        bool setUpdateRate(int delay);

    private:
        /**
         * @brief Calculates movement from current position to next position
         */
        int calculateMovement_();

        int min_pwm_;                       /**< Minimum PWM for this servo */
        int max_pwm_;                       /**< Maximum PWM for this servo */
        int servo_channel_;                 /**< Servo channel ID */

        Timer update_timer_;                /**< Timer for setting update rate */
        Adafruit_PWMServoDriver* driver_;   /**< Servo driver object */

        float target_angle_;                /**< Current servo target angle */
        float current_angle_;               /**< Current servo angle */

        int target_pwm_;                    /**< Current target PWM value */
        int current_pwm_;                   /**< Current servo PWM value */
        int step_size_;                     /**< Step size per update period */
        int update_delay_;                  /**< Delay between updates (ms) */
};

#endif