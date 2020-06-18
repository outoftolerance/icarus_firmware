#include "SimpleServo.h"

SimpleServo::SimpleServo(int min_pwm, int max_pwm, int servo_channel, Adafruit_PWMServoDriver* driver)
{
    min_pwm_ = min_pwm;
    max_pwm_ = max_pwm;
    servo_channel_ = servo_channel;
    driver_ = driver;

    current_pwm_ = min_pwm;

    step_size_ = DEFAULT_STEP_SIZE;
    update_delay_ = DEFAULT_UPDATE_DELAY;

    update_timer_.setInterval(update_delay_);
}

float SimpleServo::getCurrentAngle()
{
    return current_angle_;
}

float SimpleServo::getTargetAngle()
{
    return target_angle_;
}

bool SimpleServo::setTargetAngle(float angle)
{
    target_angle_ = angle;
    target_pwm_ = map(angle, 0, 90, min_pwm_, max_pwm_);
    return true;
}

bool SimpleServo::setTargetMicroseconds(int microseconds)
{
    target_pwm_ = microseconds;
    target_angle_ = map(target_pwm_, min_pwm_, max_pwm_, 0, 90);
    return true;
}

bool SimpleServo::goToMax()
{
    target_pwm_ = max_pwm_;
    target_angle_ = map(target_pwm_, min_pwm_, max_pwm_, 0, 90);
    driver_->setPWM(servo_channel_, 0, target_pwm_);
    current_pwm_ = target_pwm_;
    current_angle_ = target_angle_;
    return true;
}

bool SimpleServo::goToMin()
{
    target_pwm_ = min_pwm_;
    target_angle_ = map(target_pwm_, min_pwm_, max_pwm_, 0, 90);
    driver_->setPWM(servo_channel_, 0, target_pwm_);
    current_pwm_ = target_pwm_;
    current_angle_ = target_angle_;
    return true;
}

bool SimpleServo::setStepSize(int size)
{
    step_size_ = size;
    return true;
}

bool SimpleServo::setUpdateRate(int delay)
{
    update_delay_ = delay;
    bool result = update_timer_.setInterval(update_delay_);
    return result;
}

int SimpleServo::calculateMovement_()
{
    int output = current_pwm_;

    if(target_pwm_ != current_pwm_)
    {
        if(target_pwm_ - current_pwm_ > step_size_)
        {
            output += step_size_;
        }
        else if(target_pwm_ - current_pwm_ < step_size_)
        {
            output -= step_size_;
        }
        else
        {
            output = target_pwm_;
        }
    }

    if(min_pwm_ < max_pwm_ && output < min_pwm_)
    {
        return min_pwm_;
    }
    else if(min_pwm_ > max_pwm_ && output > min_pwm_)
    {
        return min_pwm_;
    }
    else if(max_pwm_ < min_pwm_ && output < max_pwm_)
    {
        return max_pwm_;
    }
    else if(max_pwm_ > min_pwm_ && output > max_pwm_)
    {
        return max_pwm_;
    }
    else
    {
        return output;
    }
}

bool SimpleServo::start()
{
    bool result = update_timer_.start();
    return result;
}

bool SimpleServo::stop()
{
    bool result = update_timer_.stop();
    return result;
}

bool SimpleServo::move()
{
    if(update_timer_.check())
    {
        current_pwm_ = calculateMovement_();
        current_angle_ = map(current_pwm_, min_pwm_, max_pwm_, 0, 90);

        driver_->setPWM(servo_channel_, 0, current_pwm_);

        update_timer_.reset();
    }

    return true;
}