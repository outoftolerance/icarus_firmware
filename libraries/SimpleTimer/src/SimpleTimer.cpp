#include "SimpleTimer.h"

/*------------------------------Constructor Methods------------------------------*/

SimpleTimer::SimpleTimer()
{
	interval_ = DEFAULT_INTERVAL;
	is_set_ = false;
}

SimpleTimer::SimpleTimer(const unsigned long interval)
{
	interval_ = interval;
	is_set_ = true;
}

/*------------------------------ Public Methods------------------------------*/

bool SimpleTimer::setInterval(const unsigned long interval)
{
	interval_ = interval;
	is_set_ = true;
	return true;
}

unsigned long SimpleTimer::getInterval()
{
	return interval_;
}

bool SimpleTimer::isSet()
{
	return is_set_;
}

bool SimpleTimer::isStarted()
{
	return is_running_;
}

bool SimpleTimer::start()
{
	if(!is_set_ || is_running_)
	{
		return false;
	}

	last_reset_ = millis();
	is_running_ = true;

	return true;
}

bool SimpleTimer::stop()
{
	if(!is_running_ || !is_set_)
	{
		return false;
	}

	is_running_ = false;

	return true;
}

bool SimpleTimer::check()
{
	if(is_running_ && millis() - last_reset_ > interval_)
	{
		return true;
	}

	return false;
}

bool SimpleTimer::reset()
{
	if(!check())
	{
		return false;
	}

	last_reset_ = millis();
	return true;
}

bool SimpleTimer::forceReset()
{
	last_reset_ = millis();
	return true;
}