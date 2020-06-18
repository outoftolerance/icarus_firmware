#include <SimpleMusic.h>

SimpleMusic::SimpleMusic(int pin):
pin_(pin)
{
	sound_on_ = false;
}

bool SimpleMusic::play()
{
	//Check if playing already, if not set things up to start
	if(!rate_timer_.isStarted())
	{
		rate_timer_.forceReset();
		rate_timer_.setInterval(sos_track_[track_location_]);
		rate_timer_.start();

		tone(pin_, FREQUENCY);
		sound_on_ = true;

		track_location_++;
	}
	
	//Check if previous interval is over
	if(rate_timer_.check())
	{
		//Toggle pin
		if(sound_on_)
		{
			noTone(pin_);		//If there is no tone() playing, noTone() crashes the program! Need the if statement to stop this.
			sound_on_ = false;
		}
		else
		{
			tone(pin_, FREQUENCY);
			sound_on_ = true;
		}

		//Set timer for next interval
		rate_timer_.stop();
		rate_timer_.setInterval(sos_track_[track_location_]);
		rate_timer_.forceReset();
		rate_timer_.start();

		//Move forward in track
		track_location_++;

		//If we reach the end, loop to beginning again
		if(track_location_ >= sizeof(sos_track_)/sizeof(int))
		{
			track_location_ = 0;
		}
	}

	return true;
}

bool SimpleMusic::stop()
{
	//Turn off
	if(sound_on_)
	{
		noTone(pin_);		//If there is no tone() playing, noTone() crashes the program! Need the if statement to stop this.
		sound_on_ = false;
	}

	//Stop timer
	rate_timer_.stop();

	//Reset track location to start
	track_location_ = 0;

	return true;
}