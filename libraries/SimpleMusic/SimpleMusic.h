#ifndef SimpleMusic_h
#define SimpleMusic_h

#include <Timer.h>

#define FREQUENCY 4000

class SimpleMusic {
	public:
		SimpleMusic(int pin);
		bool play();
		bool stop();
	private:
		Timer rate_timer_;
		int track_location_;
		int pin_;
		bool sound_on_;

		int sos_track_[18] = {50,50,50,50,50,50,150,50,150,50,150,50,50,50,50,50,50,1000};
};

#endif