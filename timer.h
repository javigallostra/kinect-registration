#ifndef timer_h
#define timer_h

#include <time.h>
#include <iostream>

class Timer
{
	private:

		struct timespec startTime, stopTime;

		double elapsed;

		bool started, newline, verbose;

	public:

		void tic ();

		void toc ();

		double getElapsedTime ();

		Timer ();

		Timer (bool nline, bool verb);
};

#endif