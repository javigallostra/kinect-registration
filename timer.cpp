#include "timer.h"

void Timer::tic ()
{
	started = true;
	clock_gettime(CLOCK_MONOTONIC, &startTime);
}

void Timer::toc()
{
	if (not started)
	{
		std::cout << "Clock was not started" << std::endl;
	}
	else
	{
		clock_gettime(CLOCK_MONOTONIC, &stopTime);
		elapsed = (stopTime.tv_sec - startTime.tv_sec);
		elapsed += (stopTime.tv_nsec - startTime.tv_nsec) / 1000000000.0;
		if (verbose)
		{
			if (newline) {std::cout << elapsed << std::endl;}
			else {std::cout << elapsed << " ";}
		}
		started = false;
	}
}

double Timer::getElapsedTime ()
{
	return elapsed;
}

Timer::Timer (bool nline, bool verb)
{
	started = false;
	newline = nline;
	verbose = verb;
}

Timer::Timer()
{
	started = false;
	newline = false;
	verbose = true;
}