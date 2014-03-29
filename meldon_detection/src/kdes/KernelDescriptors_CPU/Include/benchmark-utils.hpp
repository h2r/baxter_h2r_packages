#ifndef _BENCHMARK_UTILS_HPP_
#define _BENCHMARK_UTILS_HPP_

#include <iostream>

static void printOps(double num_ops, double exec_time)
{
  std::cout << "GFLOPS: " << num_ops / (1000000 * exec_time * 1000) << std::endl;
}




#ifdef _WIN32

#define WINDOWS_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max

class Timer
{
public:

	Timer()
	{
		QueryPerformanceFrequency(&freq);
	}

	void start()
	{
		QueryPerformanceCounter((LARGE_INTEGER*) &start_time);
	}

	double get() const
	{
		LARGE_INTEGER  end_time;
		QueryPerformanceCounter((LARGE_INTEGER*) &end_time);
		return (static_cast<double>(end_time.QuadPart) - static_cast<double>(start_time.QuadPart)) / static_cast<double>(freq.QuadPart);
	}


private:
	LARGE_INTEGER freq;
    LARGE_INTEGER start_time;
};

#else

#include <sys/time.h>

class Timer
{
public:

	Timer() : ts(0)
	{}

	void start()
	{
		struct timeval tval;
		gettimeofday(&tval, NULL);
		ts = tval.tv_sec * 1000000 + tval.tv_usec;
	}

	double get() const
	{
		struct timeval tval;
		gettimeofday(&tval, NULL);
		int64_t end_time = tval.tv_sec * 1000000 + tval.tv_usec;

		return static_cast<double>(end_time-ts) / 1000000.0;
	}

private:
	int64_t ts;
};


#endif

#endif
