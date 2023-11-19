#pragma once
#include <chrono>

class Timer
{
public:
	Timer() : start(std::chrono::high_resolution_clock::now()) {}
	~Timer();

	void Start()
	{
		start = std::chrono::high_resolution_clock::now();
	}

	double Stop()
	{
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> diff = end - start;
		return diff.count();
	}

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

