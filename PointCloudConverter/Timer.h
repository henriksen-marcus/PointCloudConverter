#pragma once
#include <chrono>

/**
 * \brief Simple class for timing code execution.
 */
class Timer
{
public:
	Timer() : start(std::chrono::high_resolution_clock::now()) {}
	~Timer() = default;

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

	std::string Get()
	{
		return std::to_string(Stop()) + " seconds";
	}

	void Println()
	{
		std::cout << "Time taken: " << Stop() << " seconds\n";
	}

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

