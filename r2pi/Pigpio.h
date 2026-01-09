#pragma once

// RAII wrapper for pigpio
class Pigpio
{
private:
	int m_Id = -1; // Identifier aquired from pigpio library.

public:
	enum class PullUpDown
	{
		OFF,
		DOWN,
		UP
	};

	enum class Mode
	{
		INPUT,
		OUTPUT
		// Alt functions are not supported
	};

public:
	Pigpio();
	~Pigpio();

	// Initialization
	bool Start();
	void Stop();

	// GPIO operations
	bool SetPullUpDown(unsigned int gpio, PullUpDown pullUpDown);
	bool SetMode(unsigned int gpio, Mode mode);
	bool Write(unsigned int gpio, bool value);


	Pigpio(const Pigpio& other) = delete;
	Pigpio(Pigpio&& other) = delete;
	Pigpio& operator=(const Pigpio& other) = delete;
};