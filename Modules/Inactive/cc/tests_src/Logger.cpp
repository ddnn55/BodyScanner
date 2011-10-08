#include "Logger.h"
#include <string>

using namespace std;

Logger::Logger()
{
	this->info();
}

void Logger::info()
{

	status = 0;

}

void Logger::warning()
{

	status = 1;

}

void Logger::debug()
{

	status = 2;

}

void Logger::log(string msg, int stat)
{

	if (getStatus()>=stat)
	cout << msg << endl;

}

int Logger::getStatus()
{
	
	return status;
}
