#ifndef DEF_LOGGER
#define DEF_LOGGER


#include <iostream>
#include <string>


class Logger
{

	public:
		Logger();
		void info();
		void warning();
		void debug();
		void log(std::string msg, int stat);
		int getStatus();

	private:
	int status; // 0: Info 1:Warning 2:Debug


};

#endif
