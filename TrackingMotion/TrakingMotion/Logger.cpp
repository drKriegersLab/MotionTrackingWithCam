#include "pch.h"
#include "Logger.h"
#include <ctime>
#include <string>

using namespace std;


Logger::Logger(string outPath)
{
	logFile.open(outPath);
	addNewEntry("Logger", "logger initialized");
}


Logger::~Logger()
{
	logFile.close();
}

void Logger::addNewEntry(string process, string entry) {
	time_t now = time(0);
	cout << "[" << ctime(&now) << "] <" << process << ">: " << entry << endl;
}
