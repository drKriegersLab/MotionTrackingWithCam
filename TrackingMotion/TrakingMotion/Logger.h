#pragma once

#include <iostream>
#include <fstream>

class Logger
{
public:
	Logger(std::string outPath);	
	~Logger();

	void addNewEntry(std::string process, std::string entry);	

private:
	std::ofstream logFile;
};

