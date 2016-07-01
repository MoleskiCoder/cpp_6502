#pragma once

#include <cstdint>
#include <string>

#include <ProcessorType.h>

#include "ConfigurationReader.h"

class Configuration : public ConfigurationReader
{
public:
	double hostSpeed;

	ProcessorType processorLevel;
	double speed;
	int pollIntervalMilliseconds;

	uint16_t inputAddress;
	uint16_t outputAddress;

	std::string romPath;
	uint16_t romLoadAddress;

	std::string ramPath;
	uint16_t ramLoadAddress;

	std::string bbcLanguageRomPath;
	std::string bbcOSRomPath;
	bool bbcVduEmulation;

	std::string disassemblyLogPath;
	std::string debugFile;

	uint16_t startAddress;

	bool resetStart;

	bool stopWhenLoopDetected;
	bool stopBreak;
	uint8_t breakInstruction;
	uint16_t stopAddress;
	bool stopAddressEnabled;

	bool disassemble;
	bool countInstructions;
	bool profileAddresses;

public:
	Configuration(std::string path);

private:
	ProcessorType GetProcessorTypeValue(std::string path, ProcessorType defaultValue);
	ProcessorType GetProcessorTypeValue(std::string path);
};

