#pragma once

#include <string>
#include <cstdint>

class ConfigurationReader {

public:
	ConfigurationReader(std::string path);

	bool GetBooleanValue(std::string path, bool defaultValue);
	bool GetBooleanValue(std::string path);

	uint8_t GetByteValue(std::string path, uint8_t defaultValue);
	uint8_t GetByteValue(std::string path);

	uint16_t GetUShortValue(std::string path, uint16_t defaultValue);
	uint16_t GetUShortValue(std::string path);

	int GetIntValue(std::string path, int defaultValue);
	int GetIntValue(std::string path);

	double GetDoubleValue(std::string path, double defaultValue);
	double GetDoubleValue(std::string path);

	std::string GetStringValue(std::string path, std::string defaultValue);
	std::string GetStringValue(std::string path);
};