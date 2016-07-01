#include "stdafx.h"
#include "ConfigurationReader.h"

ConfigurationReader::ConfigurationReader(std::string path) {
}

bool ConfigurationReader::GetBooleanValue(std::string path, bool defaultValue) {
	return defaultValue;
}

bool ConfigurationReader::GetBooleanValue(std::string path) {
	return GetBooleanValue(path, false);
}

uint8_t ConfigurationReader::GetByteValue(std::string path, uint8_t defaultValue) {
	return defaultValue;
}

uint8_t ConfigurationReader::GetByteValue(std::string path) {
	return GetByteValue(path, 0);
}

uint16_t ConfigurationReader::GetUShortValue(std::string path, uint16_t defaultValue) {
	return defaultValue;
}

uint16_t ConfigurationReader::GetUShortValue(std::string path) {
	return GetUShortValue(path, 0);
}

int ConfigurationReader::GetIntValue(std::string path, int defaultValue) {
	return defaultValue;
}

int ConfigurationReader::GetIntValue(std::string path) {
	return GetIntValue(path, 0);
}

double ConfigurationReader::GetDoubleValue(std::string path, double defaultValue) {
	return defaultValue;
}

double ConfigurationReader::GetDoubleValue(std::string path) {
	return GetDoubleValue(path, 0);
}


std::string ConfigurationReader::GetStringValue(std::string path, std::string defaultValue) {
	return defaultValue;
}

std::string ConfigurationReader::GetStringValue(std::string path) {
	return GetStringValue(path, "");
}