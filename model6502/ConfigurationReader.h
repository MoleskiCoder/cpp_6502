#pragma once

#include <string>
#include <cstdint>

#include <boost/property_tree/ptree.hpp>

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

private:
	bool GetBooleanValue(const boost::property_tree::ptree& root, std::string path, bool defaultValue);
	bool GetBooleanValue(const boost::property_tree::ptree& root, std::string path);

	uint8_t GetByteValue(const boost::property_tree::ptree& root, std::string path, uint8_t defaultValue);
	uint8_t GetByteValue(const boost::property_tree::ptree& root, std::string path);

	uint16_t GetUShortValue(const boost::property_tree::ptree& root, std::string path, uint16_t defaultValue);
	uint16_t GetUShortValue(const boost::property_tree::ptree& root, std::string path);

	int GetIntValue(const boost::property_tree::ptree& root, std::string path, int defaultValue);
	int GetIntValue(const boost::property_tree::ptree& root, std::string path);

	double GetDoubleValue(const boost::property_tree::ptree& root, std::string path, double defaultValue);
	double GetDoubleValue(const boost::property_tree::ptree& root, std::string path);

	std::string GetStringValue(const boost::property_tree::ptree& root, std::string path, std::string defaultValue);
	std::string GetStringValue(const boost::property_tree::ptree& root, std::string path);

	boost::property_tree::ptree m_root;
};