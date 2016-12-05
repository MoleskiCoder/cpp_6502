#include "stdafx.h"
#include "ConfigurationReader.h"

#include <boost/property_tree/json_parser.hpp>

ConfigurationReader::ConfigurationReader(std::string path) {
	boost::property_tree::read_json(path, m_root);
}

bool ConfigurationReader::GetBooleanValue(std::string path, bool defaultValue) {
	return GetBooleanValue(m_root, path, defaultValue);
}

bool ConfigurationReader::GetBooleanValue(std::string path) {
	return GetBooleanValue(path, false);
}

uint8_t ConfigurationReader::GetByteValue(std::string path, uint8_t defaultValue) {
	return GetByteValue(m_root, path, defaultValue);
}

uint8_t ConfigurationReader::GetByteValue(std::string path) {
	return GetByteValue(path, 0);
}

uint16_t ConfigurationReader::GetUShortValue(std::string path, uint16_t defaultValue) {
	return GetUShortValue(m_root, path, defaultValue);
}

uint16_t ConfigurationReader::GetUShortValue(std::string path) {
	return GetUShortValue(path, 0);
}

int ConfigurationReader::GetIntValue(std::string path, int defaultValue) {
	return GetIntValue(m_root, path, defaultValue);
}

int ConfigurationReader::GetIntValue(std::string path) {
	return GetIntValue(path, 0);
}

double ConfigurationReader::GetDoubleValue(std::string path, double defaultValue) {
	return GetDoubleValue(m_root, path, defaultValue);
}

double ConfigurationReader::GetDoubleValue(std::string path) {
	return GetDoubleValue(path, 0);
}

std::string ConfigurationReader::GetStringValue(std::string path, std::string defaultValue) {
	return GetStringValue(m_root, path, defaultValue);
}

std::string ConfigurationReader::GetStringValue(std::string path) {
	return GetStringValue(path, "");
}

//

bool ConfigurationReader::GetBooleanValue(const boost::property_tree::ptree& root, std::string path, bool defaultValue) {
	return root.get(path, defaultValue);
}

bool ConfigurationReader::GetBooleanValue(const boost::property_tree::ptree& root, std::string path) {
	return GetBooleanValue(root, path, false);
}

uint8_t ConfigurationReader::GetByteValue(const boost::property_tree::ptree& root, std::string path, uint8_t defaultValue) {
	return root.get(path, defaultValue);
}

uint8_t ConfigurationReader::GetByteValue(const boost::property_tree::ptree& root, std::string path) {
	return GetByteValue(root, path, 0);
}

uint16_t ConfigurationReader::GetUShortValue(const boost::property_tree::ptree& root, std::string path, uint16_t defaultValue) {
	std::stringstream conversion_stream;
	auto read = GetStringValue(root, path);
	if (read == "")
		return defaultValue;
	conversion_stream << std::hex << read;
	if (conversion_stream.fail())
		return defaultValue;
	uint16_t returnValue;
	conversion_stream >> returnValue;
	if (conversion_stream.fail())
		return defaultValue;
	return returnValue;
}

uint16_t ConfigurationReader::GetUShortValue(const boost::property_tree::ptree& root, std::string path) {
	return GetUShortValue(root, path, 0);
}

int ConfigurationReader::GetIntValue(const boost::property_tree::ptree& root, std::string path, int defaultValue) {
	return root.get(path, defaultValue);
}

int ConfigurationReader::GetIntValue(const boost::property_tree::ptree& root, std::string path) {
	return GetIntValue(root, path, 0);
}

double ConfigurationReader::GetDoubleValue(const boost::property_tree::ptree& root, std::string path, double defaultValue) {
	return root.get(path, defaultValue);
}

double ConfigurationReader::GetDoubleValue(const boost::property_tree::ptree& root, std::string path) {
	return GetDoubleValue(root, path, 0.0);
}

std::string ConfigurationReader::GetStringValue(const boost::property_tree::ptree& root, std::string path, std::string defaultValue) {
	return root.get(path, defaultValue);
}

std::string ConfigurationReader::GetStringValue(const boost::property_tree::ptree& root, std::string path) {
	return GetStringValue(root, path, "");
}
