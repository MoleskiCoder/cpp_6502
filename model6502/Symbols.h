#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <string>

class Symbols
{
public:
	std::map<uint16_t, std::string> labels;
	std::map<uint16_t, std::string> constants;
	std::map<std::string, uint16_t> scopes;
	std::map<std::string, uint64_t> addresses;

	Symbols(std::string path);

private:
	void AssignScopes();
	void AssignSymbols();

	void Parse(std::string path);

	struct kv_pair_t {
		std::map<std::string, std::string> element;
	};

	std::vector<std::string> split(const std::string& input, const std::vector<std::string>& delimiters);

	std::map<std::string, std::map<std::string, kv_pair_t>> parsed;
};
