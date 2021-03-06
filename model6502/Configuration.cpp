#include "stdafx.h"
#include "Configuration.h"

Configuration::Configuration(std::string path)
: ConfigurationReader(path) {

	hostSpeed = GetDoubleValue("Host.speed", 2900.0);

	processorLevel = GetProcessorTypeValue("CPU.level");
	speed = GetDoubleValue("CPU.speed", 2.0);
	pollIntervalMilliseconds = GetIntValue("CPU.pollIntervalMilliseconds", 10);

	inputAddress = GetUShortValue("IO.inputAddress");
	outputAddress = GetUShortValue("IO.outputAddress");

	romPath = GetStringValue("ROM.path");
	romLoadAddress = GetUShortValue("ROM.loadAddress");

	ramPath = GetStringValue("RAM.path");
	ramLoadAddress = GetUShortValue("RAM.loadAddress");

	bbcLanguageRomPath = GetStringValue("BBC.language.path");
	bbcOSRomPath = GetStringValue("BBC.OS.path");
	bbcVduEmulation = GetBooleanValue("BBC.VDUEmulation");

	startAddress = GetUShortValue("run.startAddress");
	resetStart = GetBooleanValue("run.resetStart");
	stopBreak = GetBooleanValue("run.stopBreak");
	breakInstruction = GetByteValue("run.breakInstruction", 0x00);
	stopWhenLoopDetected = GetBooleanValue("run.stopWhenLoopDetected");
	stopAddress = GetUShortValue("run.stopAddress");
	stopAddressEnabled = stopAddress != 0;

#ifdef _DEBUG
	disassemble = GetBooleanValue("debug.disassemble");
	disassemblyLogPath = GetStringValue("debug.disassemblyLogPath");
	debugFile = GetStringValue("debug.debugFile");
	countInstructions = GetBooleanValue("debug.countInstructions");
	profileAddresses = GetBooleanValue("debug.profileAddresses");
#else
	disassemble = GetBooleanValue("release.disassemble");
	disassemblyLogPath = GetStringValue("release.disassemblyLogPath");
	debugFile = GetStringValue("release.debugFile");
	countInstructions = GetBooleanValue("release.countInstructions");
	profileAddresses = GetBooleanValue("release.profileAddresses");
#endif
}

ProcessorType Configuration::GetProcessorTypeValue(std::string path, ProcessorType defaultValue) {
	auto value = GetStringValue(path);
	if (value.empty())
		return defaultValue;
	
	if (value == "Cpu65SC02")
		return ProcessorType::Cpu65SC02;

	if (value == "Cpu65C02")
		return ProcessorType::Cpu65C02;

	return ProcessorType::Cpu6502;
}

ProcessorType Configuration::GetProcessorTypeValue(std::string path) {
	return GetProcessorTypeValue(path, ProcessorType::Cpu6502);
}
