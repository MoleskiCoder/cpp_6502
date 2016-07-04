#pragma once

#include <cstdint>
#include <string>
#include <chrono>
#include <fstream>
#include <memory>
#include <functional>

#include <system6502.h>
#include <Signal.h>

#include "Profiler.h"
#include "Symbols.h"
#include "Configuration.h"
#include "Disassembly.h"
#include "DisassemblyEventArgs.h"

class Controller
{
public:
	const uint16_t BbcOSLoadAddress = 0xc000;
	const uint16_t BbcOSLanguageAddress = 0x8000;

	bool disassemble;
	std::string disassemblyLogPath;

	double hostSpeed;

	ProcessorType processorLevel;
	double speed;
	int pollIntervalMilliseconds;

	bool stopAddressEnabled;
	bool stopWhenLoopDetected;
	bool stopBreak;

	bool profileAddresses;
	bool countInstructions;

	std::string bbcLanguageRomPath;
	std::string bbcOSRomPath;

	bool bbcVduEmulation;

	std::string romPath;
	uint16_t romLoadAddress;

	std::string ramPath;
	uint16_t ramLoadAddress;

	bool resetStart;
	uint16_t startAddress;

	uint16_t stopAddress;

	uint8_t breakInstruction;

	uint16_t inputAddress;
	uint16_t outputAddress;

	std::string debugFile;

	std::shared_ptr<System6502> processor;

	uint16_t oldPC = 0;

	std::chrono::system_clock::time_point startTime;
	std::chrono::system_clock::time_point finishTime;

	std::shared_ptr<Disassembly> disassembler;
	std::shared_ptr<std::ofstream> disassemblyLog;

	std::shared_ptr<Profiler> profiler;

#if _DEBUG
	std::string diagnosticsBuffer;
#endif

	std::shared_ptr<Symbols> symbols;

public:
	Controller(Configuration configuration);

	Signal<DisassemblyEventArgs> Disassembled;

	void Configure();
	void Start();

private:
	void Processor_Starting();
	void Processor_Finished();

	void Processor_ExecutingInstruction(const AddressEventArgs& addressEvent);

	void Controller_Disassembled(const DisassemblyEventArgs& e);

	void Processor_WritingByte(const AddressEventArgs& addressEvent);
	void Processor_ReadingByte(const AddressEventArgs& addressEvent);

	void Processor_InvalidWriteAttempt(const AddressEventArgs& addressEvent);

	void Processor_Polling();

	void BufferDiagnosticsOutput(std::string output);

	void HandleByteWritten(uint8_t cell);
	void HandleByteRead(uint8_t cell);

	void Profiler_EmitScope(const ProfileScopeEventArgs& e);
	void Profiler_EmitLine(const ProfileLineEventArgs& e);

	void Profiler_FinishedScopeOutput();
	void Profiler_StartingScopeOutput();

	void Profiler_StartingLineOutput();
	void Profiler_FinishedLineOutput();

	void Profiler_StartingOutput();
	void Profiler_FinishedOutput();

	void PrintPercentage(std::ostream& out, double value) {
		out << std::fixed << std::setw(3) << std::setprecision(1) << std::setfill('0') << value << "%";
	}
};