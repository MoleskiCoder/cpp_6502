#include "stdafx.h"
#include "Controller.h"

#include <fstream>

#include <conio.h>
#include <sstream>
#include <iomanip>

#ifdef _DEBUG
#	include <Windows.h>
#endif

using namespace std::placeholders;

Controller::Controller(Configuration configuration) {

	disassemble = configuration.disassemble;
	disassemblyLogPath = configuration.disassemblyLogPath;

	hostSpeed = configuration.hostSpeed;

	processorLevel = configuration.processorLevel;
	speed = configuration.speed;
	pollIntervalMilliseconds = configuration.pollIntervalMilliseconds;

	stopAddressEnabled = configuration.stopAddressEnabled;
	stopWhenLoopDetected = configuration.stopWhenLoopDetected;
	stopBreak = configuration.stopBreak;

	profileAddresses = configuration.profileAddresses;
	countInstructions = configuration.countInstructions;

	bbcLanguageRomPath = configuration.bbcLanguageRomPath;
	bbcOSRomPath = configuration.bbcOSRomPath;

	bbcVduEmulation = configuration.bbcVduEmulation;

	romPath = configuration.romPath;
	romLoadAddress = configuration.romLoadAddress;

	ramPath = configuration.ramPath;
	ramLoadAddress = configuration.ramLoadAddress;

	resetStart = configuration.resetStart;
	startAddress = configuration.startAddress;

	stopAddress = configuration.stopAddress;

	breakInstruction = configuration.breakInstruction;

	inputAddress = configuration.inputAddress;
	outputAddress = configuration.outputAddress;

	debugFile = configuration.debugFile;
}

void Controller::Configure() {

	processor.reset(new System6502(processorLevel, speed, pollIntervalMilliseconds));

	if (disassemble || stopAddressEnabled || stopWhenLoopDetected || profileAddresses)
		processor->ExecutingInstruction.push_back(std::bind(&Controller::Processor_ExecutingInstruction, this, _1));
	if (stopBreak)
		processor->ExecutedInstruction.push_back(std::bind(&Controller::Processor_ExecutedInstruction, this, _1));

	processor->WritingByte.push_back(std::bind(&Controller::Processor_WritingByte, this, _1));
	processor->ReadingByte.push_back(std::bind(&Controller::Processor_ReadingByte, this, _1));
	processor->InvalidWriteAttempt.push_back(std::bind(&Controller::Processor_InvalidWriteAttempt, this, _1));
	processor->Starting.push_back(std::bind(&Controller::Processor_Starting, this));
	processor->Finished.push_back(std::bind(&Controller::Processor_Finished, this));
	processor->Polling.push_back(std::bind(&Controller::Processor_Polling, this));

	processor->Initialise();

	auto bbc = !bbcLanguageRomPath.empty() && !bbcOSRomPath.empty();
	if (bbc) {
		processor->LoadRom(bbcOSRomPath, BbcOSLoadAddress);
		processor->LoadRom(bbcLanguageRomPath, BbcOSLanguageAddress);
	}

	auto rom = !romPath.empty();
	if (rom)
		processor->LoadRom(romPath, romLoadAddress);

	auto ram = !ramPath.empty();
	if (ram)
		processor->LoadRam(ramPath, ramLoadAddress);

	if (resetStart)
		processor->Reset();
	else
		processor->Start(startAddress);

	symbols.reset(new Symbols(debugFile));

	disassembler.reset(new Disassembly(*processor, *symbols));
	Disassembled.push_back(std::bind(&Controller::Controller_Disassembled, this, _1));

	profiler.reset(new Profiler(*processor, *disassembler, *symbols, countInstructions, profileAddresses));
	profiler->StartingOutput.push_back(std::bind(&Controller::Profiler_StartingOutput, this));
	profiler->FinishedOutput.push_back(std::bind(&Controller::Profiler_FinishedOutput, this));
	profiler->StartingLineOutput.push_back(std::bind(&Controller::Profiler_StartingLineOutput, this));
	profiler->FinishedLineOutput.push_back(std::bind(&Controller::Profiler_FinishedLineOutput, this));
	profiler->StartingScopeOutput.push_back(std::bind(&Controller::Profiler_StartingScopeOutput, this));
	profiler->FinishedScopeOutput.push_back(std::bind(&Controller::Profiler_FinishedScopeOutput, this));
	profiler->EmitLine.push_back(std::bind(&Controller::Profiler_EmitLine, this, _1));
	profiler->EmitScope.push_back(std::bind(&Controller::Profiler_EmitScope, this, _1));
}

void Controller::Start() {
	processor->Run();
}

void Controller::Processor_Starting() {
	if (!disassemblyLogPath.empty())
		disassemblyLog.reset(new std::ofstream(disassemblyLogPath));
	startTime = std::chrono::system_clock::now();
}

void Controller::Processor_Finished() {
	finishTime = std::chrono::system_clock::now();
	profiler->Generate();
}

void Controller::Processor_ExecutingInstruction(const AddressEventArgs& addressEvent) {

	auto address = addressEvent.getAddress();
	auto cell = addressEvent.getCell();

	if (disassemble) {

		const auto& instruction = processor->getInstruction(cell);
		auto mode = instruction.mode;

		std::ostringstream output;

		output << std::endl << "[" << std::setw(9) << std::setfill('0') << processor->getCycles() << "] ";
		output << std::hex;
		output << "PC=" << std::setw(4) << std::setfill('0') << address << ":";
		output << "P=" << (std::string)processor->getP() << ", ";
		output << std::setw(2);
		output << "A=" << (int)processor->getA() << ", ";
		output << "X=" << (int)processor->getX() << ", ";
		output << "Y=" << (int)processor->getY() << ", ";
		output << "S=" << (int)processor->getS() << "\t";

		output << disassembler->Dump_ByteValue(cell);
		output << disassembler->DumpBytes(mode, address + 1);

		output << "\t ";

		output << disassembler->Disassemble(address);

		FireDelegates(Disassembled, output.str());
	}

	if (stopAddressEnabled && stopAddress == address)
		processor->setProceed(false);

	if (stopWhenLoopDetected) {
		if (oldPC == processor->getPC())
			processor->setProceed(false);
		else
			oldPC = processor->getPC();
	}
}

void Controller::Processor_ExecutedInstruction(const AddressEventArgs& addressEvent) {
	if (stopBreak && breakInstruction == addressEvent.getCell())
		processor->setProceed(false);
}

void Controller::Controller_Disassembled(const DisassemblyEventArgs& e) {
#if _DEBUG
	BufferDiagnosticsOutput(e.getOutput());
#endif
	if (disassemblyLog != nullptr)
		*disassemblyLog << e.getOutput();
}

void Controller::Processor_WritingByte(const AddressEventArgs& addressEvent) {
	if (addressEvent.getAddress() == outputAddress)
		HandleByteWritten(addressEvent.getCell());
}

void Controller::Processor_ReadingByte(const AddressEventArgs& addressEvent) {
	auto address = addressEvent.getAddress();
	if (address == inputAddress) {
		auto cell = addressEvent.getCell();
		if (cell != 0x0) {
			HandleByteRead(cell);
			processor->SetByte(address, 0x0);
		}
	}
}

void Controller::Processor_InvalidWriteAttempt(const AddressEventArgs& addressEvent) {
	std::ostringstream output;
	output << "Invalid write: ";
	auto address = addressEvent.getAddress();
	auto cell = addressEvent.getCell();
	output << std::hex << std::setw(4) << address << ":" << std::setw(2) << cell << std::endl;
	FireDelegates(Disassembled, output.str());
}

void Controller::Processor_Polling() {
	if (_kbhit()) {
		auto key = _getch();
		processor->SetByte(inputAddress, (uint8_t)key);
	}
}

void Controller::BufferDiagnosticsOutput(std::string output) {
#if _DEBUG
	for (auto character : output) {
		diagnosticsBuffer += character;
		if (character == '\n') {
			::OutputDebugStringA(diagnosticsBuffer.c_str());
			diagnosticsBuffer = "";
		}
	}
#endif
}

void Controller::HandleByteWritten(uint8_t cell) {
	auto character = (char)cell;
	if (bbcVduEmulation) {
		switch (cell)
		{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			break;
		case 7:
			//System.Console.Beep();
			break;
		case 8:
			//if (System.Console.CursorLeft > 0)
			//	--System.Console.CursorLeft;
			break;
		case 9:
			//if (System.Console.CursorLeft < System.Console.LargestWindowWidth)
			//	++System.Console.CursorLeft;
			break;
		case 10:
			//if (System.Console.CursorTop < System.Console.LargestWindowHeight)
			//	++System.Console.CursorTop;
			break;
		case 11:
			//if (System.Console.CursorTop > 0)
			//	--System.Console.CursorTop;
			break;
		case 12:
			//System.Console.Clear();
			break;
		case 13:
			//System.Console.CursorLeft = 0;
			break;
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		case 25:
		case 26:
		case 27:
		case 28:
		case 29:
			break;
		case 30:
			//System.Console.SetCursorPosition(0, 0);
			break;
		case 31:
			break;
		case 127:
			//--System.Console.CursorLeft;
			//System.Console.Write(character);
			break;
		default:
			_putch(character);
			break;
		}
	} else {
		_putch(character);
	}
#if _DEBUG
	std::ostringstream output;
	output << "Write: " << disassembler->Dump_ByteValue(cell) << ":" << character << std::endl;
	FireDelegates(Disassembled, output.str());
#endif
}

void Controller::HandleByteRead(uint8_t cell) {
	auto character = (char)cell;
#if _DEBUG
	std::ostringstream output;
	output << "Read: " << disassembler->Dump_ByteValue(cell) << ":" << character << std::endl;
	FireDelegates(Disassembled, output.str());
#endif
}

void Controller::Profiler_EmitScope(const ProfileScopeEventArgs& e) {
	auto cycles = e.getCycles();
	auto count = e.getCount();
	auto scope = e.getScope();
	auto proportion = (double)cycles / processor->getCycles();
	std::ostringstream output;
	output << "\t[";
	PrintPercentage(output, proportion * 100);
	output << "][" << std::setw(9) << cycles << "][" << count << "]\t" << scope << std::endl;
	FireDelegates(Disassembled, output.str());
}

void Controller::Profiler_EmitLine(const ProfileLineEventArgs& e) {
	auto cycles = e.getCycles();
	auto source = e.getSource();
	auto proportion = (double)cycles / processor->getCycles();
	std::ostringstream output;
	output << "\t[";
	PrintPercentage(output, proportion * 100);
	output << "][" << std::setw(9) << cycles << "]\t" << source << std::endl;
	FireDelegates(Disassembled, output.str());
}

void Controller::Profiler_FinishedScopeOutput() {
	BufferDiagnosticsOutput("Finished profiler scope output...\n");
}

void Controller::Profiler_StartingScopeOutput() {
	BufferDiagnosticsOutput("Starting profiler scope output...\n");
}

void Controller::Profiler_FinishedLineOutput() {
	BufferDiagnosticsOutput("Finished profiler line output...\n");
}

void Controller::Profiler_StartingLineOutput() {
	BufferDiagnosticsOutput("Starting profiler line output...\n");
}

void Controller::Profiler_FinishedOutput() {
	BufferDiagnosticsOutput("Finished profiler output...\n");
}

void Controller::Profiler_StartingOutput() {
	BufferDiagnosticsOutput("Starting profiler output...\n");
}
