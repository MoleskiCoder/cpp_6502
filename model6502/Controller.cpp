#include "stdafx.h"
#include "Controller.h"

#include <fstream>

#include <conio.h>
#include <sstream>
#include <iomanip>

#ifdef _DEBUG
#	include <Windows.h>
#endif

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

	if (disassemble || stopAddressEnabled || stopWhenLoopDetected || profileAddresses || stopBreak)
		processor->ExecutingInstruction.connect(std::bind(&Controller::Processor_ExecutingInstruction, this, std::placeholders::_1));

	processor->getMemory().WritingByte.connect(std::bind(&Controller::Processor_WritingByte, this, std::placeholders::_1));
	processor->getMemory().ReadingByte.connect(std::bind(&Controller::Processor_ReadingByte, this, std::placeholders::_1));
	processor->getMemory().InvalidWriteAttempt.connect(std::bind(&Controller::Processor_InvalidWriteAttempt, this, std::placeholders::_1));
	processor->Starting.connect(std::bind(&Controller::Processor_Starting, this));
	processor->Finished.connect(std::bind(&Controller::Processor_Finished, this));
	processor->Polling.connect(std::bind(&Controller::Processor_Polling, this));

	processor->Initialise();

	auto bbc = !bbcLanguageRomPath.empty() && !bbcOSRomPath.empty();
	if (bbc) {
		processor->getMemory().LoadRom(bbcOSRomPath, BbcOSLoadAddress);
		processor->getMemory().LoadRom(bbcLanguageRomPath, BbcOSLanguageAddress);
	}

	auto rom = !romPath.empty();
	if (rom)
		processor->getMemory().LoadRom(romPath, romLoadAddress);

	auto ram = !ramPath.empty();
	if (ram)
		processor->getMemory().LoadRam(ramPath, ramLoadAddress);

	if (resetStart)
		processor->Reset();
	else
		processor->Start(startAddress);

	symbols.reset(new Symbols(debugFile));

	disassembler.reset(new Disassembly(*processor, *symbols));
	Disassembled.connect(std::bind(&Controller::Controller_Disassembled, this, std::placeholders::_1));

	profiler.reset(new Profiler(*processor, *disassembler, *symbols, countInstructions, profileAddresses));
	profiler->StartingOutput.connect(std::bind(&Controller::Profiler_StartingOutput, this));
	profiler->FinishedOutput.connect(std::bind(&Controller::Profiler_FinishedOutput, this));
	profiler->StartingLineOutput.connect(std::bind(&Controller::Profiler_StartingLineOutput, this));
	profiler->FinishedLineOutput.connect(std::bind(&Controller::Profiler_FinishedLineOutput, this));
	profiler->StartingScopeOutput.connect(std::bind(&Controller::Profiler_StartingScopeOutput, this));
	profiler->FinishedScopeOutput.connect(std::bind(&Controller::Profiler_FinishedScopeOutput, this));
	profiler->EmitLine.connect(std::bind(&Controller::Profiler_EmitLine, this, std::placeholders::_1));
	profiler->EmitScope.connect(std::bind(&Controller::Profiler_EmitScope, this, std::placeholders::_1));
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
	if (profileAddresses)
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

		Disassembled.fire(DisassemblyEventArgs(output.str()));
	}

	if (stopAddressEnabled && stopAddress == address)
		processor->setProceed(false);

	if (stopWhenLoopDetected) {
		if (oldPC == processor->getPC())
			processor->setProceed(false);
		else
			oldPC = processor->getPC();
	}

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
	Disassembled.fire(DisassemblyEventArgs(output.str()));
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

#ifdef _DEBUG
	std::ostringstream output;
#endif

	auto character = (char)cell;
	if (bbcVduEmulation) {

#ifdef _DEBUG
		output << std::endl << "Write (BBC): ";
#endif

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
#ifdef _DEBUG
			output << "7 Beep";
#endif
			//System.Console.Beep();
			break;
		case 8:
#ifdef _DEBUG
			output << "8 Cursor left";
#endif
			//if (System.Console.CursorLeft > 0)
			//	--System.Console.CursorLeft;
			break;
		case 9:
#ifdef _DEBUG
			output << "9 Cursor right";
#endif
			//if (System.Console.CursorLeft < System.Console.LargestWindowWidth)
			//	++System.Console.CursorLeft;
			break;
		case 10:
#ifdef _DEBUG
			output << "10 Cursor down (LF)";
#endif
			_putch(character);
			break;
		case 11:
#ifdef _DEBUG
			output << "11 Cursor up";
#endif
			//if (System.Console.CursorTop > 0)
			//	--System.Console.CursorTop;
			break;
		case 12:
#ifdef _DEBUG
			output << "12 CLS";
#endif
			//System.Console.Clear();
			break;
		case 13:
#ifdef _DEBUG
			output << "13 CR";
#endif
			_putch(character);
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
#ifdef _DEBUG
			output << (int)cell << " Unimplemented";
#endif
			break;
		case 30:
#ifdef _DEBUG
			output << "30 Home";
#endif
			//System.Console.SetCursorPosition(0, 0);
			break;
		case 31:
			break;
		case 127:
#ifdef _DEBUG
			output << "127 Backspace";
#endif
			//--System.Console.CursorLeft;
			//System.Console.Write(character);
			break;
		default:
#ifdef _DEBUG
			output << disassembler->Dump_ByteValue(cell) << ":" << character;
#endif
			_putch(character);
			break;
		}
	} else {
#ifdef _DEBUG
		output << std::endl << "Write: " << disassembler->Dump_ByteValue(cell) << ":" << character;
#endif
		_putch(character);
	}
#ifdef _DEBUG
	Disassembled.fire(DisassemblyEventArgs(output.str()));
#endif
}

void Controller::HandleByteRead(uint8_t cell) {
	auto character = (char)cell;
#if _DEBUG
	std::ostringstream output;
	output << "Read: " << disassembler->Dump_ByteValue(cell) << ":" << character << std::endl;
	Disassembled.fire(DisassemblyEventArgs(output.str()));
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
	Disassembled.fire(DisassemblyEventArgs(output.str()));
}

void Controller::Profiler_EmitLine(const ProfileLineEventArgs& e) {
	auto cycles = e.getCycles();
	auto source = e.getSource();
	auto proportion = (double)cycles / processor->getCycles();
	std::ostringstream output;
	output << "\t[";
	PrintPercentage(output, proportion * 100);
	output << "][" << std::setw(9) << cycles << "]\t" << source << std::endl;
	Disassembled.fire(DisassemblyEventArgs(output.str()));
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
