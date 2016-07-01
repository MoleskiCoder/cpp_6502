﻿#include "stdafx.h"
#include "system6502.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <functional>

using namespace std::placeholders;

System6502::System6502(ProcessorType level, double processorSpeed, clock_t pollInterval)
:	MOS6502(level)
{
	speed = processorSpeed;

	cyclesPerSecond = speed * Mega;     // speed is in MHz
	cyclesPerMillisecond = cyclesPerSecond * Milli;
	cyclesPerInterval = (uint64_t)(cyclesPerMillisecond * pollInterval);

	Polling.push_back(std::bind(&System6502::System6502_Polling, this));
	Starting.push_back(std::bind(&System6502::System6502_Starting, this));
	Finished.push_back(std::bind(&System6502::System6502_Finished, this));
	ExecutedInstruction.push_back(std::bind(&System6502::System6502_ExecutedInstruction, this, _1));
}

void System6502::Initialise() {
	__super::Initialise();
	std::fill(locked.begin(), locked.end(), false);
	ClearMemory();
}

void System6502::LoadRom(std::string path, uint16_t offset) {
	auto length = LoadMemory(path, offset);
	LockMemory(offset, length);
}

void System6502::LoadRam(std::string path, uint16_t offset) {
	LoadMemory(path, offset);
}

void System6502::LockMemory(uint16_t offset, uint16_t length) {
	for (auto i = 0; i < length; ++i)
		locked[offset + i] = true;
}

void System6502::Run() {
	FireDelegates(Starting);
	__super::Run();
	FireDelegates(Finished);
}

uint8_t System6502::GetByte(uint16_t offset) {
	auto content = memory[offset];
	AddressEventArgs e(offset, content);
	FireDelegates(ReadingByte, e);
	return content;
}

void System6502::SetByte(uint16_t offset, uint8_t value) {
	AddressEventArgs e(offset, value);
	if (locked[offset]) {
		FireDelegates(InvalidWriteAttempt, e);
	} else {
		memory[offset] = value;
		FireDelegates(WritingByte, e);
	}
}

void System6502::Execute(uint8_t cell) {

	// XXXX Fetch byte has already incremented PC.
	auto executingAddress = (uint16_t)(pc - 1);

	AddressEventArgs e(executingAddress, cell);
	FireDelegates(ExecutingInstruction, e);
	__super::Execute(cell);
	FireDelegates(ExecutedInstruction, e);
}

void System6502::CheckPoll() {
	if ((cycles % cyclesPerInterval) == 0)
		FireDelegates(Polling);
}

void System6502::System6502_ExecutedInstruction(const AddressEventArgs&) {
	CheckPoll();
}

void System6502::System6502_Starting() {
	startTime = std::chrono::high_resolution_clock::now();
	running = true;
}

void System6502::System6502_Finished() {
	running = false;
}

void System6502::System6502_Polling() {
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed = now - startTime;
	auto timerCurrent = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();

	auto cyclesAllowed = timerCurrent * cyclesPerMillisecond;
	auto cyclesMismatch = cycles - cyclesAllowed;
	if (cyclesMismatch > 0.0) {
		auto delay = cyclesMismatch / cyclesPerMillisecond;
		if (delay > 0) {
			heldCycles += (uint64_t)cyclesMismatch;
			std::this_thread::sleep_for(std::chrono::milliseconds((long long)delay));
		}
	}
}

void System6502::ClearMemory() {
	std::fill(memory.begin(), memory.end(), 0);
}

uint16_t System6502::LoadMemory(std::string path, uint16_t offset) {
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	auto size = (int)file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(&buffer[0], size);
	file.close();

	std::copy(buffer.begin(), buffer.end(), memory.begin() + offset);

	return (uint16_t)size;
}