#include "stdafx.h"
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

	Polling.connect(std::bind(&System6502::System6502_Polling, this));
	Starting.connect(std::bind(&System6502::System6502_Starting, this));
	Finished.connect(std::bind(&System6502::System6502_Finished, this));
	ExecutedInstruction.connect(std::bind(&System6502::System6502_ExecutedInstruction, this, _1));
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
	Starting.fire(EventArgs());
	__super::Run();
	Finished.fire(EventArgs());
}

uint8_t System6502::GetByte(uint16_t offset) const {
	auto content = memory[offset];
	ReadingByte.fire(AddressEventArgs(offset, content));
	return content;
}

void System6502::SetByte(uint16_t offset, uint8_t value) {
	AddressEventArgs e(offset, value);
	if (locked[offset]) {
		InvalidWriteAttempt.fire(e);
	} else {
		memory[offset] = value;
		WritingByte.fire(e);
	}
}

void System6502::Execute(uint8_t cell) {

	// XXXX Fetch byte has already incremented PC.
	auto executingAddress = (uint16_t)(getPC() - 1);

	AddressEventArgs e(executingAddress, cell);
	ExecutingInstruction.fire(e);
	__super::Execute(cell);
	ExecutedInstruction.fire(e);
}

void System6502::CheckPoll() {
	if ((getCycles() % cyclesPerInterval) == 0)
		Polling.fire(EventArgs());
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
	auto cyclesMismatch = getCycles() - cyclesAllowed;
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
