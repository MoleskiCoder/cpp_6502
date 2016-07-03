#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <functional>

#include "mos6502.h"
#include "EventArgs.h"
#include "AddressEventArgs.h"
#include "Signal.h"

class System6502 : public MOS6502
{
public:
	const double Mega = 1000000;
	const double Milli = 0.001;

	uint64_t getHeldCycles() const { return heldCycles; }

public:
	System6502(ProcessorType level, double speed, clock_t pollInterval);

	virtual void Initialise();

	void LoadRom(std::string path, uint16_t offset);
	void LoadRam(std::string path, uint16_t offset);
	void LockMemory(uint16_t offset, uint16_t length);

	virtual void Run();

	Signal<EventArgs> Starting;
	Signal<EventArgs> Finished;

	Signal<EventArgs> Polling;

	Signal<AddressEventArgs> InvalidWriteAttempt;
	Signal<AddressEventArgs> WritingByte;
	Signal<AddressEventArgs> ReadingByte;
	Signal<AddressEventArgs> ExecutingInstruction;
	Signal<AddressEventArgs> ExecutedInstruction;

	virtual uint8_t GetByte(uint16_t offset) const;
	virtual void SetByte(uint16_t offset, uint8_t value);

protected:
	virtual void Execute(uint8_t cell);

private:
	void ClearMemory();
	uint16_t LoadMemory(std::string path, uint16_t offset);

	void CheckPoll();

	void System6502_Starting();
	void System6502_Finished();
	void System6502_Polling();
	void System6502_ExecutedInstruction(const AddressEventArgs&);

	double speed;  // Speed in MHz, e.g. 2.0 == 2Mhz, 1.79 = 1.79Mhz

	double cyclesPerSecond;
	double cyclesPerMillisecond;
	uint64_t cyclesPerInterval;

	uint64_t heldCycles = 0;

	bool running;

	std::array<uint8_t, 0x10000> memory;
	std::array<bool, 0x10000> locked;

	std::chrono::high_resolution_clock::time_point startTime;
};
