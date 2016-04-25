#pragma once
#include "mos6502.h"
#include <map>

class system6502 : public mos6502
{
public:
	system6502();
	~system6502();

	void clear();
	void loadRom(std::string path, size_t offset);

protected:
	void clearMemory();

	virtual bool execute(uint8_t instruction);

	virtual uint8_t getByte(uint16_t offset);
	virtual void setByte(uint16_t offset, uint8_t value);

private:
	uint16_t oldPC;
	std::vector<uint8_t> memory;
	std::map<uint8_t, int> instructionCounts;

	uint16_t input = 0xF004;
	uint16_t output = 0xF001;

	void poll();
#ifdef EHBASIC
	void pollInput();
#endif
};

