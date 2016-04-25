#include "stdafx.h"
#include "system6502.h"

#include <conio.h>

system6502::system6502()
: memory(0x10000)
{
}


system6502::~system6502()
{
}

bool system6502::execute(uint8_t instruction)
{
	poll();

#ifdef TEST_SUITE1
	if (PC == 0x45c0)
	{
		auto test = getByte(0x0210);
		if (test == 0xff)
			printf("\n** success!!");
		else
			printf("\n** failed!!");
		return false;
	}
#endif

#ifdef TEST_SUITE2
	auto test = getByte(0x0200);
	if (oldPC == PC)
	{
		printf("\n** PC=%04x: test=%02x: stopped!!", PC, test);
		return false;
	}
	else
	{
		oldPC = PC;
	}
#endif

	instructionCounts[instruction]++;
	return __super::execute(instruction);
}

void system6502::clear()
{
	clearMemory();
	resetRegisters();

#ifdef TEST_SUITE2
	oldPC = (uint16_t)-1;
#endif
}

void system6502::clearMemory()
{
	std::fill(memory.begin(), memory.end(), 0);
}

void system6502::loadRom(std::string path, size_t offset)
{
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	auto size = (int)file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(&buffer[0], size);
	file.close();

	std::copy(buffer.begin(), buffer.end(), memory.begin() + offset);
}

uint8_t system6502::getByte(uint16_t offset)
{
	return memory[offset];
}

void system6502::setByte(uint16_t offset, uint8_t value)
{
	memory[offset] = value;
}

void system6502::poll()
{
#ifdef EHBASIC
	pollInput();
	pollOutput();
#endif
}

void system6502::pollInput()
{
	setByte(input, 0x0);
	static uint64_t interval = 0;
	if (++interval % 1000000 == 0)
	{
		if (_kbhit())
		{
			auto key = _getch();
			setByte(input, (uint8_t)key);
		}
	}
}

void system6502::pollOutput()
{
	auto character = getByte(output);
	if (character != 0x0)
		printf("%c", character);
}
