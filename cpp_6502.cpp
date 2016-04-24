// cpp_6502.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "mos6502.h"

#include <ctime>
#include <iostream>
#include <fstream>

void readRom(mos6502& processor, std::string path, size_t offset)
{
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	auto size = (int)file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(&buffer[0], size);
	file.close();

	std::copy(buffer.begin(), buffer.end(), processor.memory.begin() + offset);
}

int main() {

	mos6502 processor;

	processor.clearMemory();
	processor.resetRegisters();


#ifdef TEST_SUITE1
	// Test suite one
	processor.PC = 0x4000;
	readRom(processor, "C:\\github\\cpp\\cpp_6502\\AllSuiteA.bin", 0x4000);
#endif

#ifdef TEST_SUITE2
	// Test suite two: https://github.com/Klaus2m5/6502_65C02_functional_tests
	processor.PC = 0x400;
	readRom(processor, "C:\\github\\cpp\\cpp_6502\\6502_functional_test.bin", 0x0);
#endif

#ifdef EHBASIC
	// EHBASIC
	processor.PC = 0xc000;
	readRom(processor, "C:\\github\\cpp\\cpp_6502\\ehbasic.bin", 0xc000);
#endif

	auto start = std::clock();
	processor.run();
	auto finish = std::clock();

	auto elapsed = finish - start;

	auto seconds = (elapsed % CLOCKS_PER_SEC) / double(CLOCKS_PER_SEC) + (elapsed / CLOCKS_PER_SEC);
	std::cout << std::endl << std::endl << "Time taken " << seconds << std::endl;

	auto cyclesPerSecond = processor.cycles / seconds;
	std::cout << std::endl << std::endl << "Cycles per second " << cyclesPerSecond << std::endl;

	auto speedup = cyclesPerSecond / 2000000;
	std::cout << std::endl << std::endl << "Speedup over 2Mhz 6502 " << speedup << std::endl;

	return 0;
}