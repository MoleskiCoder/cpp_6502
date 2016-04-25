// cpp_6502.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "system6502.h"

#include <ctime>
#include <iostream>
#include <fstream>

int main() {

	system6502 processor;

	processor.clear();

	auto start = std::clock();

#ifdef TEST_SUITE1
	// Test suite one
	processor.loadRom("C:\\github\\cpp\\cpp_6502\\AllSuiteA.bin", 0x4000);
	processor.start(0x4000);
#endif

#ifdef TEST_SUITE2
	// Test suite two: https://github.com/Klaus2m5/6502_65C02_functional_tests
	processor.loadRom("C:\\github\\cpp\\cpp_6502\\6502_functional_test.bin", 0x0);
	processor.start(0x400);
#endif

#ifdef EHBASIC
	// EHBASIC
	processor.loadRom("C:\\github\\cpp\\cpp_6502\\ehbasic.bin", 0xc000);
	processor.reset();
#endif

	auto finish = std::clock();

	auto elapsed = finish - start;

	auto seconds = (elapsed % CLOCKS_PER_SEC) / double(CLOCKS_PER_SEC) + (elapsed / CLOCKS_PER_SEC);
	std::cout << std::endl << std::endl << "Time taken " << seconds << std::endl;

	auto cyclesPerSecond = processor.getCycles() / seconds;
	std::cout << std::endl << std::endl << "Cycles per second " << cyclesPerSecond << std::endl;

	auto speedup = cyclesPerSecond / 2000000;
	std::cout << std::endl << std::endl << "Speedup over 2Mhz 6502 " << speedup << std::endl;

	return 0;
}