// cpp_6502.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "mos6502.h"

int main() {

	mos6502 processor;

	processor.clearMemory();
	processor.resetRegisters();


#ifdef TEST_SUITE1
	// Test suite one
	processor.PC = 0x4000;
	processor.readRom("C:\\github\\cpp\\cpp_6502\\AllSuiteA.bin", 0x4000);
#endif

#ifdef TEST_SUITE2
	// Test suite two: https://github.com/Klaus2m5/6502_65C02_functional_tests
	processor.PC = 0x400;
	processor.readRom("C:\\github\\cpp\\cpp_6502\\6502_functional_test.bin", 0x0);
#endif

#ifdef EHBASIC
	// EHBASIC
	processor.PC = 0xc000;
	processor.readRom("C:\\github\\cpp\\cpp_6502\\ehbasic.bin", 0xc000);
#endif

	processor.run();

	return 0;
}