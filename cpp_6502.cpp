// cpp_6502.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "mos6502.h"

int main() {

	mos6502 processor;

	processor.clearMemory();
	processor.resetRegisters();

	processor.PC = 0x4000;
	processor.readRom("C:\\github\\cpp\\cpp_6502\\AllSuiteA.bin", 0x4000);

	//processor.PC = 0xc000;
	//processor.readRom("C:\\github\\cpp\\cpp_6502\\ehbasic.bin", 0xc000);

	processor.run();

	return 0;
}