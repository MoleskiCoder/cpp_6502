// cpp_6502.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "mos6502.h"

int main() {

	mos6502 processor;

	processor.clearMemory();
	processor.readRom("C:\\github\\6502\\cpp_6502\\ehbasic.bin", 0xc000);
	processor.resetRegisters();
	processor.run(0xc000);

	return 0;
}