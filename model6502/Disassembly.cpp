#include "stdafx.h"
#include "Disassembly.h"

#include <sstream>
#include <iomanip>
#include <functional>

using namespace std::placeholders;

Disassembly::Disassembly(MOS6502& targetProcessor, Symbols& targetSymbols)
:	processor(targetProcessor),
	symbols(targetSymbols)
{
	dumpers = {
		{ AddressingMode::Illegal,				{ std::bind(&Disassembly::Dump_Nothing, this, _1),	std::bind(&Disassembly::Dump_Nothing, this, _1)	} },
		{ AddressingMode::Implied,				{ std::bind(&Disassembly::Dump_Nothing, this, _1),	std::bind(&Disassembly::Dump_Nothing, this, _1)	} },
		{ AddressingMode::Accumulator,			{ std::bind(&Disassembly::Dump_Nothing, this, _1),	std::bind(&Disassembly::Dump_A, this, _1)			} },
		{ AddressingMode::Immediate,			{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_imm, this, _1)		} },
		{ AddressingMode::Relative,				{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_rel, this, _1)		} },
		{ AddressingMode::XIndexed,				{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_xind, this, _1)		} },
		{ AddressingMode::IndexedY,				{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_indy, this, _1)		} },
		{ AddressingMode::ZeroPageIndirect,		{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_zpind, this, _1)		} },
		{ AddressingMode::ZeroPage,				{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_zp, this, _1)		} },
		{ AddressingMode::ZeroPageX,			{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_zpx, this, _1)		} },
		{ AddressingMode::ZeroPageY,			{ std::bind(&Disassembly::Dump_Byte, this, _1),		std::bind(&Disassembly::Dump_zpy, this, _1)		} },
		{ AddressingMode::Absolute,				{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_abs, this, _1)		} },
		{ AddressingMode::AbsoluteX,			{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_absx, this, _1)		} },
		{ AddressingMode::AbsoluteY,			{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_absy, this, _1)		} },
		{ AddressingMode::AbsoluteXIndirect,	{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_absxind, this, _1)	} },
		{ AddressingMode::Indirect,				{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_ind, this, _1)		} },
		{ AddressingMode::ZeroPageRelative,		{ std::bind(&Disassembly::Dump_DByte, this, _1),	std::bind(&Disassembly::Dump_zprel, this, _1)		} },
	};
}

std::string Disassembly::Dump_ByteValue(uint8_t value) {
	std::ostringstream output;
	output << std::hex << std::setw(2) << std::setfill('0') << (int)value;
	return output.str();
}

std::string Disassembly::DumpBytes(AddressingMode mode, uint16_t current) {
	return dumpers[mode].byteDumper(current);
}

std::string Disassembly::Disassemble(uint16_t current) {

	std::ostringstream output;

	auto content = processor.GetByte(current);
	auto instruction = processor.instructions[content];

	auto mode = instruction.mode;
	auto mnemomic = instruction.display;

	auto operand = DumpOperand(mode, current + 1);

	auto label = symbols.labels.find(current);
	if (label != symbols.labels.end())
		output << label->second << ": ";
	output << mnemomic << " " << operand;

	return output.str();
}

std::string Disassembly::DumpOperand(AddressingMode mode, uint16_t current) {
	return dumpers[mode].disassemblyDumper(current);
}

////

uint8_t Disassembly::GetByte(uint16_t address) {
	return processor.GetByte(address);
}

uint16_t Disassembly::GetWord(uint16_t address) {
	return processor.GetWord(address);
}

////

std::string Disassembly::Dump_Nothing(uint16_t) {
	return "";
}

std::string Disassembly::Dump_Byte(uint16_t address) {
	return Dump_ByteValue(GetByte(address));
}

std::string Disassembly::Dump_DByte(uint16_t address) {
	return Dump_Byte(address) + Dump_Byte(address + 1);
}

////

std::string Disassembly::ConvertAddress(uint16_t address) {
	auto label = symbols.labels.find(address);
	if (label != symbols.labels.end())
		return label->second;
	std::ostringstream output;
	output << "$" << Dump_DByte(address);
	return output.str();
}

std::string Disassembly::ConvertAddress(uint8_t address) {
	auto label = symbols.labels.find(address);
	if (label != symbols.labels.end())
		return label->second;
	std::ostringstream output;
	output << "$" << Dump_Byte(address);
	return output.str();
}

std::string Disassembly::ConvertConstant(uint16_t constant) {
	auto label = symbols.constants.find(constant);
	if (label != symbols.constants.end())
		return label->second;
	return Dump_DByte(constant);
}

std::string Disassembly::ConvertConstant(uint8_t constant) {
	auto label = symbols.constants.find(constant);
	if (label != symbols.constants.end())
		return label->second;
	return Dump_Byte(constant);
}

////

std::string Disassembly::Dump_A(uint16_t) {
	return "A";
}

std::string Disassembly::Dump_imm(uint16_t current) {
	std::ostringstream output;
	auto immediate = GetByte(current);
	output << "#" << ConvertConstant(immediate);
	return output.str();
}

std::string Disassembly::Dump_abs(uint16_t current) {
	auto address = GetWord(current);
	return ConvertAddress(address);
}

std::string Disassembly::Dump_zp(uint16_t current) {
	auto zp = GetByte(current);
	return ConvertAddress(zp);
}

std::string Disassembly::Dump_zpx(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	output << ConvertAddress(zp) << ",X";
	return output.str();
}

std::string Disassembly::Dump_zpy(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	output << ConvertAddress(zp) << ",Y";
	return output.str();
}

std::string Disassembly::Dump_absx(uint16_t current) {
	std::ostringstream output;
	auto address = GetWord(current);
	output << ConvertAddress(address) << ",X";
	return output.str();
}

std::string Disassembly::Dump_absy(uint16_t current) {
	std::ostringstream output;
	auto address = GetWord(current);
	output << ConvertAddress(address) << ",Y";
	return output.str();
}

std::string Disassembly::Dump_absxind(uint16_t current) {
	std::ostringstream output;
	auto address = GetWord(current);
	output << "(" << ConvertAddress(address) << ",X)";
	return output.str();
}

std::string Disassembly::Dump_xind(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	output << "(" << ConvertAddress(zp) << ",X)";
	return output.str();
}

std::string Disassembly::Dump_indy(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	output << "(" << ConvertAddress(zp) << "),Y)";
	return output.str();
}

std::string Disassembly::Dump_ind(uint16_t current) {
	std::ostringstream output;
	auto address = GetWord(current);
	output << "(" << ConvertAddress(address) << ")";
	return output.str();
}

std::string Disassembly::Dump_zpind(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	output << "(" << ConvertAddress(zp) << ")";
	return output.str();
}

std::string Disassembly::Dump_rel(uint16_t current) {
	uint16_t relative = 1 + current + (int8_t)GetByte(current);
	return ConvertAddress(relative);
}

std::string Disassembly::Dump_zprel(uint16_t current) {
	std::ostringstream output;
	auto zp = GetByte(current);
	int8_t displacement = GetByte(current + 1);
	uint16_t address = 1 + current + displacement;
	output << ConvertAddress(zp) << "," << ConvertAddress(address);
	return output.str();
}
 