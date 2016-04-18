#include "stdafx.h"
#include "mos6502.h"

#include <fstream>
#include <iterator>
#include <algorithm>
#include <assert.h>

mos6502::mos6502()
:	memory(0xffff)
{
}

mos6502::~mos6502()
{
}

void mos6502::readRom(std::string path, size_t offset)
{
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	auto size = (int)file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(&buffer[0], size);
	file.close();

	std::copy(buffer.begin(), buffer.end() - 1, memory.begin() + offset);
}

void mos6502::clearMemory()
{
	std::fill(memory.begin(), memory.end(), 0);
}

void mos6502::resetRegisters()
{
	PC = 0;
	X = Y = A = P = 0;
	S = 0xff;
}

//

uint8_t mos6502::getByte(uint16_t offset)
{
	return memory[offset];
}

uint16_t mos6502::getWord(uint16_t offset)
{
	auto low = memory[offset];
	auto high = memory[offset + 1];
	return makeWord(low, high);
}

//

uint8_t mos6502::fetchByte(uint16_t& counter)
{
	return getByte(counter++);
}

uint8_t mos6502::fetchByte()
{
	return fetchByte(PC);
}

uint16_t mos6502::fetchWord(uint16_t& counter)
{
	auto word = getWord(counter);
	counter += 2;
	return word;
}

uint16_t mos6502::fetchWord()
{
	return fetchWord(PC);
}

uint16_t mos6502::fetchWord_Indirect()
{
	auto indirection = fetchWord();
	return fetchWord(indirection);
}

//

uint8_t mos6502::readByte_Immediate()
{
	return fetchByte();
}

uint8_t mos6502::readByte_ZeroPage()
{
	return ZEROPAGE;
}

uint8_t mos6502::readByte_Absolute()
{
	return ABSOLUTE;
}

uint8_t mos6502::readByte_ZeroPageX()
{
	return ZEROPAGEX;
}

uint8_t mos6502::readByte_ZeroPageY()
{
	return ZEROPAGEY;
}

uint8_t mos6502::readByte_AbsoluteX()
{
	return ABSOLUTEX;
}

uint8_t mos6502::readByte_AbsoluteY()
{
	return ABSOLUTEY;
}

//

void mos6502::writeByte_ZeroPage(uint8_t value)
{
	ZEROPAGE = value;
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	ABSOLUTE = value;
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	ZEROPAGEX = value;
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	ZEROPAGEY = value;
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	ABSOLUTEX = value;
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	ABSOLUTEY = value;
}

//

void mos6502::updateFlag_Zero(uint8_t value)
{
	if (value == 0)
		P |= F_Z;
}

void mos6502::updateFlag_Negative(int8_t value)
{
	if (value < 0)
		P |= F_N;
}

//

void mos6502::updateFlags_ZeroNegative(uint8_t value)
{
	updateFlag_Zero(value);
	updateFlag_Negative(value);
}

void mos6502::reflectFlags_ZeroNegative(uint8_t value)
{
	P &= ~(F_N & F_Z);
	updateFlags_ZeroNegative(value);
}

//

void mos6502::CMP(uint8_t first, uint8_t second)
{
	P &= ~(F_N | F_Z | F_C);

	uint16_t result = first - second;

	if (!result)
		P |= F_Z;
	else
		if (int8_t(result) < 0)
			P |= F_N;

	if (!(result & 0xff00))
		P |= F_C;
}

void mos6502::BIT(uint8_t data)
{
	P &= ~(F_Z & F_V & F_N);

	if (A & data)
		P |= F_Z;

	if (data & F_V)
		P |= F_V;

	if (data & F_N)
		P |= F_N;
}

void mos6502::CPX(uint8_t data)
{
	CMP(X, data);
}

void mos6502::CPY(uint8_t data)
{
	CMP(Y, data);
}

void mos6502::LDY(uint8_t data)
{
	Y = data;
	reflectFlags_ZeroNegative(Y);
}

void mos6502::ORA(uint8_t data)
{
	A |= data;
	reflectFlags_ZeroNegative(A);
}

void mos6502::AND(uint8_t data)
{
	A &= data;
	reflectFlags_ZeroNegative(A);
}

void mos6502::EOR(uint8_t data)
{
	A ^= data;
	reflectFlags_ZeroNegative(A);
}

void mos6502::ADC(uint8_t data)
{
	uint8_t carry = (P & F_C ? 0 : 1);

	uint16_t sum = A + data + carry;
	P &= ~(F_N | F_V | F_Z | F_C);

	if (!(uint8_t)sum)
		P |= F_Z;
	else
		if ((int8_t)sum < 0)
			P |= F_N;

	if (~(A ^ data) & (A ^ sum) & 0x80)
		P |= F_V;

	if (sum & 0xff00)
		P |= F_C;

	A = sum;
}

void mos6502::LDA(uint8_t data)
{
	A = data;
	reflectFlags_ZeroNegative(A);
}

void mos6502::CMP(uint8_t data)
{
	CMP(A, data);
}

void mos6502::SBC(uint8_t data)
{
	uint8_t carry = (P & F_C ? 0 : 1);
	uint16_t difference = A - data - carry;

	P &= ~(F_Z & F_V & F_N & F_C);

	if (!((uint8_t)difference))
		P |= F_Z;
	else
		if ((int8_t)difference < 0)
			P |= F_N;

	if ((A ^ data) & (A ^ difference) & 0x80)
		P |= F_V;

	if (!(difference & 0xff00))
		P |= F_C;

	A = (uint8_t)difference;
}

void mos6502::LDX(uint8_t data)
{
	X = data;
	reflectFlags_ZeroNegative(X);
}

void mos6502::BPL(int8_t data)
{
	if (!(P & F_N)) {
		PC += data;
	}
}

void mos6502::BEQ(int8_t data)
{
	if (P & F_Z) {
		PC += data;
	}
}

void mos6502::BNE(int8_t data)
{
	if (!(P & F_Z)) {
		PC += data;
	}
}

void mos6502::BCC(int8_t data)
{
	if (!(P & F_C)) {
		PC += data;
	}
}

void mos6502::BCS(int8_t data)
{
	if (P & F_C) {
		PC += data;
	}
}

void mos6502::DEC(uint16_t offset)
{
	int8_t content = memory[offset];
	memory[offset] = --content;
	reflectFlags_ZeroNegative(content);
}

void mos6502::DEX()
{
	reflectFlags_ZeroNegative(--X);
}

void mos6502::DEY()
{
	reflectFlags_ZeroNegative(--Y);
}

void mos6502::INC(uint16_t offset)
{
	int8_t content = memory[offset];
	memory[offset] = ++content;
	reflectFlags_ZeroNegative(content);
}

void mos6502::INX()
{
	reflectFlags_ZeroNegative(++X);
}

void mos6502::INY()
{
	reflectFlags_ZeroNegative(++Y);
}

void mos6502::JSR()
{
	auto destination = fetchWord();
	pushWord(PC + 1);
	PC = destination;
}

void mos6502::RTS()
{
	PC = popWord();
}

void mos6502::PHA()
{
	pushByte(A);
}

void mos6502::PLA()
{
	A = popByte();
	reflectFlags_ZeroNegative(A);
}

void mos6502::TXS()
{
	S = X;
}

void mos6502::TAY()
{
	Y = A;
	reflectFlags_ZeroNegative(Y);
}

uint8_t mos6502::ASL(uint8_t data)
{
	P &= ~(F_N | F_Z | F_C);

	uint8_t result = data << 1;
	if (!result)
		P |= F_Z;
	else
		if ((uint8_t)result < 0)
			P |= F_N;

	if (data & 0x80)
		P |= F_C;

	return result;
}

uint8_t mos6502::ROL(uint8_t data)
{
	bool carry = P & F_C;

	P &= ~(F_N | F_Z | F_C);

	if (data & 0x80)
		P |= F_C;

	uint8_t result = data << 1;

	if (carry)
		result |= 0x01;

	if (!result)
		P |= F_Z;
	else
		if ((int8_t)result < 0)
			P |= F_N;

	return result;
}

void mos6502::CLC()
{
	P &= ~F_C;
}

//

void mos6502::run(uint16_t offset) {

	auto class_mask = 0b11;
	auto addressing_mode_mask = 0b11100;
	auto op_code_mask = 0b11100000;

	PC = offset;

	for (;;) {
		auto current = fetchByte();

#ifdef _DEBUG
		instructionCounts[current]++;
#endif

		auto classification = current & class_mask;
		auto addressing_mode = (current & addressing_mode_mask) >> 2;
		auto op_code = (current & op_code_mask) >> 5;

#ifdef _DEBUG
		printf("INS=%02x: PC=%04x, A=%02x, X=%02x, Y=%02x, S=%02x\n", current, PC - 1, A, X, Y, S);
#endif

		switch (classification) {

		case 0b00:
			/*
			000	#immediate
			001	zero page
			011	absolute
			101	zero page,X
			111	absolute,X
			*/
			switch (op_code) {

			case 0b000:	// BPL
				switch (addressing_mode) {
				case 0b100:
					BPL(readByte_Immediate());
					break;

				case 0b110:	 // CLC
					CLC();
					break;

				default:
					assert(false && "unknown BPL addressing mode");
				}
				break;

			case 0b001:	//	BIT
				switch (addressing_mode) {
				case 0b001:
					BIT(readByte_ZeroPage());
					break;
				case 0b011:
					BIT(readByte_Absolute());
					break;

				case 0b000: // JSR
					JSR();
					break;

				default:
					assert(false && "unknown BIT addressing mode");
				}
				break;

			case 0b010:
				switch (addressing_mode)
				{
				case 0b010: // PHA
					PHA();
					break;
				//case 0b???:	// JMP absolute
					//PC = fetchWord();
					//break;
				default:
					assert(false && "unknown instruction");
				}
				break;

			case 0b011:
				switch (addressing_mode)
				{
				case 0b000: // RTS
					RTS();
					break;
				case 0b010:	// PLA
					PLA();
					break;
				case 0b011: // JMP (indirect)
					PC = ABSOLUTE;
					break;
				default:
					assert(false && "unknown instruction");
				}
				break;

			case 0b100:	//	DEY
				switch (addressing_mode)
				{
				case 0b010:	// Implied DEY
					DEY();
					break;

				case 0b100: // BCC rel
					BCC(readByte_Immediate());
					break;

				case 0b001: // STY zero page
					writeByte_ZeroPage(Y);
					break;

				case 0b110: // TAY
					TAY();
					break;

				default:
					assert(false && "unknown DEY addressing mode");
				}
				break;

			case 0b101:	//	LDY
				switch (addressing_mode) {
				case 0b000:
					LDY(readByte_Immediate());
					break;
				case 0b001:
					LDY(readByte_ZeroPage());
					break;
				case 0b011:
					LDY(readByte_Absolute());
					break;
				case 0b101:
					LDY(readByte_ZeroPageX());
					break;
				case 0b111:
					LDY(readByte_AbsoluteX());
					break;

				case 0b100:
					BCS(readByte_Immediate());
					break;

				default:
					assert(false && "unknown LDY addressing mode");
				}
				break;

			case 0b110:	//	CPY
				switch (addressing_mode) {
				case 0b000:
					CPY(readByte_Immediate());
					break;
				case 0b001:
					CPY(readByte_ZeroPage());
					break;
				case 0b011:
					CPY(readByte_Absolute());
					break;

				case 0b100:	// BNE
					BNE(readByte_Immediate());
					break;

				case 0b010:	// INY
					INY();
					break;

				default:
					assert(false && "unknown CPY addressing mode");
				}
				break;

			case 0b111:	//	CPX
				switch (addressing_mode) {
				case 0b000:
					CPX(readByte_Immediate());
					break;
				case 0b001:
					CPX(readByte_ZeroPage());
					break;
				case 0b011:
					CPX(readByte_Absolute());
					break;

				case 0b100:
					BEQ(readByte_Immediate());
					break;

				case 0b010:
					INX();
					break;

				default:
					assert(false && "unknown CPX addressing mode");
				}
				break;

			default:
				assert(false && "unknown opcode in 00 classification");
			}
			break;

		case 0b01:

			/*
			000	(zero page, X)
			001	zero page
			010	#immediate
			011	absolute
			100	(zero page), Y
			101	zero page, X
			110	absolute, Y
			111	absolute, X
			*/

			switch (op_code) {

			case 0b000:	//	ORA
				switch (addressing_mode) {
				case 0b000:
					ORA(readByte_ZeroPageX());
					break;
				case 0b001:
					ORA(readByte_ZeroPage());
					break;
				case 0b010:
					ORA(readByte_Immediate());
					break;
				case 0b011:
					ORA(readByte_Absolute());
					break;
				case 0b100:
					ORA(readByte_ZeroPageY());
					break;
				case 0b101:
					ORA(readByte_ZeroPageX());
					break;
				case 0b110:
					ORA(readByte_AbsoluteY());
					break;
				case 0b111:
					ORA(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown ORA addressing mode");
				}
				break;

			case 0b001:	//	AND
				switch (addressing_mode) {
				case 0b000:
					AND(readByte_ZeroPageX());
					break;
				case 0b001:
					AND(readByte_ZeroPage());
					break;
				case 0b010:
					AND(readByte_Immediate());
					break;
				case 0b011:
					AND(readByte_Absolute());
					break;
				case 0b100:
					AND(readByte_ZeroPageY());
					break;
				case 0b101:
					AND(readByte_ZeroPageX());
					break;
				case 0b110:
					AND(readByte_AbsoluteY());
					break;
				case 0b111:
					AND(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown AND addressing mode");
				}
				break;

			case 0b010:	//	EOR
				switch (addressing_mode) {
				case 0b000:
					EOR(readByte_ZeroPageX());
					break;
				case 0b001:
					EOR(readByte_ZeroPage());
					break;
				case 0b010:
					EOR(readByte_Immediate());
					break;
				case 0b011:
					EOR(readByte_Absolute());
					break;
				case 0b100:
					EOR(readByte_ZeroPageY());
					break;
				case 0b101:
					EOR(readByte_ZeroPageX());
					break;
				case 0b110:
					EOR(readByte_AbsoluteY());
					break;
				case 0b111:
					EOR(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown EOR addressing mode");
				}
				break;

			case 0b011:	//	ADC
				switch (addressing_mode) {
				case 0b000:
					ADC(readByte_ZeroPageX());
					break;
				case 0b001:
					ADC(readByte_ZeroPage());
					break;
				case 0b010:
					ADC(readByte_Immediate());
					break;
				case 0b011:
					ADC(readByte_Absolute());
					break;
				case 0b100:
					ADC(readByte_ZeroPageY());
					break;
				case 0b101:
					ADC(readByte_ZeroPageX());
					break;
				case 0b110:
					EOR(readByte_AbsoluteY());
					break;
				case 0b111:
					ADC(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown ADC addressing mode");
				}
				break;

			case 0b100:	//	STA
				switch (addressing_mode) {
				case 0b000:
					writeByte_ZeroPageX(A);
					break;
				case 0b001:
					writeByte_ZeroPage(A);
					break;
				case 0b011:
					writeByte_Absolute(A);
					break;
				case 0b100:
					writeByte_ZeroPageY(A);
					break;
				case 0b101:
					writeByte_ZeroPageX(A);
					break;
				case 0b110:
					writeByte_AbsoluteY(A);
					break;
				case 0b111:
					writeByte_AbsoluteX(A);
					break;
				default:
					assert(false && "unknown STA addressing mode");
				}
				break;

			case 0b101:	//	LDA
				switch (addressing_mode) {
				case 0b000:
					LDA(readByte_ZeroPageX());
					break;
				case 0b001:
					LDA(readByte_ZeroPage());
					break;
				case 0b010:
					LDA(readByte_Immediate());
					break;
				case 0b011:
					LDA(readByte_Absolute());
					break;
				case 0b100:
					LDA(readByte_ZeroPageY());
					break;
				case 0b101:
					LDA(readByte_ZeroPageX());
					break;
				case 0b110:
					LDA(readByte_AbsoluteY());
					break;
				case 0b111:
					LDA(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown LDA addressing mode");
				}
				break;

			case 0b110:	//	CMP
				switch (addressing_mode) {
				case 0b000:
					CMP(readByte_ZeroPageX());
					break;
				case 0b001:
					CMP(readByte_ZeroPage());
					break;
				case 0b010:
					CMP(readByte_Immediate());
					break;
				case 0b011:
					CMP(readByte_Absolute());
					break;
				case 0b100:
					CMP(readByte_ZeroPageY());
					break;
				case 0b101:
					CMP(readByte_ZeroPageX());
					break;
				case 0b110:
					CMP(readByte_AbsoluteY());
					break;
				case 0b111:
					CMP(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown CMP addressing mode");
				}
				break;

			case 0b111:	//	SBC
				switch (addressing_mode) {
				case 0b000:
					SBC(readByte_ZeroPageX());
					break;
				case 0b001:
					SBC(readByte_ZeroPage());
					break;
				case 0b010:
					SBC(readByte_Immediate());
					break;
				case 0b011:
					SBC(readByte_Absolute());
					break;
				case 0b100:
					SBC(readByte_ZeroPageY());
					break;
				case 0b101:
					SBC(readByte_ZeroPageX());
					break;
				case 0b110:
					SBC(readByte_AbsoluteY());
					break;
				case 0b111:
					SBC(readByte_AbsoluteX());
					break;
				default:
					assert(false && "unknown SBC addressing mode");
				}
				break;

			default:
				assert(false && "unknown opcode in 01 classification");
			}
			break;


		case 0b10:
			/*
			000	#immediate
			001	zero page
			010	accumulator
			011	absolute
			101	zero page,X
			111	absolute,X
			*/
			switch (op_code) {

			case 0b000:	//	ASL
				switch (addressing_mode)
				{
				case 0b010:	// ASL A
					A = ASL(A);
					break;
				default:
					assert(false && "unknown ASL addressing mode");
				}
				break;

			case 0b001:	//	ROL
				switch (addressing_mode)
				{
				case 0b010:	// ROL A
					A = ROL(A);
					break;

				case 0b111: // ROL abs,X
					{
						auto address = FETCH_ADDR_ABSOLUTEX;
						auto contents = memory[address];
						memory[address] = ROL(contents);
					}
					break;

				default:
					assert(false && "unknown ROL addressing mode");
				}
				break;

			case 0b010:	//	LSR
				assert(false && "LSR not implemented");
				break;

			case 0b011:	//	ROR
				assert(false && "ROR not implemented");
				break;

			case 0b100:	//	STX
				switch (addressing_mode) {
				case 0b001:
					writeByte_ZeroPage(X);
					break;
				case 0b011:
					writeByte_Absolute(X);
					break;
				case 0b101:
					writeByte_ZeroPageY(X);
					break;

				case 0b110:	// TXS
					TXS();
					break;

				default:
					assert(false && "unknown STX addressing mode");
				}
				break;

			case 0b101:	//	LDX
				switch (addressing_mode) {
				case 0b000:
					LDX(readByte_Immediate());
					break;
				case 0b001:
					LDX(readByte_ZeroPage());
					break;
				case 0b011:
					LDX(readByte_Absolute());
					break;
				case 0b110:
					LDX(readByte_ZeroPageY());
					break;
				case 0b111:
					LDX(readByte_AbsoluteY());
					break;
				default:
					assert(false && "unknown LDX addressing mode");
				}
				break;

			case 0b110:	//	DEC
				switch (addressing_mode) {
				case 0b001:
					DEC(FETCH_ADDR_ZEROPAGE);
					break;
				case 0b011:
					DEC(FETCH_ADDR_ABSOLUTE);
					break;
				case 0b101:
					DEC(FETCH_ADDR_ZEROPAGEX);
					break;
				case 0b111:
					DEC(FETCH_ADDR_ABSOLUTEX);
					break;

				case 0b010:	// DEX
					DEX();
					break;

				default:
					assert(false && "unknown DEC addressing mode");
				}
				break;

			case 0b111:	//	INC
				// zero page
				// zero page, x
				// absolute
				// absolute, x
				switch (addressing_mode) {
				case 0b001:
					INC(FETCH_ADDR_ZEROPAGE);
					break;
				case 0b011:
					INC(FETCH_ADDR_ABSOLUTE);
					break;
				case 0b101:
					INC(FETCH_ADDR_ZEROPAGEX);
					break;
				case 0b111:
					INC(FETCH_ADDR_ABSOLUTEX);
					break;
				}
				break;

			default:
				assert(false && "unknown opcode in 10 classification");
			}
			break;
		}
	}
}
