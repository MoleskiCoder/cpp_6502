#include "stdafx.h"
#include "mos6502.h"

#include <fstream>
#include <iterator>
#include <algorithm>
#include <assert.h>

mos6502::mos6502()
:	memory(0x10000)
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

	std::copy(buffer.begin(), buffer.end(), memory.begin() + offset);
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
	auto value = fetchByte();
	DUMP_IMMEDIATE(value);
	cycles += 2;
	return value;
}

uint8_t mos6502::readByte_ZeroPage()
{
	auto value = fetchByte();
	DUMP_ZEROPAGE(value);
	cycles += 3;
	return memory[value];
}

int8_t mos6502::readByte_ImmediateDisplacement()
{
	auto displacment = (int8_t)fetchByte();
	DUMP_ABSOLUTE(PC + displacment);
	return displacment;
}

uint8_t mos6502::readByte_ZeroPageX()
{
	auto zp = fetchByte();
	DUMP_ZEROPAGEX(zp);
	cycles += 4;
	return memory[lowByte(zp + X)];
}

uint8_t mos6502::readByte_ZeroPageY()
{
	auto zp = fetchByte();
	DUMP_ZEROPAGEY(zp);
	return memory[lowByte(zp + Y)];
}

uint8_t mos6502::readByte_AbsoluteX()
{
	cycles += 4;
	auto base = fetchWord();
	DUMP_ABSOLUTEX(base);
	auto offset = base + X;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return memory[offset];
}

uint8_t mos6502::readByte_AbsoluteY()
{
	cycles += 4;
	auto base = fetchWord();
	DUMP_ABSOLUTEY(base);
	auto offset = base + Y;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return memory[offset];
}

uint8_t mos6502::readByte_Absolute()
{
	auto address = fetchWord();
	DUMP_ABSOLUTE(address);
	cycles += 4;
	return memory[address];
}

uint8_t mos6502::readByte_IndexedIndirectX()
{
	cycles += 6;
	auto zp = fetchByte();
	DUMP_INDEXEDINDIRECTX(zp);
	return memory[getWord(zp + X)];
}

uint8_t mos6502::readByte_IndirectIndexedY()
{
	cycles += 5;
	auto zp = fetchByte();
	DUMP_INDIRECTINDEXEDY(zp);
	auto indirection = getWord(zp);
	if (lowByte(indirection) == 0xff)
		cycles += 1;
	return memory[indirection + Y];
}

//

void mos6502::writeByte_ZeroPage(uint8_t value)
{
	auto zp = fetchByte();
	DUMP_ZEROPAGE(zp);
	memory[zp] = value;
	cycles += 3;
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	auto address = fetchWord();
	DUMP_ABSOLUTE(address);
	memory[address] = value;
	cycles += 4;
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	auto zp = fetchByte();
	DUMP_INDEXEDINDIRECTX(zp);
	memory[getWord(lowByte(zp + X))] = value;
	cycles += 6;
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	auto zp = fetchByte();
	DUMP_INDIRECTINDEXEDY(zp);
	memory[getWord(zp) + Y] = value;
	cycles += 6;
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	auto zp = fetchByte();
	DUMP_ZEROPAGEX(zp);
	memory[lowByte(zp + X)] = value;
	cycles += 4;
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	auto zp = fetchByte();
	DUMP_ZEROPAGEY(zp);
	memory[lowByte(zp + Y)] = value;
	cycles += 4;
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	auto base = fetchWord();
	DUMP_ABSOLUTEX(base);
	memory[(uint16_t)(base + X)] = value;
	cycles += 5;
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	auto base = fetchWord();
	DUMP_ABSOLUTEY(base);
	memory[(uint16_t)(base + Y)] = value;
	cycles += 5;
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
	P &= ~(F_N | F_Z);
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
	P &= ~(F_Z | F_V |F_N);

	uint8_t result = A & data;

	if (!result)
		P |= F_Z;

	if (data & 0x80)
		P |= F_N;
	if (data & 0x40)
		P |= F_V;

	cycles += 2;
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
	assert(!(P & F_D));	// BCD not implemented!!

	uint8_t carry = (P & F_C ? 1 : 0);
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

	A = (uint8_t)sum;
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
	assert(!(P & F_D));	// BCD not implemented!!

	uint8_t carry = (P & F_C ? 0 : 1);
	uint16_t difference = A - data - carry;

	P &= ~(F_Z | F_V | F_N | F_C);

	if (!(uint8_t)difference)
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

//

void mos6502::BCS(int8_t data)	{	cycles += 2;	if (P & F_C)	branch(data);	}
void mos6502::BCC(int8_t data)	{	cycles += 2;	if (!(P & F_C))	branch(data);	}

void mos6502::BMI(int8_t data)	{	cycles += 2;	if (P & F_N)	branch(data);	}
void mos6502::BPL(int8_t data)	{	cycles += 2;	if (!(P & F_N))	branch(data);	}

void mos6502::BEQ(int8_t data)	{	cycles += 2;	if (P & F_Z)	branch(data);	}
void mos6502::BNE(int8_t data)	{	cycles += 2;	if (!(P & F_Z))	branch(data);	}

void mos6502::BVS(int8_t data)	{	cycles += 2;	if (P & F_V)	branch(data);	}
void mos6502::BVC(int8_t data)	{	cycles += 2;	if (!(P & F_V))	branch(data);	}

void mos6502::branch(int8_t displacement)
{
	++cycles;
	auto oldPage = highByte(PC);
	PC += displacement;
	auto newPage = highByte(PC);
	if (oldPage != newPage)
		cycles += 2;
}

void mos6502::SED() { cycles += 2;	P |= F_D;	}
void mos6502::CLD() { cycles += 2;	P &= ~F_D;	}

void mos6502::SEI() { cycles += 2;	P |= F_I;	}
void mos6502::CLI() { cycles += 2;	P &= ~F_I;	}

void mos6502::CLV() { cycles += 2;	P &= ~F_V;	}

void mos6502::SEC() { cycles += 2;	P |= F_C;	}
void mos6502::CLC() { cycles += 2;	P &= ~F_C;	}

//

void mos6502::DEC(uint16_t offset)
{
	int8_t content = memory[offset];
	memory[offset] = --content;
	reflectFlags_ZeroNegative(content);
}

void mos6502::DEX()
{
	reflectFlags_ZeroNegative(--X);
	cycles += 2;
}

void mos6502::DEY()
{
	reflectFlags_ZeroNegative(--Y);
	cycles += 2;
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
	cycles += 2;
}

void mos6502::INY()
{
	reflectFlags_ZeroNegative(++Y);
	cycles += 2;
}

void mos6502::JSR()
{
	auto destination = fetchWord();
	DUMP_ABSOLUTE(destination);
	pushWord(PC - 1);
	PC = destination;
	cycles += 6;
}

void mos6502::RTS()
{
	PC = popWord() + 1;
	cycles += 6;
}

void mos6502::PHA()
{
	pushByte(A);
	cycles += 3;
}

void mos6502::PLA()
{
	A = popByte();
	reflectFlags_ZeroNegative(A);
	cycles += 4;
}

void mos6502::PHP()
{
	pushByte(P);
	cycles + 3;
}

void mos6502::PLP()
{
	P = popByte();
	cycles += 4;
}

void mos6502::TXS()
{
	S = X;
	cycles += 2;
}

void mos6502::TSX()
{
	X = S;
	reflectFlags_ZeroNegative(X);
	cycles += 2;
}

void mos6502::TAX()
{
	X = A;
	reflectFlags_ZeroNegative(X);
	cycles += 2;
}

void mos6502::TXA()
{
	A = X;
	reflectFlags_ZeroNegative(A);
	cycles += 2;
}

void mos6502::TAY()
{
	Y = A;
	reflectFlags_ZeroNegative(Y);
	cycles += 2;
}

void mos6502::TYA()
{
	A = Y;
	reflectFlags_ZeroNegative(A);
	cycles += 2;
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

uint8_t mos6502::LSR(uint8_t data)
{
	P &= ~(F_N | F_Z | F_C);

	if (data & 1)
		P |= F_C;

	uint8_t result = data >> 1;

	if (!result)
		P |= F_Z;

	return result;
}

uint8_t mos6502::ROR(uint8_t data)
{
	bool carry = P & F_C;
	P &= ~(F_N | F_Z | F_C);

	if (data & 1)
		P |= F_C;

	uint8_t result = data >> 1;
	if (carry)
		result |= 0x80;

	if (!result)
		P |= F_Z;
	else
		if ((int8_t)result < 0)
			P |= F_N;

	return result;
}

void mos6502::ASL(uint16_t offset)
{
	auto contents = memory[offset];
	memory[offset] = ASL(contents);
}

void mos6502::ROL(uint16_t offset)
{
	auto contents = memory[offset];
	memory[offset] = ROL(contents);
}

void mos6502::LSR(uint16_t offset)
{
	auto contents = memory[offset];
	memory[offset] = LSR(contents);
}

void mos6502::ROR(uint16_t offset)
{
	auto contents = memory[offset];
	memory[offset] = ROR(contents);
}

void mos6502::BRK()
{
	pushWord(PC + 2);
	P |= F_B;
	pushByte(P);
	PC = getWord(0xfffe);
	cycles += 7;
}

void mos6502::RTI()
{
	P = popByte();
	PC = popWord();
	cycles += 6;
}

//

void mos6502::step()
{
#ifdef _DEBUG
	printf("\n");
	printf("PC=%04x:", PC);
	printf("P=%02x, A=%02x, X=%02x, Y=%02x, S=%02x	", P, A, X, Y, S);
#endif

	auto current = fetchByte();
	DUMP_BYTEVALUE(current);

#ifdef _DEBUG
	instructionCounts[current]++;
#endif

	auto class_mask = 0b11;
	auto addressing_mode_mask = 0b11100;
	auto op_code_mask = 0b11100000;

	auto classification = current & class_mask;
	auto addressing_mode = (current & addressing_mode_mask) >> 2;
	auto op_code = (current & op_code_mask) >> 5;

	switch (classification)
	{
	case 0b00:

		switch (op_code) {

		case 0b000:	// BPL
			switch (addressing_mode) {

			case 0b00:
				DUMP_PREFIX(BRK);
				BRK();
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BPL);
				BPL(readByte_ImmediateDisplacement());
				break;

			case 0b110:	 // CLC
				DUMP_PREFIX(CLC);
				CLC();
				break;

			case 0b010:
				DUMP_PREFIX(PHP);
				PHP();
				break;

			default:
				assert(false && "unknown BPL/CLC addressing mode");
			}
			break;

		case 0b001:	//	BIT
			switch (addressing_mode) {

			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BIT);
				BIT(readByte_ZeroPage());
				cycles++;
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(BIT);
				BIT(readByte_Absolute());
				cycles += 2;
				break;

			case 0b000: // JSR
				DUMP_DBYTE(PC);
				DUMP_PREFIX(JSR);
				JSR();
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BMI);
				BMI(readByte_ImmediateDisplacement());
				break;

			case 0b110:
				DUMP_PREFIX(SEC);
				SEC();
				break;

			case 0b010:
				DUMP_PREFIX(PLP);
				PLP();
				break;

			default:
				assert(false && "unknown BIT/JSR addressing mode");
			}
			break;

		case 0b010:
			switch (addressing_mode)
			{

			case 0b010: // PHA
				DUMP_PREFIX(PHA);
				PHA();
				break;

			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(JMP);
				{
					auto address = fetchWord();
					DUMP_ABSOLUTE(address);
					PC = address;
					cycles += 3;
				}
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BVC);
				BVC(readByte_ImmediateDisplacement());
				break;

			case 0b000:
				DUMP_PREFIX(RTI);
				RTI();
				break;

			case 0b110:
				DUMP_PREFIX(CLI);
				CLI();
				break;

			default:
				assert(false && "unknown PHA/JMP instruction");
			}
			break;

		case 0b011:
			switch (addressing_mode)
			{

			case 0b000: // RTS
				DUMP_PREFIX(RTS);
				RTS();
				break;

			case 0b010:	// PLA
				DUMP_PREFIX(PLA);
				PLA();
				break;

			case 0b011: // JMP (indirect)
				DUMP_DBYTE(PC);
				DUMP_PREFIX(JMP);
				{
					auto address = fetchWord();
					DUMP_INDIRECT(address);
					PC = getWord(address);
					cycles += 5;
				}
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BVS);
				BVS(readByte_ImmediateDisplacement());
				break;

			case 0b110:
				DUMP_PREFIX(SEI);
				SEI();
				break;

			default:
				assert(false && "unknown RTS/PLA/JMP addressing_mode");
			}
			break;

		case 0b100:	//	DEY
			switch (addressing_mode)
			{

			case 0b010:	// Implied DEY
				DUMP_PREFIX(DEY);
				DEY();
				break;

			case 0b100: // BCC rel
				DUMP_BYTE(PC);
				DUMP_PREFIX(BCC);
				BCC(readByte_ImmediateDisplacement());
				break;

			case 0b001: // STY zero page
				DUMP_BYTE(PC);
				DUMP_PREFIX(STY);
				writeByte_ZeroPage(Y);
				break;
			case 0b011: // STY absolute
				DUMP_DBYTE(PC);
				DUMP_PREFIX(STY);
				writeByte_Absolute(Y);
				break;
			case 0b101: // STY zero page, X
				DUMP_BYTE(PC);
				DUMP_PREFIX(STY);
				writeByte_ZeroPageX(Y);
				break;

			case 0b110: // TYA
				DUMP_PREFIX(TYA);
				TYA();
				break;

			default:
				assert(false && "unknown DEY/BCC/STY/TYA addressing mode");
			}
			break;

		case 0b101:
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDY);
				LDY(readByte_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDY);
				LDY(readByte_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDY);
				LDY(readByte_Absolute());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDY);
				LDY(readByte_ZeroPageX());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDY);
				LDY(readByte_AbsoluteX());
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BCS);
				BCS(readByte_ImmediateDisplacement());
				break;

			case 0b010: // TAY
				DUMP_PREFIX(TAY);
				TAY();
				break;

			case 0b110:
				DUMP_PREFIX(CLV);
				CLV();
				break;

			default:
				assert(false && "unknown LDY/BCS/TAY addressing mode");
			}
			break;

		case 0b110:	//	CPY
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CPY);
				CPY(readByte_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CPY);
				CPY(readByte_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(CPY);
				CPY(readByte_Absolute());
				break;

			case 0b100:	// BNE
				DUMP_BYTE(PC);
				DUMP_PREFIX(BNE);
				BNE(readByte_ImmediateDisplacement());
				break;

			case 0b010:	// INY
				DUMP_PREFIX(INY);
				INY();
				break;

			case 0b110:
				DUMP_PREFIX(CLD);
				CLD();
				break;

			default:
				assert(false && "unknown CPY/BNE/INY addressing mode");
			}
			break;

		case 0b111:	//	CPX
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CPX);
				CPX(readByte_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CPX);
				CPX(readByte_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(CPX);
				CPX(readByte_Absolute());
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(BEQ);
				BEQ(readByte_ImmediateDisplacement());
				break;

			case 0b010:
				DUMP_PREFIX(INX);
				INX();
				break;

			case 0b110:
				DUMP_PREFIX(SED);
				SED();
				break;

			default:
				assert(false && "unknown CPX/BEQ/INX addressing mode");
			}
			break;

		default:
			assert(false && "unknown opcode in 00 classification");
		}
		break;

	case 0b01:

		switch (op_code) {

		case 0b000:	//	ORA
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ORA);
				ORA(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown ORA addressing mode");
			}
			break;

		case 0b001:	//	AND
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(AND);
				AND(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown AND addressing mode");
			}
			break;

		case 0b010:	//	EOR
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(EOR);
				EOR(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown EOR addressing mode");
			}
			break;

		case 0b011:	//	ADC
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(ADC);
				ADC(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown ADC addressing mode");
			}
			break;

		case 0b100:	//	STA
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_IndexedIndirectX(A);
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_ZeroPage(A);
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_Absolute(A);
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_IndirectIndexedY(A);
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_ZeroPageX(A);
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_AbsoluteY(A);
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(STA);
				writeByte_AbsoluteX(A);
				break;

			default:
				assert(false && "unknown STA addressing mode");
			}
			break;

		case 0b101:	//	LDA

			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDA);
				LDA(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown LDA addressing mode");
			}
			break;

		case 0b110:	//	CMP
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(CMP);
				CMP(readByte_AbsoluteX());
				break;

			default:
				assert(false && "unknown CMP addressing mode");
			}
			break;

		case 0b111:	//	SBC
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(SBC);
				SBC(readByte_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(SBC);
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

		switch (op_code) {

		case 0b000:	//	ASL
			switch (addressing_mode)
			{

			case 0b001:	// ASL zp
				ACTION_ZP(ASL);
				cycles += 5;
				break;
			case 0b010:	// ASL A
				ACTION_A(ASL);
				break;
			case 0b011:	// ASL absolute
				ACTION_ABSOLUTE(ASL);
				break;
			case 0b101:	// ASL zp,X
				ACTION_ZEROPAGEX(ASL);
				break;
			case 0b111:	// ASL absolute,X
				ACTION_ABSOLUTEX(ASL);
				break;

			default:
				assert(false && "unknown ASL addressing mode");
			}
			break;

		case 0b001:	//	ROL
			switch (addressing_mode)
			{

			case 0b001:	// ROL ZP
				ACTION_ZP(ROL);
				cycles += 5;
				break;
			case 0b010:	// ROL A
				ACTION_A(ROL);
				break;
			case 0b011:	// ROL absolute
				ACTION_ABSOLUTE(ROL);
				break;
			case 0b101:	// ROL zp,x
				ACTION_ZEROPAGEX(ROL);
				break;
			case 0b111:	// ROL absolute,x
				ACTION_ABSOLUTEX(ROL);
				break;

			default:
				assert(false && "unknown ROL addressing mode");
			}
			break;

		case 0b010:	//	LSR
			switch (addressing_mode)
			{

			case 0b001:	// LSR ZP
				ACTION_ZP(LSR);
				cycles += 5;
				break;
			case 0b010:	// LSR A
				ACTION_A(LSR);
				break;
			case 0b011:	// LSR absolute
				ACTION_ABSOLUTE(LSR);
				break;
			case 0b101:	// LSR zp,x
				ACTION_ZEROPAGEX(LSR);
				break;
			case 0b111:	// LSR absolute,x
				ACTION_ABSOLUTEX(LSR);
				break;

			default:
				assert(false && "unknown LSR addressing mode");
			}
			break;

		case 0b011:	//	ROR
			switch (addressing_mode)
			{

			case 0b001:	// ROR ZP
				ACTION_ZP(ROR);
				cycles += 5;
				break;
			case 0b010:	// ROR A
				ACTION_A(ROR);
				break;
			case 0b011:	// ROR absolute
				ACTION_ABSOLUTE(ROR);
				break;
			case 0b101:	// ROR zp,x
				ACTION_ZEROPAGEX(ROR);
				break;
			case 0b111:	// ROR absolute,x
				ACTION_ABSOLUTEX(ROR);
				break;

			default:
				assert(false && "unknown ROR addressing mode");
			}
			break;

		case 0b100:	//	STX
			switch (addressing_mode) {

			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STX);
				writeByte_ZeroPage(X);
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(STX);
				writeByte_Absolute(X);
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(STX);
				writeByte_ZeroPageY(X);
				break;

			case 0b010:	// TXA
				DUMP_PREFIX(TXA);
				TXA();
				break;
			case 0b110:	// TXS
				DUMP_PREFIX(TXS);
				TXS();
				break;
			default:
				assert(false && "unknown STX/TXA/TXS addressing mode");
			}
			break;

		case 0b101:	//	LDX
			switch (addressing_mode) {

			case 0b000:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDX);
				LDX(readByte_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDX);
				LDX(readByte_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDX);
				LDX(readByte_Absolute());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DUMP_PREFIX(LDX);
				LDX(readByte_ZeroPageY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DUMP_PREFIX(LDX);
				LDX(readByte_AbsoluteY());
				break;

			case 0b010:
				DUMP_PREFIX(TAX);
				TAX();
				break;
			case 0b110:
				DUMP_PREFIX(TSX);
				TSX();
				break;

			default:
				assert(false && "unknown LDX/TAX/TSX addressing mode");
			}
			break;

		case 0b110:	//	DEC
			switch (addressing_mode) {

			case 0b001:
				ACTION_ZP(DEC);
				break;
			case 0b011:
				ACTION_ABSOLUTE(DEC);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(DEC);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(DEC);
				break;
			case 0b010:	// DEX
				ACTION_IMPLIED(DEX);
				break;
			default:
				assert(false && "unknown DEC/DEX addressing mode");
			}
			break;

		case 0b111:	//	INC
			switch (addressing_mode) {

			case 0b001:
				ACTION_ZP(INC);
				break;
			case 0b011:
				ACTION_ABSOLUTE(INC);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(INC);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(INC);
				break;

			case 0b010:
				DUMP_PREFIX(NOP);
				break;

			default:
				assert(false && "unknown INC addressing mode");
			}
			break;
		default:
			assert(false && "unknown opcode in 10 classification");
		}
		break;

	default:
		assert(false && "unknown classification");
	}
}

void mos6502::run()
{
	cycles = 0;
	for (;;)
	{
		auto result = memory[0x0210];
		step();
	}
}
