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
	PC = 0x0000;
	X = 0x80;
	Y = 0x00;
	A = 0x00;
	P = 0x36;
	S = 0xff;
}

//

uint8_t mos6502::getByte(uint16_t offset)
{
	return memory[offset];
}

void mos6502::setByte(uint16_t offset, uint8_t value)
{
	memory[offset] = value;
}

uint16_t mos6502::getWord(uint16_t offset)
{
	auto low = getByte(offset);
	auto high = getByte(offset + 1);
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
	cycles += 2;
	return value;
}

uint8_t mos6502::readByte_ZeroPage()
{
	auto zp = fetchByte();
	cycles += 3;
	return getByte(zp);
}

int8_t mos6502::readByte_ImmediateDisplacement()
{
	auto displacment = (int8_t)fetchByte();
	return displacment;
}

uint8_t mos6502::readByte_ZeroPageX()
{
	auto zp = fetchByte();
	cycles += 4;
	return getByte(lowByte(zp + X));
}

uint8_t mos6502::readByte_ZeroPageY()
{
	auto zp = fetchByte();
	return getByte(lowByte(zp + Y));
}

uint8_t mos6502::readByte_AbsoluteX()
{
	cycles += 4;
	auto base = fetchWord();
	uint16_t offset = base + X;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return getByte(offset);
}

uint8_t mos6502::readByte_AbsoluteY()
{
	cycles += 4;
	auto base = fetchWord();
	uint16_t offset = base + Y;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return getByte(offset);
}

uint8_t mos6502::readByte_Absolute()
{
	auto address = fetchWord();
	cycles += 4;
	return getByte(address);
}

uint8_t mos6502::readByte_IndexedIndirectX()
{
	cycles += 6;
	auto zp = fetchByte();
	return getByte(getWord(lowByte(zp + X)));
}

uint8_t mos6502::readByte_IndirectIndexedY()
{
	cycles += 5;
	auto zp = fetchByte();
	auto indirection = getWord(zp);
	if (lowByte(indirection) == 0xff)
		cycles += 1;
	return getByte(indirection + Y);
}

//

void mos6502::writeByte_ZeroPage(uint8_t value)
{
	auto zp = fetchByte();
	setByte(zp, value);
	cycles += 3;
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	auto address = fetchWord();
	setByte(address, value);
	cycles += 4;
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	auto zp = fetchByte();
	setByte(getWord(lowByte(zp + X)), value);
	cycles += 6;
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	auto zp = fetchByte();
	setByte(getWord(zp) + Y, value);
	cycles += 6;
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	auto zp = fetchByte();
	setByte(lowByte(zp + X), value);
	cycles += 4;
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	auto zp = fetchByte();
	setByte(lowByte(zp + Y), value);
	cycles += 4;
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	auto base = fetchWord();
	setByte((uint16_t)(base + X), value);
	cycles += 5;
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	auto base = fetchWord();
	setByte((uint16_t)(base + Y), value);
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
	if (P & F_D)
		ADC_d(data);
	else
		ADC_b(data);
}

void mos6502::ADC_b(uint8_t data)
{
	assert(!(P & F_D));

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

void mos6502::ADC_d(uint8_t data)
{
	assert(P & F_D);

	uint8_t carry = P & F_C ? 1 : 0;

	P &= ~(F_N | F_V | F_Z | F_C);

	uint8_t low = (A & 0x0f) + (data & 0x0f) + carry;
	if (low > 9)
		low += 6;

	uint8_t high = (A >> 4) + (data >> 4) + (low > 0x0f);

	if (!uint8_t(A + data + carry))
		P |= F_Z;
	else
		if (high & 8)
			P |= F_N;

	if (~(A^data) & (A ^ (high << 4)) & 0x80)
		P |= F_V;

	if (high > 9)
		high += 6;
	if (high > 0x0f)
		P |= F_C;

	A = (high << 4) | (low & 0x0f);
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
	if (P & F_D)
		SBC_d(data);
	else
		SBC_b(data);
}

void mos6502::SBC_b(uint8_t data)
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

void mos6502::SBC_d(uint8_t data)
{
	assert(P & F_D);	// BCD not implemented!!

	uint8_t carry = P & F_C ? 0 : 1;

	P &= ~(F_N | F_V | F_Z | F_C);

	uint16_t difference = A - data - carry;

	uint8_t low = (A & 0x0f) - (data & 0x0f) - carry;
	if (int8_t(low) < 0)
		low -= 6;

	uint8_t high = (A >> 4) - (data >> 4) - (int8_t(low) < 0);

	if (!uint8_t(difference))
		P |= F_Z;
	else
		if (difference & 0x80)
			P |= F_N;

	if ((A ^ data) & (A ^ difference) & 0x80)
		P |= F_V;

	if (!(difference & 0xff00))
		P |= F_C;

	if (int8_t(high) < 0)
		high -= 6;

	A = (high << 4) | (low & 0x0f);
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
	int8_t content = getByte(offset);
	setByte(offset, --content);
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
	int8_t content = getByte(offset);
	setByte(offset, ++content);
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
	pushStatus();
	cycles += 3;
}

void mos6502::pushStatus()
{
	P |= (F_B | F_reserved);
	pushByte(P);
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
		if ((int8_t)result < 0)
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
	auto contents = getByte(offset);
	setByte(offset, ASL(contents));
}

void mos6502::ROL(uint16_t offset)
{
	auto contents = getByte(offset);
	setByte(offset, ROL(contents));
}

void mos6502::LSR(uint16_t offset)
{
	auto contents = getByte(offset);
	setByte(offset, LSR(contents));
}

void mos6502::ROR(uint16_t offset)
{
	auto contents = getByte(offset);
	setByte(offset, ROR(contents));
}

void mos6502::BRK()
{
	pushWord(PC + 1);
	pushStatus();
	P |= F_I;
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
	auto current = fetchByte();

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
				BRK();
				break;

			case 0b100:
				BPL(readByte_ImmediateDisplacement());
				break;

			case 0b110:	 // CLC
				CLC();
				break;

			case 0b010:
				PHP();
				break;
			}
			break;

		case 0b001:	//	BIT
			switch (addressing_mode) {

			case 0b001:
				BIT(readByte_ZeroPage());
				cycles++;
				break;
			case 0b011:
				BIT(readByte_Absolute());
				cycles += 2;
				break;

			case 0b000: // JSR
				JSR();
				break;

			case 0b100:
				BMI(readByte_ImmediateDisplacement());
				break;

			case 0b110:
				SEC();
				break;

			case 0b010:
				PLP();
				break;
			}
			break;

		case 0b010:
			switch (addressing_mode)
			{

			case 0b010: // PHA
				PHA();
				break;

			case 0b011:
				{
					auto address = fetchWord();
					PC = address;
					cycles += 3;
				}
				break;

			case 0b100:
				BVC(readByte_ImmediateDisplacement());
				break;

			case 0b000:
				RTI();
				break;

			case 0b110:
				CLI();
				break;
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
				{
					auto address = fetchWord();
					PC = getWord(address);
					cycles += 5;
				}
				break;

			case 0b100:
				BVS(readByte_ImmediateDisplacement());
				break;

			case 0b110:
				SEI();
				break;
			}
			break;

		case 0b100:	//	DEY
			switch (addressing_mode)
			{

			case 0b010:	// Implied DEY
				DEY();
				break;

			case 0b100: // BCC rel
				BCC(readByte_ImmediateDisplacement());
				break;

			case 0b001: // STY zero page
				writeByte_ZeroPage(Y);
				break;
			case 0b011: // STY absolute
				writeByte_Absolute(Y);
				break;
			case 0b101: // STY zero page, X
				writeByte_ZeroPageX(Y);
				break;

			case 0b110: // TYA
				TYA();
				break;
			}
			break;

		case 0b101:
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
				BCS(readByte_ImmediateDisplacement());
				break;

			case 0b010: // TAY
				TAY();
				break;

			case 0b110:
				CLV();
				break;
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
				BNE(readByte_ImmediateDisplacement());
				break;

			case 0b010:	// INY
				INY();
				break;

			case 0b110:
				CLD();
				break;
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
				BEQ(readByte_ImmediateDisplacement());
				break;

			case 0b010:
				INX();
				break;

			case 0b110:
				SED();
				break;
			}
			break;
		}
		break;

	case 0b01:

		switch (op_code) {

		case 0b000:	//	ORA
			switch (addressing_mode) {

			case 0b000:
				ORA(readByte_IndexedIndirectX());
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
				ORA(readByte_IndirectIndexedY());
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
			}
			break;

		case 0b001:	//	AND
			switch (addressing_mode) {

			case 0b000:
				AND(readByte_IndexedIndirectX());
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
				AND(readByte_IndirectIndexedY());
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
			}
			break;

		case 0b010:	//	EOR
			switch (addressing_mode) {

			case 0b000:
				EOR(readByte_IndexedIndirectX());
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
				EOR(readByte_IndirectIndexedY());
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
			}
			break;

		case 0b011:	//	ADC
			switch (addressing_mode) {

			case 0b000:
				ADC(readByte_IndexedIndirectX());
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
				ADC(readByte_IndirectIndexedY());
				break;
			case 0b101:
				ADC(readByte_ZeroPageX());
				break;
			case 0b110:
				ADC(readByte_AbsoluteY());
				break;
			case 0b111:
				ADC(readByte_AbsoluteX());
				break;
			}
			break;

		case 0b100:	//	STA
			switch (addressing_mode) {

			case 0b000:
				writeByte_IndexedIndirectX(A);
				break;
			case 0b001:
				writeByte_ZeroPage(A);
				break;
			case 0b011:
				writeByte_Absolute(A);
				break;
			case 0b100:
				writeByte_IndirectIndexedY(A);
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
			}
			break;

		case 0b101:	//	LDA

			switch (addressing_mode) {
			case 0b000:
				LDA(readByte_IndexedIndirectX());
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
				LDA(readByte_IndirectIndexedY());
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
				CMP(readByte_IndexedIndirectX());
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
				CMP(readByte_IndirectIndexedY());
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
			}
			break;

		case 0b111:	//	SBC
			switch (addressing_mode) {

			case 0b000:
				SBC(readByte_IndexedIndirectX());
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
				SBC(readByte_IndirectIndexedY());
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
			}
			break;
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
			}
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

			case 0b010:	// TXA
				TXA();
				break;
			case 0b110:	// TXS
				TXS();
				break;
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
			case 0b101:
				LDX(readByte_ZeroPageY());
				break;
			case 0b111:
				LDX(readByte_AbsoluteY());
				break;

			case 0b010:
				TAX();
				break;
			case 0b110:
				TSX();
				break;
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

			case 0b010:	// NOP
				break;
			}
			break;
		}
		break;
	}
}

void mos6502::run()
{
#ifdef TEST_SUITE2
	uint16_t oldPC = (uint16_t)-1;
#endif

	cycles = 0;
	for (;;)
	{
#ifdef TEST_SUITE1
		if (PC == 0x45c0)
		{
			auto test = getByte(0x0210);
			if (test == 0xff)
				printf("\n** success!!");
			else
				printf("\n** failed!!");
			break;
		}
#endif

#ifdef TEST_SUITE2
		auto test = getByte(0x0200);
		if (oldPC == PC)
		{
			printf("\n** PC=%04x: test=%02x: stopped!!", PC, test);
			break;
		}
		else
		{
			oldPC = PC;
		}
#endif
		step();
	}
}
