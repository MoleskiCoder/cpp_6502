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

void mos6502::BIT_zp()
{
	BIT(readByte_ZeroPage());
	cycles++;
}

void mos6502::BIT_abs()
{
	BIT(readByte_Absolute());
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
	assert(!(P & F_D));

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
	assert(P & F_D);

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

#define BRANCH_DEFINITION_S(ins, flag) \
	void mos6502:: ins (int8_t displacement) \
	{ \
		cycles += 2; \
		if (P & flag) \
			branch(displacement); \
	}

#define BRANCH_DEFINITION_C(ins, flag) \
	void mos6502:: ins (int8_t displacement) \
	{ \
		cycles += 2; \
		if ((!(P & flag))) \
			branch(displacement); \
	}

#define BRANCH_REL_DEFINITION(ins) \
	void mos6502:: ins ## _rel() \
	{ \
		ins(readByte_ImmediateDisplacement()); \
	}

#define BRANCH_DEFINITION(ins_s, ins_c, flag) \
	BRANCH_DEFINITION_S(ins_s, flag) \
	BRANCH_DEFINITION_C(ins_c, flag) \
	BRANCH_REL_DEFINITION(ins_s) \
	BRANCH_REL_DEFINITION(ins_c)

BRANCH_DEFINITION(BCS, BCC, F_C)
BRANCH_DEFINITION(BMI, BPL, F_N)
BRANCH_DEFINITION(BEQ, BNE, F_Z)
BRANCH_DEFINITION(BVS, BVC, F_V)

void mos6502::branch(int8_t displacement)
{
	++cycles;
	auto oldPage = highByte(PC);
	PC += displacement;
	auto newPage = highByte(PC);
	if (oldPage != newPage)
		cycles += 2;
}

void mos6502::SED_imp() { cycles += 2;	P |= F_D;	}
void mos6502::CLD_imp() { cycles += 2;	P &= ~F_D;	}

void mos6502::SEI_imp() { cycles += 2;	P |= F_I;	}
void mos6502::CLI_imp() { cycles += 2;	P &= ~F_I;	}

void mos6502::CLV_imp() { cycles += 2;	P &= ~F_V;	}

void mos6502::SEC_imp() { cycles += 2;	P |= F_C;	}
void mos6502::CLC_imp() { cycles += 2;	P &= ~F_C;	}

//

void mos6502::DEC(uint16_t offset)
{
	int8_t content = getByte(offset);
	setByte(offset, --content);
	reflectFlags_ZeroNegative(content);
}

void mos6502::DEX_imp()
{
	reflectFlags_ZeroNegative(--X);
	cycles += 2;
}

void mos6502::DEY_imp()
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

void mos6502::INX_imp()
{
	reflectFlags_ZeroNegative(++X);
	cycles += 2;
}

void mos6502::INY_imp()
{
	reflectFlags_ZeroNegative(++Y);
	cycles += 2;
}

void mos6502::JSR_abs()
{
	auto destination = fetchWord();
	pushWord(PC - 1);
	PC = destination;
	cycles += 6;
}

void mos6502::RTS_imp()
{
	PC = popWord() + 1;
	cycles += 6;
}

void mos6502::PHA_imp()
{
	pushByte(A);
	cycles += 3;
}

void mos6502::PLA_imp()
{
	A = popByte();
	reflectFlags_ZeroNegative(A);
	cycles += 4;
}

void mos6502::PHP_imp()
{
	pushStatus();
	cycles += 3;
}

void mos6502::pushStatus()
{
	P |= (F_B | F_reserved);
	pushByte(P);
}

void mos6502::PLP_imp()
{
	P = popByte();
	cycles += 4;
}

void mos6502::TXS_imp()
{
	S = X;
	cycles += 2;
}

void mos6502::TSX_imp()
{
	X = S;
	reflectFlags_ZeroNegative(X);
	cycles += 2;
}

void mos6502::TAX_imp()
{
	X = A;
	reflectFlags_ZeroNegative(X);
	cycles += 2;
}

void mos6502::TXA_imp()
{
	A = X;
	reflectFlags_ZeroNegative(A);
	cycles += 2;
}

void mos6502::TAY_imp()
{
	Y = A;
	reflectFlags_ZeroNegative(Y);
	cycles += 2;
}

void mos6502::TYA_imp()
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

void mos6502::BRK_imp()
{
	pushWord(PC + 1);
	pushStatus();
	P |= F_I;
	PC = getWord(0xfffe);
	cycles += 7;
}

void mos6502::RTI_imp()
{
	P = popByte();
	PC = popWord();
	cycles += 6;
}

void mos6502::JMP_abs()
{
	auto address = fetchWord();
	PC = address;
	cycles += 3;
}

void mos6502::JMP_ind()
{
	auto address = fetchWord();
	PC = getWord(address);
	cycles += 5;
}

//

#define READER_XIND_DEFINITION(x) \
	void mos6502::x ## _xind()	{ x(readByte_IndexedIndirectX());	}

#define READER_ZP_DEFINITION(x) \
	void mos6502::x ## _zp()	{ x(readByte_ZeroPage());			}

#define READER_IMM_DEFINITION(x) \
	void mos6502::x ## _imm()	{ x(readByte_Immediate());			}

#define READER_ABS_DEFINITION(x) \
	void mos6502::x ## _abs()	{ x(readByte_Absolute());			}

#define READER_INDY_DEFINITION(x) \
	void mos6502::x ## _indy()	{ x(readByte_IndirectIndexedY());	}

#define READER_ZPX_DEFINITION(x) \
	void mos6502::x ## _zpx()	{ x(readByte_ZeroPageX());			}

#define READER_ZPY_DEFINITION(x) \
	void mos6502::x ## _zpy()	{ x(readByte_ZeroPageY());			}

#define READER_ABSY_DEFINITION(x) \
	void mos6502::x ## _absy()	{ x(readByte_AbsoluteY());			}

#define READER_ABSX_DEFINITION(x) \
	void mos6502::x ## _absx()	{ x(readByte_AbsoluteX());			}

#define READER_GROUP_A_DEFINITIONS(x) \
	READER_XIND_DEFINITION(x) \
	READER_ZP_DEFINITION(x) \
	READER_IMM_DEFINITION(x) \
	READER_ABS_DEFINITION(x) \
	READER_INDY_DEFINITION(x) \
	READER_ZPX_DEFINITION(x) \
	READER_ABSY_DEFINITION(x) \
	READER_ABSX_DEFINITION(x)

READER_GROUP_A_DEFINITIONS(ORA)
READER_GROUP_A_DEFINITIONS(AND)
READER_GROUP_A_DEFINITIONS(EOR)
READER_GROUP_A_DEFINITIONS(ADC)
READER_GROUP_A_DEFINITIONS(LDA)
READER_GROUP_A_DEFINITIONS(CMP)
READER_GROUP_A_DEFINITIONS(SBC)

#define READER_GROUP_X_DEFINITIONS(x) \
	READER_IMM_DEFINITION(x) \
	READER_ZP_DEFINITION(x) \
	READER_ABS_DEFINITION(x) \
	READER_ZPY_DEFINITION(x) \
	READER_ABSY_DEFINITION(x)

READER_GROUP_X_DEFINITIONS(LDX)

#define READER_GROUP_Y_DEFINITIONS(x) \
	READER_IMM_DEFINITION(x) \
	READER_ZP_DEFINITION(x) \
	READER_ABS_DEFINITION(x) \
	READER_ZPX_DEFINITION(x) \
	READER_ABSX_DEFINITION(x)

READER_GROUP_Y_DEFINITIONS(LDY)

#define READER_GROUP_CPXY_DEFINITIONS(x) \
	READER_IMM_DEFINITION(x) \
	READER_ZP_DEFINITION(x) \
	READER_ABS_DEFINITION(x)

READER_GROUP_CPXY_DEFINITIONS(CPX)
READER_GROUP_CPXY_DEFINITIONS(CPY)

//

#define WRITER_XIND_DEFINITION(x, y) \
	void mos6502::x ## _xind()	{ writeByte_IndexedIndirectX(y);	}

#define WRITER_ZP_DEFINITION(x, y) \
	void mos6502::x ## _zp()	{ writeByte_ZeroPage(y);			}

#define WRITER_ABS_DEFINITION(x, y) \
	void mos6502::x ## _abs()	{ writeByte_Absolute(y);			}

#define WRITER_INDY_DEFINITION(x, y) \
	void mos6502::STA_indy()	{ writeByte_IndirectIndexedY(y);	}

#define WRITER_ZPX_DEFINITION(x, y) \
	void mos6502::x ## _zpx()	{ writeByte_ZeroPageX(y);			}

#define WRITER_ZPY_DEFINITION(x, y) \
	void mos6502::x ## _zpy()	{ writeByte_ZeroPageY(y);			}

#define WRITER_ABSY_DEFINITION(x, y) \
	void mos6502::x ## _absy()	{ writeByte_AbsoluteY(y);			}

#define WRITER_ABSX_DEFINITION(x, y) \
	void mos6502::x ## _absx()	{ writeByte_AbsoluteX(y);			}

#define WRITER_GROUP_A_DEFINITIONS(x, y) \
	WRITER_XIND_DEFINITION(x, y) \
	WRITER_ZP_DEFINITION(x, y) \
	WRITER_ABS_DEFINITION(x, y) \
	WRITER_INDY_DEFINITION(x, y) \
	WRITER_ZPX_DEFINITION(x, y) \
	WRITER_ABSY_DEFINITION(x, y) \
	WRITER_ABSX_DEFINITION(x, y)

WRITER_GROUP_A_DEFINITIONS(STA, A)

#define WRITER_GROUP_X_DEFINITIONS(x, y) \
	WRITER_ZP_DEFINITION(x, y) \
	WRITER_ABS_DEFINITION(x, y) \
	WRITER_ZPY_DEFINITION(x, y)

WRITER_GROUP_X_DEFINITIONS(STX, X)

#define WRITER_GROUP_Y_DEFINITIONS(x, y) \
	WRITER_ZP_DEFINITION(x, y) \
	WRITER_ABS_DEFINITION(x, y) \
	WRITER_ZPX_DEFINITION(x, y)

WRITER_GROUP_Y_DEFINITIONS(STY, Y)

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

		case 0b000:
			switch (addressing_mode) {
			case 0b00:
				BRK_imp();
				break;
			case 0b100:
				BPL_rel();
				break;
			case 0b110:
				CLC_imp();
				break;
			case 0b010:
				PHP_imp();
				break;
			}
			break;
		case 0b001:
			switch (addressing_mode) {
			case 0b001:
				BIT_zp();
				break;
			case 0b011:
				BIT_abs();
				break;
			case 0b000:
				JSR_abs();
				break;
			case 0b100:
				BMI_rel();
				break;
			case 0b110:
				SEC_imp();
				break;
			case 0b010:
				PLP_imp();
				break;
			}
			break;
		case 0b010:
			switch (addressing_mode)
			{
			case 0b010:
				PHA_imp();
				break;
			case 0b011:
				JMP_abs();
				break;
			case 0b100:
				BVC_rel();
				break;
			case 0b000:
				RTI_imp();
				break;
			case 0b110:
				CLI_imp();
				break;
			}
			break;
		case 0b011:
			switch (addressing_mode)
			{
			case 0b000:
				RTS_imp();
				break;
			case 0b010:
				PLA_imp();
				break;
			case 0b011:
				JMP_ind();
				break;
			case 0b100:
				BVS_rel();
				break;
			case 0b110:
				SEI_imp();
				break;
			}
			break;
		case 0b100:
			switch (addressing_mode)
			{
			case 0b010:
				DEY_imp();
				break;
			case 0b100:
				BCC_rel();
				break;
			case 0b001:
				STY_zp();
				break;
			case 0b011:
				STY_abs();
				break;
			case 0b101:
				STY_zpx();
				break;
			case 0b110:
				TYA_imp();
				break;
			}
			break;
		case 0b101:
			switch (addressing_mode) {
			case 0b000:
				LDY_imm();
				break;
			case 0b001:
				LDY_zp();
				break;
			case 0b011:
				LDY_abs();
				break;
			case 0b101:
				LDY_zpx();
				break;
			case 0b111:
				LDY_absx();
				break;
			case 0b100:
				BCS_rel();
				break;
			case 0b010:
				TAY_imp();
				break;
			case 0b110:
				CLV_imp();
				break;
			}
			break;
		case 0b110:
			switch (addressing_mode) {
			case 0b000:
				CPY_imm();
				break;
			case 0b001:
				CPY_zp();
				break;
			case 0b011:
				CPY_abs();
				break;
			case 0b100:
				BNE_rel();
				break;
			case 0b010:
				INY_imp();
				break;
			case 0b110:
				CLD_imp();
				break;
			}
			break;
		case 0b111:
			switch (addressing_mode) {
			case 0b000:
				CPX_imm();
				break;
			case 0b001:
				CPX_zp();
				break;
			case 0b011:
				CPX_abs();
				break;
			case 0b100:
				BEQ_rel();
				break;
			case 0b010:
				INX_imp();
				break;
			case 0b110:
				SED_imp();
				break;
			}
			break;
		}
		break;

	case 0b01:

		switch (op_code) {
		case 0b000:
			switch (addressing_mode) {
			case 0b000:
				ORA_xind();
				break;
			case 0b001:
				ORA_zp();
				break;
			case 0b010:
				ORA_imm();
				break;
			case 0b011:
				ORA_abs();
				break;
			case 0b100:
				ORA_indy();
				break;
			case 0b101:
				ORA_zpx();
				break;
			case 0b110:
				ORA_absy();
				break;
			case 0b111:
				ORA_absx();
				break;
			}
			break;
		case 0b001:
			switch (addressing_mode) {
			case 0b000:
				AND_xind();
				break;
			case 0b001:
				AND_zp();
				break;
			case 0b010:
				AND_imm();
				break;
			case 0b011:
				AND_abs();
				break;
			case 0b100:
				AND_indy();
				break;
			case 0b101:
				AND_zpx();
				break;
			case 0b110:
				AND_absy();
				break;
			case 0b111:
				AND_absx();
				break;
			}
			break;
		case 0b010:
			switch (addressing_mode) {
			case 0b000:
				EOR_xind();
				break;
			case 0b001:
				EOR_zp();
				break;
			case 0b010:
				EOR_imm();
				break;
			case 0b011:
				EOR_abs();
				break;
			case 0b100:
				EOR_indy();
				break;
			case 0b101:
				EOR_zpx();
				break;
			case 0b110:
				EOR_absy();
				break;
			case 0b111:
				EOR_absx();
				break;
			}
			break;
		case 0b011:
			switch (addressing_mode) {
			case 0b000:
				ADC_xind();
				break;
			case 0b001:
				ADC_zp();
				break;
			case 0b010:
				ADC_imm();
				break;
			case 0b011:
				ADC_abs();
				break;
			case 0b100:
				ADC_indy();
				break;
			case 0b101:
				ADC_zpx();
				break;
			case 0b110:
				ADC_absy();
				break;
			case 0b111:
				ADC_absx();
				break;
			}
			break;
		case 0b100:
			switch (addressing_mode) {
			case 0b000:
				STA_xind();
				break;
			case 0b001:
				STA_zp();
				break;
			case 0b011:
				STA_abs();
				break;
			case 0b100:
				STA_indy();
				break;
			case 0b101:
				STA_zpx();
				break;
			case 0b110:
				STA_absy();
				break;
			case 0b111:
				STA_absx();
				break;
			}
			break;
		case 0b101:
			switch (addressing_mode) {
			case 0b000:
				LDA_xind();
				break;
			case 0b001:
				LDA_zp();
				break;
			case 0b010:
				LDA_imm();
				break;
			case 0b011:
				LDA_abs();
				break;
			case 0b100:
				LDA_indy();
				break;
			case 0b101:
				LDA_zpx();
				break;
			case 0b110:
				LDA_absy();
				break;
			case 0b111:
				LDA_absx();
				break;
			}
			break;
		case 0b110:
			switch (addressing_mode) {
			case 0b000:
				CMP_xind();
				break;
			case 0b001:
				CMP_zp();
				break;
			case 0b010:
				CMP_imm();
				break;
			case 0b011:
				CMP_abs();
				break;
			case 0b100:
				CMP_indy();
				break;
			case 0b101:
				CMP_zpx();
				break;
			case 0b110:
				CMP_absy();
				break;
			case 0b111:
				CMP_absx();
				break;
			}
			break;
		case 0b111:
			switch (addressing_mode) {
			case 0b000:
				SBC_xind();
				break;
			case 0b001:
				SBC_zp();
				break;
			case 0b010:
				SBC_imm();
				break;
			case 0b011:
				SBC_abs();
				break;
			case 0b100:
				SBC_indy();
				break;
			case 0b101:
				SBC_zpx();
				break;
			case 0b110:
				SBC_absy();
				break;
			case 0b111:
				SBC_absx();
				break;
			}
			break;
		}
		break;

	case 0b10:

		switch (op_code) {
		case 0b000:
			switch (addressing_mode)
			{
			case 0b001:
				ACTION_ZP(ASL);
				cycles += 5;
				break;
			case 0b010:
				ACTION_A(ASL);
				break;
			case 0b011:
				ACTION_ABSOLUTE(ASL);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(ASL);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(ASL);
				break;
			}
			break;
		case 0b001:
			switch (addressing_mode)
			{
			case 0b001:
				ACTION_ZP(ROL);
				cycles += 5;
				break;
			case 0b010:
				ACTION_A(ROL);
				break;
			case 0b011:
				ACTION_ABSOLUTE(ROL);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(ROL);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(ROL);
				break;
			}
			break;
		case 0b010:
			switch (addressing_mode)
			{
			case 0b001:
				ACTION_ZP(LSR);
				cycles += 5;
				break;
			case 0b010:
				ACTION_A(LSR);
				break;
			case 0b011:
				ACTION_ABSOLUTE(LSR);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(LSR);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(LSR);
				break;
			}
			break;
		case 0b011:
			switch (addressing_mode)
			{
			case 0b001:
				ACTION_ZP(ROR);
				cycles += 5;
				break;
			case 0b010:
				ACTION_A(ROR);
				break;
			case 0b011:
				ACTION_ABSOLUTE(ROR);
				break;
			case 0b101:
				ACTION_ZEROPAGEX(ROR);
				break;
			case 0b111:
				ACTION_ABSOLUTEX(ROR);
				break;
			}
			break;
		case 0b100:
			switch (addressing_mode) {
			case 0b001:
				STX_zp();
				break;
			case 0b011:
				STX_abs();
				break;
			case 0b101:
				STX_zpy();
				break;
			case 0b010:
				TXA_imp();
				break;
			case 0b110:
				TXS_imp();
				break;
			}
			break;
		case 0b101:
			switch (addressing_mode) {
			case 0b000:
				LDX_imm();
				break;
			case 0b001:
				LDX_zp();
				break;
			case 0b011:
				LDX_abs();
				break;
			case 0b101:
				LDX_zpy();
				break;
			case 0b111:
				LDX_absy();
				break;
			case 0b010:
				TAX_imp();
				break;
			case 0b110:
				TSX_imp();
				break;
			}
			break;
		case 0b110:
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
			case 0b010:
				DEX_imp();
				break;
			}
			break;
		case 0b111:
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
