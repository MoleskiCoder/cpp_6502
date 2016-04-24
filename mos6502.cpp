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
	return value;
}

uint8_t mos6502::readByte_ZeroPage()
{
	auto zp = fetchByte();
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
	return getByte(lowByte(zp + X));
}

uint8_t mos6502::readByte_ZeroPageY()
{
	auto zp = fetchByte();
	return getByte(lowByte(zp + Y));
}

uint8_t mos6502::readByte_AbsoluteX()
{
	auto base = fetchWord();
	uint16_t offset = base + X;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return getByte(offset);
}

uint8_t mos6502::readByte_AbsoluteY()
{
	auto base = fetchWord();
	uint16_t offset = base + Y;
	if (lowByte(offset) == 0xff)
		cycles += 1;
	return getByte(offset);
}

uint8_t mos6502::readByte_Absolute()
{
	auto address = fetchWord();
	return getByte(address);
}

uint8_t mos6502::readByte_IndexedIndirectX()
{
	auto zp = fetchByte();
	return getByte(getWord(lowByte(zp + X)));
}

uint8_t mos6502::readByte_IndirectIndexedY()
{
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
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	auto address = fetchWord();
	setByte(address, value);
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	auto zp = fetchByte();
	setByte(getWord(lowByte(zp + X)), value);
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	auto zp = fetchByte();
	setByte(getWord(zp) + Y, value);
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	auto zp = fetchByte();
	setByte(lowByte(zp + X), value);
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	auto zp = fetchByte();
	setByte(lowByte(zp + Y), value);
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	auto base = fetchWord();
	setByte((uint16_t)(base + X), value);
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	auto base = fetchWord();
	setByte((uint16_t)(base + Y), value);
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
}

void mos6502::BIT_zp()
{
	BIT(readByte_ZeroPage());
}

void mos6502::BIT_abs()
{
	BIT(readByte_Absolute());
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
		if (P & flag) \
			branch(displacement); \
	}

#define BRANCH_DEFINITION_C(ins, flag) \
	void mos6502:: ins (int8_t displacement) \
	{ \
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

void mos6502::SED_imp() { P |= F_D;		}
void mos6502::CLD_imp() { P &= ~F_D;	}

void mos6502::SEI_imp() { P |= F_I;		}
void mos6502::CLI_imp() { P &= ~F_I;	}

void mos6502::CLV_imp() { P &= ~F_V;	}

void mos6502::SEC_imp() { P |= F_C;		}
void mos6502::CLC_imp() { P &= ~F_C;	}

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
}

void mos6502::DEY_imp()
{
	reflectFlags_ZeroNegative(--Y);
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
}

void mos6502::INY_imp()
{
	reflectFlags_ZeroNegative(++Y);
}

void mos6502::JSR_abs()
{
	auto destination = fetchWord();
	pushWord(PC - 1);
	PC = destination;
}

void mos6502::RTS_imp()
{
	PC = popWord() + 1;
}

void mos6502::PHA_imp()
{
	pushByte(A);
}

void mos6502::PLA_imp()
{
	A = popByte();
	reflectFlags_ZeroNegative(A);
}

void mos6502::PHP_imp()
{
	pushStatus();
}

void mos6502::pushStatus()
{
	P |= (F_B | F_reserved);
	pushByte(P);
}

void mos6502::PLP_imp()
{
	P = popByte();
}

void mos6502::TXS_imp()
{
	S = X;
}

void mos6502::TSX_imp()
{
	X = S;
	reflectFlags_ZeroNegative(X);
}

void mos6502::TAX_imp()
{
	X = A;
	reflectFlags_ZeroNegative(X);
}

void mos6502::TXA_imp()
{
	A = X;
	reflectFlags_ZeroNegative(A);
}

void mos6502::TAY_imp()
{
	Y = A;
	reflectFlags_ZeroNegative(Y);
}

void mos6502::TYA_imp()
{
	A = Y;
	reflectFlags_ZeroNegative(A);
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
}

void mos6502::RTI_imp()
{
	P = popByte();
	PC = popWord();
}

void mos6502::JMP_abs()
{
	auto address = fetchWord();
	PC = address;
}

void mos6502::JMP_ind()
{
	auto address = fetchWord();
	PC = getWord(address);
}

void mos6502::NOP_imp()
{
}

void mos6502::___()
{
	assert(false && "Unknown instruction");
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

#define ACTION_ZP_DEFINITION(ACTION) \
	void mos6502::ACTION ## _zp() \
	{ \
		auto zp = fetchByte(); \
		ACTION((uint16_t)zp); \
	}

#define ACTION_IMP_DEFINITION(ACTION, REGISTER) \
	void mos6502::ACTION ## _imp() \
	{ \
		REGISTER = ACTION(REGISTER); \
	}

#define ACTION_ABS_DEFINITION(ACTION) \
	void mos6502::ACTION ## _abs() \
	{ \
		auto address = fetchWord(); \
		ACTION(address); \
	}

#define ACTION_ZPX_DEFINITION(ACTION) \
	void mos6502::ACTION ## _zpx() \
	{ \
		auto zp = fetchByte(); \
		ACTION((uint16_t)(lowByte(zp + X))); \
	}

#define ACTION_ABSX_DEFINITION(ACTION) \
	void mos6502::ACTION ## _absx() \
	{ \
		auto address = fetchWord(); \
		ACTION((uint16_t)(address + X)); \
	}

#define ROTATION_GROUP_DEFINITIONS(x) \
	ACTION_ZP_DEFINITION(x) \
	ACTION_IMP_DEFINITION(x, A) \
	ACTION_ABS_DEFINITION(x) \
	ACTION_ZPX_DEFINITION(x) \
	ACTION_ABSX_DEFINITION(x)

ROTATION_GROUP_DEFINITIONS(ASL);
ROTATION_GROUP_DEFINITIONS(ROL);
ROTATION_GROUP_DEFINITIONS(LSR);
ROTATION_GROUP_DEFINITIONS(ROR);

#define INCDEC_GROUP_A_DEFINITIONS(x) \
	ACTION_ZP_DEFINITION(x) \
	ACTION_ABS_DEFINITION(x) \
	ACTION_ZPX_DEFINITION(x) \
	ACTION_ABSX_DEFINITION(x)

INCDEC_GROUP_A_DEFINITIONS(INC)
INCDEC_GROUP_A_DEFINITIONS(DEC)

//

void mos6502::step()
{
	auto current = fetchByte();

#ifdef _DEBUG
	instructionCounts[current]++;
#endif

	auto details = instructions[current];

	(this->*details.first)();
	cycles += details.second;
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
