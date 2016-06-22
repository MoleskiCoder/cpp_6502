#include "mos6502.h"

#include <assert.h>

mos6502::mos6502()
{
}

mos6502::~mos6502()
{
}

void mos6502::resetRegisters()
{
	PC = 0x0000;
	X = 0x80;
	Y = 0x00;
	A = 0x00;
	P = F_reserved;
	S = 0xff;
}

//

uint16_t mos6502::getWord(uint16_t offset)
{
	auto low = getByte(offset);
	auto high = getByte(offset + 1);
	return makeWord(low, high);
}

//

uint8_t mos6502::fetchByte()
{
	return getByte(PC++);
}

uint16_t mos6502::fetchWord()
{
	auto word = getWord(PC);
	PC += 2;
	return word;
}

//

uint8_t mos6502::readByte_Immediate()
{
	return fetchByte();
}

uint8_t mos6502::readByte_ZeroPage()
{
	return getByte(fetchByte());
}

int8_t mos6502::readByte_ImmediateDisplacement()
{
	return fetchByte();
}

uint8_t mos6502::readByte_ZeroPageX()
{
	return getByte(lowByte(fetchByte() + X));
}

uint8_t mos6502::readByte_ZeroPageY()
{
	return getByte(lowByte(fetchByte() + Y));
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
	return getByte(fetchWord());
}

uint8_t mos6502::readByte_IndexedIndirectX()
{
	return getByte(getWord(lowByte(fetchByte() + X)));
}

uint8_t mos6502::readByte_IndirectIndexedY()
{
	auto indirection = getWord(fetchByte());
	if (lowByte(indirection) == 0xff)
		cycles += 1;
	return getByte(indirection + Y);
}

//

void mos6502::writeByte_ZeroPage(uint8_t value)
{
	setByte(fetchByte(), value);
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	setByte(fetchWord(), value);
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	setByte(getWord(lowByte(fetchByte() + X)), value);
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	setByte(getWord(fetchByte()) + Y, value);
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	setByte(lowByte(fetchByte() + X), value);
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	setByte(lowByte(fetchByte() + Y), value);
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	setByte((uint16_t)(fetchWord() + X), value);
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	setByte((uint16_t)(fetchWord() + Y), value);
}

//

void mos6502::CMP(uint8_t first, uint8_t second)
{
	P &= ~(F_N | F_Z | F_C);

	uint16_t result = first - second;

	updateFlags_ZeroNegative((uint8_t)result);

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

	updateFlags_ZeroNegative((uint8_t)sum);

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

	updateFlags_ZeroNegative((uint8_t)difference);

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

	updateFlags_ZeroNegative((uint8_t)difference);

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
	P |= F_B;
	pushByte(P);
}

void mos6502::PLP_imp()
{
	P = popByte() | F_reserved;
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
	updateFlags_ZeroNegative(result);

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

	updateFlags_ZeroNegative(result);

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

	updateFlags_ZeroNegative(result);

	return result;
}

void mos6502::ASL(uint16_t offset)
{
	setByte(offset, ASL(getByte(offset)));
}

void mos6502::ROL(uint16_t offset)
{
	setByte(offset, ROL(getByte(offset)));
}

void mos6502::LSR(uint16_t offset)
{
	setByte(offset, LSR(getByte(offset)));
}

void mos6502::ROR(uint16_t offset)
{
	setByte(offset, ROR(getByte(offset)));
}

void mos6502::reset()
{
	PC = getWord(reset_vector);
}

void mos6502::interrupt(uint16_t vector)
{
	pushWord(PC);
	pushByte(P);
	P |= F_I;
	PC = getWord(vector);
}

void mos6502::nmi()
{
	interrupt(nmi_vector);
}

void mos6502::irq()
{
	interrupt(irq_vector);
}


void mos6502::BRK_imp()
{
	pushWord(PC + 1);
	PHP_imp();
	P |= F_I;
	PC = getWord(irq_vector);
}

void mos6502::RTI_imp()
{
	PLP_imp();
	PC = popWord();
}

void mos6502::JMP_abs()
{
	PC = fetchWord();
}

void mos6502::JMP_ind()
{
	PC = getWord(fetchWord());
}

void mos6502::NOP_imp()
{
}

void mos6502::___()
{
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
		ACTION((uint16_t)fetchByte()); \
	}

#define ACTION_IMP_DEFINITION(ACTION, REGISTER) \
	void mos6502::ACTION ## _imp() \
	{ \
		REGISTER = ACTION(REGISTER); \
	}

#define ACTION_ABS_DEFINITION(ACTION) \
	void mos6502::ACTION ## _abs() \
	{ \
		ACTION(fetchWord()); \
	}

#define ACTION_ZPX_DEFINITION(ACTION) \
	void mos6502::ACTION ## _zpx() \
	{ \
		ACTION((uint16_t)(lowByte(fetchByte() + X))); \
	}

#define ACTION_ABSX_DEFINITION(ACTION) \
	void mos6502::ACTION ## _absx() \
	{ \
		ACTION((uint16_t)(fetchWord() + X)); \
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

void mos6502::start(uint16_t address)
{
	PC = address;
}

bool mos6502::step()
{
	assert(P & F_reserved);		// The F_reserved flag *must* always be set on the NMOS 6502

#ifdef _DEBUG
	printf("\n[%09lld] PC=%04x:P=%02x, A=%02x, X=%02x, Y=%02x, S=%02x	", cycles, PC, P, A, X, Y, S);
#endif

	return execute(fetchByte());
}

bool mos6502::execute(uint8_t instruction)
{
	const auto& details = instructions[instruction];

	auto method = details.vector;
	auto count = details.count;

#ifdef _DEBUG
	dump_bytevalue(instruction);

	auto mode = details.mode;
	auto mnemomic = details.display;

	const auto& dumper = addressingMode_Dumper[mode];

	(this->*(dumper.first))();

	printf("	%s ", mnemomic.c_str());
	(this->*(dumper.second))();
#endif

	(this->*method)();
	cycles += count;

	return true;
}

void mos6502::run()
{
	cycles = 0;
	while (step());
}
