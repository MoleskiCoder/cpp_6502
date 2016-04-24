#pragma once

#include <vector>
#include <map>

#define FIRST_PAGE 0x100

#define ACTION_ZP(ACTION) \
	{ \
		auto zp = fetchByte(); \
		ACTION((uint16_t)zp); \
	}

#define ACTION_A(ACTION) \
	A = ACTION(A);

#define ACTION_ABSOLUTE(ACTION) \
	{ \
		auto address = fetchWord(); \
		ACTION(address); \
	}

#define ACTION_ZEROPAGEX(ACTION) \
	{ \
		auto zp = fetchByte(); \
		ACTION((uint16_t)(lowByte(zp + X))); \
	}

#define ACTION_ABSOLUTEX(ACTION) \
	{ \
		auto address = fetchWord(); \
		ACTION((uint16_t)(address + X)); \
	}

class mos6502
{
public:
	mos6502();
	~mos6502();

	void readRom(std::string path, size_t offset);
	void clearMemory();
	void resetRegisters();

	void run();
	void step();

	std::vector<uint8_t> memory;

	uint64_t cycles;

	uint16_t PC;	// program counter
	uint8_t X;		// index register X
	uint8_t Y;		// index register Y
	uint8_t A;		// accumulator
	uint8_t S;		// stack pointer

	enum F_BITS
	{
		F_N = 0x80,	// Negative
		F_V = 0x40,	// Overflow
		F_reserved = 0x20, // ignored
		F_B = 0x10,	// Break
		F_D = 0x08,	// Decimal(use BCD for arithmetics)
		F_I = 0x04,	// Interrupt(IRQ disable)
		F_Z = 0x02,	// Zero
		F_C = 0x01,	// Carry
	};

	uint8_t P;		// processor status

private:

#define BRANCH_DECLARATION(ins) \
	void ins(int8_t displacement); \
	void ins ## _rel();

	BRANCH_DECLARATION(BCS)
	BRANCH_DECLARATION(BCC)

	BRANCH_DECLARATION(BMI)
	BRANCH_DECLARATION(BPL)

	BRANCH_DECLARATION(BEQ)
	BRANCH_DECLARATION(BNE)

	BRANCH_DECLARATION(BVS)
	BRANCH_DECLARATION(BVC)

	void branch(int8_t displacement);

	void CLV_imp();
	void SEI_imp();
	void CLI_imp();
	void SED_imp();
	void CLD_imp();
	void SEC_imp();
	void CLC_imp();

	void BIT(uint8_t data);
	void BIT_abs();
	void BIT_zp();

	void JMP_abs();
	void JMP_ind();

#define XIND_DECLARATION(x)	void x ## _xind();
#define ZP_DECLARATION(x)	void x ## _zp();
#define IMM_DECLARATION(x)	void x ## _imm();
#define ABS_DECLARATION(x)	void x ## _abs();
#define INDY_DECLARATION(x)	void x ## _indy();
#define ZPX_DECLARATION(x)	void x ## _zpx();
#define ZPY_DECLARATION(x)	void x ## _zpy();
#define ABSX_DECLARATION(x)	void x ## _absx();
#define ABSY_DECLARATION(x)	void x ## _absy();

#define READER_GROUP_A_DECLARATIONS(x) \
	XIND_DECLARATION(x) \
	ZP_DECLARATION(x) \
	IMM_DECLARATION(x) \
	ABS_DECLARATION(x) \
	INDY_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSX_DECLARATION(x) \
	ABSY_DECLARATION(x)

	READER_GROUP_A_DECLARATIONS(ORA)
	READER_GROUP_A_DECLARATIONS(AND)
	READER_GROUP_A_DECLARATIONS(EOR)
	READER_GROUP_A_DECLARATIONS(ADC)
	READER_GROUP_A_DECLARATIONS(LDA)
	READER_GROUP_A_DECLARATIONS(CMP)
	READER_GROUP_A_DECLARATIONS(SBC)

#define READER_GROUP_X_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPY_DECLARATION(x) \
	ABSY_DECLARATION(x)

	READER_GROUP_X_DECLARATIONS(LDX)

#define READER_GROUP_Y_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSX_DECLARATION(x)

	READER_GROUP_Y_DECLARATIONS(LDY)

#define READER_GROUP_CPXY_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \

READER_GROUP_CPXY_DECLARATIONS(CPX)
READER_GROUP_CPXY_DECLARATIONS(CPY)

#define WRITER_GROUP_A_DECLARATIONS(x) \
	XIND_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	INDY_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSY_DECLARATION(x) \
	ABSX_DECLARATION(x)

	WRITER_GROUP_A_DECLARATIONS(STA)

#define WRITER_GROUP_X_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPY_DECLARATION(x)

	WRITER_GROUP_X_DECLARATIONS(STX)

#define WRITER_GROUP_Y_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x)

	WRITER_GROUP_Y_DECLARATIONS(STY)

	void CMP(uint8_t first, uint8_t second);

	void ADC(uint8_t data);
	void ADC_d(uint8_t data);
	void ADC_b(uint8_t data);

	void SBC(uint8_t data);
	void SBC_d(uint8_t data);
	void SBC_b(uint8_t data);

	void CPX(uint8_t data);
	void CPY(uint8_t data);
	void LDY(uint8_t data);
	void ORA(uint8_t data);
	void AND(uint8_t data);
	void EOR(uint8_t data);
	void LDA(uint8_t data);
	void CMP(uint8_t data);
	void LDX(uint8_t data);
	void DEC(uint16_t offset);
	void INC(uint16_t offset);

	void JSR_abs();
	void RTS_imp();
	void PHA_imp();
	void PLA_imp();
	void PHP_imp();
	void PLP_imp();
	void DEX_imp();
	void DEY_imp();
	void INX_imp();
	void INY_imp();
	void TAX_imp();
	void TXA_imp();
	void TAY_imp();
	void TYA_imp();
	void TXS_imp();
	void TSX_imp();
	void BRK_imp();
	void RTI_imp();

	void ASL(uint16_t offset);
	void ROL(uint16_t offset);
	void LSR(uint16_t offset);
	void ROR(uint16_t offset);

	uint8_t ASL(uint8_t data);
	uint8_t ROL(uint8_t data);
	uint8_t LSR(uint8_t data);
	uint8_t ROR(uint8_t data);

	// get/set memory

	uint8_t getByte(uint16_t offset);
	void setByte(uint16_t offset, uint8_t value);

	uint16_t getWord(uint16_t offset);

	// Fetches increment their reference counter

	uint8_t fetchByte(uint16_t& counter);
	uint16_t fetchWord(uint16_t& counter);

	// Fetches without a reference counter increment PC

	uint8_t fetchByte();

	uint16_t fetchWord();
	uint16_t fetchWord_Indirect();

	//

	int8_t readByte_ImmediateDisplacement();

	uint8_t readByte_Immediate();
	uint8_t readByte_ZeroPage();
	uint8_t readByte_Absolute();
	uint8_t readByte_IndexedIndirectX();
	uint8_t readByte_IndirectIndexedY();
	uint8_t readByte_ZeroPageX();
	uint8_t readByte_ZeroPageY();
	uint8_t readByte_AbsoluteX();
	uint8_t readByte_AbsoluteY();

	void writeByte_ZeroPage(uint8_t value);
	void writeByte_Absolute(uint8_t value);
	void writeByte_IndexedIndirectX(uint8_t value);
	void writeByte_IndirectIndexedY(uint8_t value);
	void writeByte_ZeroPageX(uint8_t value);
	void writeByte_ZeroPageY(uint8_t value);
	void writeByte_AbsoluteX(uint8_t value);
	void writeByte_AbsoluteY(uint8_t value);

	//

	void updateFlag_Zero(uint8_t value);
	void updateFlag_Negative(int8_t value);

	void updateFlags_ZeroNegative(uint8_t value);

	void reflectFlags_ZeroNegative(uint8_t value);

	//

	uint8_t lowByte(uint16_t value)
	{
		return value & 0xff;
	}

	uint8_t highByte(uint16_t value)
	{
		return (value & ~0xff) >> 8;
	}

	uint16_t makeWord(uint8_t low, uint8_t high)
	{
		return (high << 8) + low;
	}

	//

	void pushByte(uint8_t value)
	{
		setByte(FIRST_PAGE + S--, value);
	}

	uint8_t popByte()
	{
		return getByte(FIRST_PAGE + ++S);
	}

	void pushWord(uint16_t value)
	{
		pushByte(highByte(value));
		pushByte(lowByte(value));
	}

	uint16_t popWord()
	{
		auto low = popByte();
		auto high = popByte();
		return makeWord(low, high);
	}

	void pushStatus();

#ifdef _DEBUG
	std::map<uint8_t, int> instructionCounts;
#endif
};
