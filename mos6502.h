#pragma once

#include <vector>

#define FIRST_PAGE 0x100

#define BRANCH_DECLARATION(ins) \
	void ins(int8_t displacement); \
	void ins ## _rel();

#define IMP_DECLARATION(x)	void x ## _imp();
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

#define READER_GROUP_X_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPY_DECLARATION(x) \
	ABSY_DECLARATION(x)

#define READER_GROUP_Y_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSX_DECLARATION(x)

#define READER_GROUP_CPXY_DECLARATIONS(x) \
	IMM_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x)

#define WRITER_GROUP_A_DECLARATIONS(x) \
	XIND_DECLARATION(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	INDY_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSY_DECLARATION(x) \
	ABSX_DECLARATION(x)

#define WRITER_GROUP_X_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPY_DECLARATION(x)

#define WRITER_GROUP_Y_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x)

#define INCDEC_GROUP_A_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSX_DECLARATION(x)

#define ROTATION_GROUP_DECLARATIONS(x) \
	ZP_DECLARATION(x) \
	IMP_DECLARATION(x) \
	ABS_DECLARATION(x) \
	ZPX_DECLARATION(x) \
	ABSX_DECLARATION(x)

#define INS(x, y) std::pair<instruction_t, unsigned>(&mos6502:: x, y)

class mos6502
{
public:
	mos6502();
	~mos6502();

	virtual void start(uint16_t address);
	virtual void run();

	virtual void reset();
	virtual void irq();
	virtual void nmi();

	uint64_t getCycles() const
	{
		return cycles;
	}

protected:
	virtual bool execute(uint8_t instruction);
	virtual void ___();

	void resetRegisters();

	bool step();

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
	const uint16_t irq_vector = 0xfffe;
	const uint16_t reset_vector = 0xfffc;
	const uint16_t nmi_vector = 0xfffa;

	typedef void (mos6502::*instruction_t)();
	std::vector<std::pair<instruction_t, unsigned>> instructions =
	{
		//		0 					1					2					3					4					5					6					7					8					9					A					B					C					D					E					F
		/* 0 */	INS(BRK_imp, 7),	INS(ORA_xind, 6),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ORA_zp, 4),		INS(ASL_zp, 5),		INS(___, 0),		INS(PHP_imp, 3),	INS(ORA_imm, 2),	INS(ASL_imp, 2),	INS(___, 0),		INS(___, 0),		INS(ORA_abs, 4),	INS(ASL_abs, 6),	INS(___, 0),
		/* 1 */	INS(BPL_rel, 2),	INS(ORA_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ORA_zpx, 4),	INS(ASL_zpx, 6),	INS(___, 0),		INS(CLC_imp, 2),	INS(ORA_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ORA_absx, 4),	INS(ASL_absx, 7),	INS(___, 0),
		/* 2 */	INS(JSR_abs, 6),	INS(AND_xind, 6),	INS(___, 0),		INS(___, 0),		INS(BIT_zp, 3),		INS(AND_zp, 3),		INS(ROL_zp, 5),		INS(___, 0),		INS(PLP_imp, 4),	INS(AND_imm, 2),	INS(ROL_imp, 2),	INS(___, 0),		INS(BIT_abs, 4),	INS(AND_abs, 4),	INS(ROL_abs, 6),	INS(___, 0),
		/* 3 */	INS(BMI_rel, 2),	INS(AND_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(AND_zpx, 4),	INS(ROL_zpx, 6),	INS(___, 0),		INS(SEC_imp, 2),	INS(AND_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(AND_absx, 4),	INS(ROL_absx, 7),	INS(___, 0),
		/* 4 */	INS(RTI_imp, 6),	INS(EOR_xind, 6),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(EOR_zp, 3),		INS(LSR_zp, 5),		INS(___, 0),		INS(PHA_imp, 3),	INS(EOR_imm, 2),	INS(LSR_imp, 2),	INS(___, 0),		INS(JMP_abs, 3),	INS(EOR_abs, 4),	INS(LSR_abs, 6),	INS(___, 0),
		/* 5 */	INS(BVC_rel, 2),	INS(EOR_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(EOR_zpx, 4),	INS(LSR_zpx, 6),	INS(___, 0),		INS(CLI_imp, 2),	INS(EOR_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(EOR_absx, 4),	INS(LSR_absx, 7),	INS(___, 0),
		/* 6 */	INS(RTS_imp, 6),	INS(ADC_xind, 6),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ADC_zp, 3),		INS(ROR_zp, 5),		INS(___, 0),		INS(PLA_imp, 4),	INS(ADC_imm, 2),	INS(ROR_imp, 2),	INS(___, 0),		INS(JMP_ind, 5),	INS(ADC_abs, 4),	INS(ROR_abs, 6),	INS(___, 0),
		/* 7 */	INS(BVS_rel, 2),	INS(ADC_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ADC_zpx, 4),	INS(ROR_zpx, 6),	INS(___, 0),		INS(SEI_imp, 2),	INS(ADC_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(ADC_absx, 4),	INS(ROR_absx, 7),	INS(___, 0),
		/* 8 */	INS(___, 0),		INS(STA_xind, 6),	INS(___, 0),		INS(___, 0),		INS(STY_zp, 3),		INS(STA_zp, 3),		INS(STX_zp, 3),		INS(___, 0),		INS(DEY_imp, 2),	INS(___, 0),		INS(TXA_imp, 2),	INS(___, 0),		INS(STY_abs, 4),	INS(STA_abs, 4),	INS(STX_abs, 4),	INS(___, 0),
		/* 9 */	INS(BCC_rel, 2),	INS(STA_indy, 6),	INS(___, 0),		INS(___, 0),		INS(STY_zpx, 4),	INS(STA_zpx, 4),	INS(STX_zpy, 4),	INS(___, 0),		INS(TYA_imp, 2),	INS(STA_absy, 5),	INS(TXS_imp, 2),	INS(___, 0),		INS(___, 0),		INS(STA_absx, 5),	INS(___, 0),		INS(___, 0),
		/* A */	INS(LDY_imm, 2),	INS(LDA_xind, 6),	INS(LDX_imm, 2),	INS(___, 0),		INS(LDY_zp, 3),		INS(LDA_zp, 3),		INS(LDX_zp, 3),		INS(___, 0),		INS(TAY_imp, 2),	INS(LDA_imm, 2),	INS(TAX_imp, 2),	INS(___, 0),		INS(LDY_abs, 4),	INS(LDA_abs, 4),	INS(LDX_abs, 4),	INS(___, 0),
		/* B */	INS(BCS_rel, 2),	INS(LDA_indy, 5),	INS(___, 0),		INS(___, 0),		INS(LDY_zpx, 4),	INS(LDA_zpx, 4),	INS(LDX_zpy, 4),	INS(___, 0),		INS(CLV_imp, 2),	INS(LDA_absy, 4),	INS(TSX_imp, 2),	INS(___, 0),		INS(LDY_absx, 4),	INS(LDA_absx, 4),	INS(LDX_absy, 4),	INS(___, 0),
		/* C */	INS(CPY_imm, 2),	INS(CMP_xind, 6),	INS(___, 0),		INS(___, 0),		INS(CPY_zp, 3),		INS(CMP_zp, 3),		INS(DEC_zp, 5),		INS(___, 0),		INS(INY_imp, 2),	INS(CMP_imm, 2),	INS(DEX_imp, 2),	INS(___, 0),		INS(CPY_abs, 4),	INS(CMP_abs, 4),	INS(DEC_abs, 6),	INS(___, 0),
		/* D */	INS(BNE_rel, 2),	INS(CMP_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(CMP_zpx, 4),	INS(DEC_zpx, 6),	INS(___, 0),		INS(CLD_imp, 2),	INS(CMP_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(CMP_absx, 4),	INS(DEC_absx, 7),	INS(___, 0),
		/* E */	INS(CPX_imm, 2),	INS(SBC_xind, 6),	INS(___, 0),		INS(___, 0),		INS(CPX_zp, 3),		INS(SBC_zp, 3),		INS(INC_zp, 5),		INS(___, 0),		INS(INX_imp, 2),	INS(SBC_imm, 2),	INS(NOP_imp, 2),	INS(___, 0),		INS(CPX_abs, 4),	INS(SBC_abs, 4),	INS(INC_abs, 6),	INS(___, 0),
		/* F */	INS(BEQ_rel, 2),	INS(SBC_indy, 5),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(SBC_zpx, 4),	INS(INC_zpx, 6),	INS(___, 0),		INS(SED_imp, 2),	INS(SBC_absy, 4),	INS(___, 0),		INS(___, 0),		INS(___, 0),		INS(SBC_absx, 4),	INS(INC_absx, 7),	INS(___, 0),
	};

	BRANCH_DECLARATION(BCS)
	BRANCH_DECLARATION(BCC)

	BRANCH_DECLARATION(BMI)
	BRANCH_DECLARATION(BPL)

	BRANCH_DECLARATION(BEQ)
	BRANCH_DECLARATION(BNE)

	BRANCH_DECLARATION(BVS)
	BRANCH_DECLARATION(BVC)

	void branch(int8_t displacement);

	void NOP_imp();

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

	READER_GROUP_A_DECLARATIONS(ORA)
	READER_GROUP_A_DECLARATIONS(AND)
	READER_GROUP_A_DECLARATIONS(EOR)
	READER_GROUP_A_DECLARATIONS(ADC)
	READER_GROUP_A_DECLARATIONS(LDA)
	READER_GROUP_A_DECLARATIONS(CMP)
	READER_GROUP_A_DECLARATIONS(SBC)

	READER_GROUP_X_DECLARATIONS(LDX)
	READER_GROUP_Y_DECLARATIONS(LDY)

	READER_GROUP_CPXY_DECLARATIONS(CPX)
	READER_GROUP_CPXY_DECLARATIONS(CPY)

	WRITER_GROUP_A_DECLARATIONS(STA)
	WRITER_GROUP_X_DECLARATIONS(STX)
	WRITER_GROUP_Y_DECLARATIONS(STY)

	INCDEC_GROUP_A_DECLARATIONS(INC)
	INCDEC_GROUP_A_DECLARATIONS(DEC)

	ROTATION_GROUP_DECLARATIONS(ASL)
	ROTATION_GROUP_DECLARATIONS(ROL)
	ROTATION_GROUP_DECLARATIONS(LSR)
	ROTATION_GROUP_DECLARATIONS(ROR)

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

	virtual uint8_t getByte(uint16_t offset) = 0;
	virtual void setByte(uint16_t offset, uint8_t value) = 0;

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

	bool updateFlag_Zero(uint8_t value)
	{
		return !value ? P |= F_Z, true : false;
	}

	bool updateFlag_Negative(int8_t value)
	{
		return value < 0 ? P |= F_N, true : false;
	}

	void updateFlags_ZeroNegative(uint8_t value)
	{
		if (!updateFlag_Zero(value))
			updateFlag_Negative(value);
	}

	void reflectFlags_ZeroNegative(uint8_t value)
	{
		P &= ~(F_N | F_Z);
		updateFlags_ZeroNegative(value);
	}

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
};
