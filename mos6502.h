#pragma once

#include <vector>
#include <map>

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

#define INS(x) &mos6502:: x

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

	typedef void (mos6502::*instruction_t)();
	std::vector<instruction_t> instructions =
	{
		//		0 				1				2				3				4				5				6				7				8				9				A				B				C				D				E				F
		/* 0 */	INS(BRK_imp),	INS(ORA_xind),	INS(___),		INS(___),		INS(___),		INS(ORA_zp),	INS(ASL_zp),	INS(___),		INS(PHP_imp),	INS(ORA_imm),	INS(ASL_imp),	INS(___),		INS(___),		INS(ORA_abs),	INS(ASL_abs),	INS(___),
		/* 1 */	INS(BPL_rel),	INS(ORA_indy),	INS(___),		INS(___),		INS(___),		INS(ORA_zpx),	INS(ASL_zpx),	INS(___),		INS(CLC_imp),	INS(ORA_absy),	INS(___),		INS(___),		INS(___),		INS(ORA_absx),	INS(ASL_absx),	INS(___),
		/* 2 */	INS(JSR_abs),	INS(AND_xind),	INS(___),		INS(___),		INS(BIT_zp),	INS(AND_zp),	INS(ROL_zp),	INS(___),		INS(PLP_imp),	INS(AND_imm),	INS(ROL_imp),	INS(___),		INS(BIT_abs),	INS(AND_abs),	INS(ROL_abs),	INS(___),
		/* 3 */	INS(BMI_rel),	INS(AND_indy),	INS(___),		INS(___),		INS(___),		INS(AND_zpx),	INS(ROL_zpx),	INS(___),		INS(SEC_imp),	INS(AND_absy),	INS(___),		INS(___),		INS(___),		INS(AND_absx),	INS(ROL_absx),	INS(___),
		/* 4 */	INS(RTI_imp),	INS(EOR_xind),	INS(___),		INS(___),		INS(___),		INS(EOR_zp),	INS(LSR_zp),	INS(___),		INS(PHA_imp),	INS(EOR_imm),	INS(LSR_imp),	INS(___),		INS(JMP_abs),	INS(EOR_abs),	INS(LSR_abs),	INS(___),
		/* 5 */	INS(BVC_rel),	INS(EOR_indy),	INS(___),		INS(___),		INS(___),		INS(EOR_zpx),	INS(LSR_zpx),	INS(___),		INS(CLI_imp),	INS(EOR_absy),	INS(___),		INS(___),		INS(___),		INS(EOR_absx),	INS(LSR_absx),	INS(___),
		/* 6 */	INS(RTS_imp),	INS(ADC_xind),	INS(___),		INS(___),		INS(___),		INS(ADC_zp),	INS(ROR_zp),	INS(___),		INS(PLA_imp),	INS(ADC_imm),	INS(ROR_imp),	INS(___),		INS(JMP_ind),	INS(ADC_abs),	INS(ROR_abs),	INS(___),
		/* 7 */	INS(BVS_rel),	INS(ADC_indy),	INS(___),		INS(___),		INS(___),		INS(ADC_zpx),	INS(ROR_zpx),	INS(___),		INS(SEI_imp),	INS(ADC_absy),	INS(___),		INS(___),		INS(___),		INS(ADC_absx),	INS(ROR_absx),	INS(___),
		/* 8 */	INS(___),		INS(STA_xind),	INS(___),		INS(___),		INS(STY_zp),	INS(STA_zp),	INS(STX_zp),	INS(___),		INS(DEY_imp),	INS(___),		INS(TXA_imp),	INS(___),		INS(STY_abs),	INS(STA_abs),	INS(STX_abs),	INS(___),
		/* 9 */	INS(BCC_rel),	INS(STA_indy),	INS(___),		INS(___),		INS(STY_zpx),	INS(STA_zpx),	INS(STX_zpy),	INS(___),		INS(TYA_imp),	INS(STA_absy),	INS(TXS_imp),	INS(___),		INS(___),		INS(STA_absx),	INS(___),		INS(___),
		/* A */	INS(LDY_imm),	INS(LDA_xind),	INS(LDX_imm),	INS(___),		INS(LDY_zp),	INS(LDA_zp),	INS(LDX_zp),	INS(___),		INS(TAY_imp),	INS(LDA_imm),	INS(TAX_imp),	INS(___),		INS(LDY_abs),	INS(LDA_abs),	INS(LDX_abs),	INS(___),
		/* B */	INS(BCS_rel),	INS(LDA_indy),	INS(___),		INS(___),		INS(LDY_zpx),	INS(LDA_zpx),	INS(LDX_zpy),	INS(___),		INS(CLV_imp),	INS(LDA_absy),	INS(TSX_imp),	INS(___),		INS(LDY_absx),	INS(LDA_absx),	INS(LDX_absy),	INS(___),
		/* C */	INS(CPY_imm),	INS(CMP_xind),	INS(___),		INS(___),		INS(CPY_zp),	INS(CMP_zp),	INS(DEC_zp),	INS(___),		INS(INY_imp),	INS(CMP_imm),	INS(DEX_imp),	INS(___),		INS(CPY_abs),	INS(CMP_abs),	INS(DEC_abs),	INS(___),
		/* D */	INS(BNE_rel),	INS(CMP_indy),	INS(___),		INS(___),		INS(___),		INS(CMP_zpx),	INS(DEC_zpx),	INS(___),		INS(CLD_imp),	INS(CMP_absy),	INS(___),		INS(___),		INS(___),		INS(CMP_absx),	INS(DEC_absx),	INS(___),
		/* E */	INS(CPX_imm),	INS(SBC_xind),	INS(___),		INS(___),		INS(CPX_zp),	INS(SBC_zp),	INS(INC_zp),	INS(___),		INS(INX_imp),	INS(SBC_imm),	INS(NOP_imp),	INS(___),		INS(CPX_abs),	INS(SBC_abs),	INS(INC_abs),	INS(___),
		/* F */	INS(BEQ_rel),	INS(SBC_indy),	INS(___),		INS(___),		INS(___),		INS(SBC_zpx),	INS(INC_zpx),	INS(___),		INS(SED_imp),	INS(SBC_absy),	INS(___),		INS(___),		INS(___),		INS(SBC_absx),	INS(INC_absx),	INS(___),
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

	void ___();

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
