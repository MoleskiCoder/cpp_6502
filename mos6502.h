#pragma once

#include "config.h"

#include <cstdint>
#include <vector>
#include <map>
#include <stdio.h>

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

#define INS(INS, MODE, CYCLES) \
	{ &mos6502:: INS ## _ ## MODE, CYCLES, MODE, #INS }

#define DMP(ADDR, BYTE) \
	{ ADDR,	std::pair<instruction_t, instruction_t>(&mos6502::dump_ ## BYTE, &mos6502::dump_ ## ADDR)	}

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
	virtual void interrupt(uint16_t vector);
	virtual bool execute(uint8_t instruction);
	virtual void ___();

	void resetRegisters();

	virtual bool step();

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
	const uint16_t page_1 = 0x100;
	const uint16_t irq_vector = 0xfffe;
	const uint16_t reset_vector = 0xfffc;
	const uint16_t nmi_vector = 0xfffa;

	typedef void (mos6502::*instruction_t)();

	enum addressing_mode
	{
		_,	// For the undefined instruction/mode handler
		imp, imm,
		rel,
		xind, indy,
		zp, zpx, zpy,
		abs, absx, absy,
		ind
	};

	void dump_nothing()	{ }

	void dump_bytevalue(uint8_t value)	{ printf("%02x", value);			}
	void dump_byte(uint16_t address)	{ dump_bytevalue(getByte(address));	}
	void dump_byte()					{ dump_byte(PC);					}
	void dump_dbyte()					{ dump_byte(PC); dump_byte(PC + 1);	}

	void dump_imp()		{												}
	void dump_a()		{ printf("A");									}
	void dump_imm()		{ printf("#$%02x", getByte(PC));				}
	void dump_abs()		{ printf("$%04x", getWord(PC));					}
	void dump_zp()		{ printf("$%02x", getByte(PC));					}
	void dump_zpx()		{ printf("$%02x,X", getByte(PC));				}
	void dump_zpy()		{ printf("$%02x,Y", getByte(PC));				}
	void dump_absx()	{ printf("$%04x,X", getWord(PC));				}
	void dump_absy()	{ printf("$%04x,Y", getWord(PC));				}
	void dump_xind()	{ printf("($%02x,X)", getByte(PC));				}
	void dump_indy()	{ printf("($%02x),Y", getByte(PC));				}
	void dump_ind()		{ printf("($%04x)", getWord(PC));				}
	void dump_rel()		{ printf("$%04x", 1 + PC + (int8_t)getByte(PC)); }

	std::map<addressing_mode, std::pair<instruction_t, instruction_t>> addressingMode_Dumper =
	{
		DMP(imp, nothing),
		DMP(xind, byte),
		DMP(zp, byte),
		DMP(imm, byte),
		DMP(abs, dbyte),
		DMP(indy, byte),
		DMP(zpx, byte),
		DMP(zpy, byte),
		DMP(absx, dbyte),
		DMP(absy, dbyte),
		DMP(rel, byte),
		DMP(ind, dbyte),
	};

	struct instruction
	{
		instruction_t vector;
		unsigned count;
		addressing_mode mode;
		std::string display;
	};

	std::vector<instruction> instructions =
	{
		//		0 					1					2					3					4					5					6					7					8					9					A					B					C					D					E					F
		/* 0 */	INS(BRK,imp, 7),	INS(ORA,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ORA,zp, 4),		INS(ASL,zp, 5),		INS(_,_, 0),		INS(PHP,imp, 3),	INS(ORA,imm, 2),	INS(ASL,imp, 2),	INS(_,_, 0),		INS(_,_, 0),		INS(ORA,abs, 4),	INS(ASL,abs, 6),	INS(_,_, 0),
		/* 1 */	INS(BPL,rel, 2),	INS(ORA,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ORA,zpx, 4),	INS(ASL,zpx, 6),	INS(_,_, 0),		INS(CLC,imp, 2),	INS(ORA,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ORA,absx, 4),	INS(ASL,absx, 7),	INS(_,_, 0),
		/* 2 */	INS(JSR,abs, 6),	INS(AND,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(BIT,zp, 3),		INS(AND,zp, 3),		INS(ROL,zp, 5),		INS(_,_, 0),		INS(PLP,imp, 4),	INS(AND,imm, 2),	INS(ROL,imp, 2),	INS(_,_, 0),		INS(BIT,abs, 4),	INS(AND,abs, 4),	INS(ROL,abs, 6),	INS(_,_, 0),
		/* 3 */	INS(BMI,rel, 2),	INS(AND,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(AND,zpx, 4),	INS(ROL,zpx, 6),	INS(_,_, 0),		INS(SEC,imp, 2),	INS(AND,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(AND,absx, 4),	INS(ROL,absx, 7),	INS(_,_, 0),
		/* 4 */	INS(RTI,imp, 6),	INS(EOR,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(EOR,zp, 3),		INS(LSR,zp, 5),		INS(_,_, 0),		INS(PHA,imp, 3),	INS(EOR,imm, 2),	INS(LSR,imp, 2),	INS(_,_, 0),		INS(JMP,abs, 3),	INS(EOR,abs, 4),	INS(LSR,abs, 6),	INS(_,_, 0),
		/* 5 */	INS(BVC,rel, 2),	INS(EOR,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(EOR,zpx, 4),	INS(LSR,zpx, 6),	INS(_,_, 0),		INS(CLI,imp, 2),	INS(EOR,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(EOR,absx, 4),	INS(LSR,absx, 7),	INS(_,_, 0),
		/* 6 */	INS(RTS,imp, 6),	INS(ADC,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ADC,zp, 3),		INS(ROR,zp, 5),		INS(_,_, 0),		INS(PLA,imp, 4),	INS(ADC,imm, 2),	INS(ROR,imp, 2),	INS(_,_, 0),		INS(JMP,ind, 5),	INS(ADC,abs, 4),	INS(ROR,abs, 6),	INS(_,_, 0),
		/* 7 */	INS(BVS,rel, 2),	INS(ADC,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ADC,zpx, 4),	INS(ROR,zpx, 6),	INS(_,_, 0),		INS(SEI,imp, 2),	INS(ADC,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(ADC,absx, 4),	INS(ROR,absx, 7),	INS(_,_, 0),
		/* 8 */	INS(_,_, 0),		INS(STA,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(STY,zp, 3),		INS(STA,zp, 3),		INS(STX,zp, 3),		INS(_,_, 0),		INS(DEY,imp, 2),	INS(_,_, 0),		INS(TXA,imp, 2),	INS(_,_, 0),		INS(STY,abs, 4),	INS(STA,abs, 4),	INS(STX,abs, 4),	INS(_,_, 0),
		/* 9 */	INS(BCC,rel, 2),	INS(STA,indy, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(STY,zpx, 4),	INS(STA,zpx, 4),	INS(STX,zpy, 4),	INS(_,_, 0),		INS(TYA,imp, 2),	INS(STA,absy, 5),	INS(TXS,imp, 2),	INS(_,_, 0),		INS(_,_, 0),		INS(STA,absx, 5),	INS(_,_, 0),		INS(_,_, 0),
		/* A */	INS(LDY,imm, 2),	INS(LDA,xind, 6),	INS(LDX,imm, 2),	INS(_,_, 0),		INS(LDY,zp, 3),		INS(LDA,zp, 3),		INS(LDX,zp, 3),		INS(_,_, 0),		INS(TAY,imp, 2),	INS(LDA,imm, 2),	INS(TAX,imp, 2),	INS(_,_, 0),		INS(LDY,abs, 4),	INS(LDA,abs, 4),	INS(LDX,abs, 4),	INS(_,_, 0),
		/* B */	INS(BCS,rel, 2),	INS(LDA,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(LDY,zpx, 4),	INS(LDA,zpx, 4),	INS(LDX,zpy, 4),	INS(_,_, 0),		INS(CLV,imp, 2),	INS(LDA,absy, 4),	INS(TSX,imp, 2),	INS(_,_, 0),		INS(LDY,absx, 4),	INS(LDA,absx, 4),	INS(LDX,absy, 4),	INS(_,_, 0),
		/* C */	INS(CPY,imm, 2),	INS(CMP,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(CPY,zp, 3),		INS(CMP,zp, 3),		INS(DEC,zp, 5),		INS(_,_, 0),		INS(INY,imp, 2),	INS(CMP,imm, 2),	INS(DEX,imp, 2),	INS(_,_, 0),		INS(CPY,abs, 4),	INS(CMP,abs, 4),	INS(DEC,abs, 6),	INS(_,_, 0),
		/* D */	INS(BNE,rel, 2),	INS(CMP,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(CMP,zpx, 4),	INS(DEC,zpx, 6),	INS(_,_, 0),		INS(CLD,imp, 2),	INS(CMP,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(CMP,absx, 4),	INS(DEC,absx, 7),	INS(_,_, 0),
		/* E */	INS(CPX,imm, 2),	INS(SBC,xind, 6),	INS(_,_, 0),		INS(_,_, 0),		INS(CPX,zp, 3),		INS(SBC,zp, 3),		INS(INC,zp, 5),		INS(_,_, 0),		INS(INX,imp, 2),	INS(SBC,imm, 2),	INS(NOP,imp, 2),	INS(_,_, 0),		INS(CPX,abs, 4),	INS(SBC,abs, 4),	INS(INC,abs, 6),	INS(_,_, 0),
		/* F */	INS(BEQ,rel, 2),	INS(SBC,indy, 5),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(SBC,zpx, 4),	INS(INC,zpx, 6),	INS(_,_, 0),		INS(SED,imp, 2),	INS(SBC,absy, 4),	INS(_,_, 0),		INS(_,_, 0),		INS(_,_, 0),		INS(SBC,absx, 4),	INS(INC,absx, 7),	INS(_,_, 0),
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

	static uint8_t lowByte(uint16_t value)
	{
		return value & 0xff;
	}

	static uint8_t highByte(uint16_t value)
	{
		return (value & ~0xff) >> 8;
	}

	static uint16_t makeWord(uint8_t low, uint8_t high)
	{
		return (high << 8) + low;
	}

	//

	void pushByte(uint8_t value)
	{
		setByte(page_1 + S--, value);
	}

	uint8_t popByte()
	{
		return getByte(page_1 + ++S);
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
