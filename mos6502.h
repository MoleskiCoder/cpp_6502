#pragma once

#include <vector>
#include <map>

#ifdef _DEBUG

#	define DUMP_PREFIX(x)			printf("	" #x " ")

#	define DUMP_A					printf("A")
#	define DUMP_IMMEDIATE(x)		printf("#%02x", x)
#	define DUMP_ABSOLUTE(x)			printf("$%04x", x)
#	define DUMP_ZEROPAGE(x)			printf("$%02x", x)
#	define DUMP_ZEROPAGEX(x)		printf("$%02x,X", x)
#	define DUMP_ZEROPAGEY(x)		printf("$%02x,Y", x)
#	define DUMP_ABSOLUTEX(x)		printf("$%04x,X", x)
#	define DUMP_ABSOLUTEY(x)		printf("$%04x,Y", x)
#	define DUMP_INDEXEDINDIRECTX(x)	printf("($%02x,X)", x);
#	define DUMP_INDIRECTINDEXEDY(x)	printf("($%02x),Y", x);
#	define DUMP_INDIRECT(x)			printf("($%04x)", x);

#	define DUMP_BYTEVALUE(x)		printf("%02x", x)
#	define DUMP_BYTE(x)				DUMP_BYTEVALUE(getByte(x))
#	define DUMP_DBYTE(x)			DUMP_BYTE(x), DUMP_BYTE(x + 1);

#else

#	define DUMP_PREFIX(x)

#	define DUMP_A
#	define DUMP_IMMEDIATE(x)
#	define DUMP_ABSOLUTE(x)
#	define DUMP_ZEROPAGE(x)
#	define DUMP_ZEROPAGEX(x)
#	define DUMP_ZEROPAGEY(x)
#	define DUMP_ABSOLUTEX(x)
#	define DUMP_ABSOLUTEY(x)
#	define DUMP_INDEXEDINDIRECTX(x)
#	define DUMP_INDIRECTINDEXEDY(x)

#	define DUMP_BYTEVALUE(x)
#	define DUMP_BYTE(x)
#	define DUMP_DBYTE(x)

#endif

#define FIRST_PAGE 0x100

#define FETCH_ADDR_INDEXEDINDIRECTX		getWord(lowByte(fetchByte() + X))
#define FETCH_ADDR_INDIRECTINDEXEDY		getWord(fetchByte()) + Y
#define FETCH_ADDR_ZEROPAGEX			lowByte(fetchByte() + X)
#define FETCH_ADDR_ZEROPAGEY			lowByte(fetchByte() + Y)
#define FETCH_ADDR_ABSOLUTEX			(uint16_t)(fetchWord() + X)
#define FETCH_ADDR_ABSOLUTEY			(uint16_t)(fetchWord() + Y)

#define ACTION_ZP(ACTION) \
	DUMP_BYTE(PC); \
	DUMP_PREFIX(ACTION); \
	{ \
		auto zp = fetchByte(); \
		DUMP_ZEROPAGE(zp); \
		ACTION(zp); \
	}

#define ACTION_A(ACTION) \
	DUMP_PREFIX(ACTION); \
	DUMP_A; \
	A = ACTION(A);

#define ACTION_ABSOLUTE(ACTION) \
	DUMP_DBYTE(PC); \
	DUMP_PREFIX(ACTION); \
	{ \
		auto address = fetchWord(); \
		DUMP_ABSOLUTE(address); \
		ACTION(address); \
	}

#define ACTION_ZEROPAGEX(ACTION) \
	DUMP_BYTE(PC); \
	DUMP_PREFIX(ACTION); \
	{ \
		auto zp = fetchByte(); \
		DUMP_ZEROPAGEX(zp); \
		ACTION(lowByte(zp + X)); \
	}

#define ACTION_ABSOLUTEX(ACTION) \
	DUMP_DBYTE(PC); \
	DUMP_PREFIX(ACTION); \
	{ \
		auto address = fetchWord(); \
		DUMP_ABSOLUTEX(address); \
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

	uint16_t PC;	// program counter
	uint8_t X;		// index register X
	uint8_t Y;		// index register Y
	uint8_t A;		// accumulator
	uint8_t S;		// stack pointer

	enum F_BITS
	{
		F_N = 0x80,	// Negative
		F_V = 0x40,	// Overflow
					// ignored
		F_B = 0x10,	// Break
		F_D = 0x08,	// Decimal(use BCD for arithmetics)
		F_I = 0x04,	// Interrupt(IRQ disable)
		F_Z = 0x02,	// Zero
		F_C = 0x01,	// Carry
	};

	uint8_t P;		// processor status

private:
	void CMP(uint8_t first, uint8_t second);

	void BIT(uint8_t data);
	void CPX(uint8_t data);
	void CPY(uint8_t data);
	void LDY(uint8_t data);
	void ORA(uint8_t data);
	void AND(uint8_t data);
	void EOR(uint8_t data);
	void ADC(uint8_t data);
	void LDA(uint8_t data);
	void CMP(uint8_t data);
	void SBC(uint8_t data);
	void LDX(uint8_t data);
	void BPL(int8_t data);
	void DEC(uint16_t offset);
	void INC(uint16_t offset);
	void BEQ(int8_t data);
	void BNE(int8_t data);
	void BCC(int8_t data);
	void BCS(int8_t data);
	void JSR();
	void RTS();
	void PHA();
	void PLA();
	void DEX();
	void DEY();
	void TXS();
	void INX();
	void INY();
	void CLC();
	void TAY();
	void ASL(uint16_t offset);
	void ROL(uint16_t offset);
	void LSR(uint16_t offset);
	void ROR(uint16_t offset);
	void BRK();

	uint8_t ASL(uint8_t data);
	uint8_t ROL(uint8_t data);
	uint8_t LSR(uint8_t data);
	uint8_t ROR(uint8_t data);

	// get/set memory

	uint8_t getByte(uint16_t offset);
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
		memory[FIRST_PAGE + --S] = value;
	}

	uint8_t popByte()
	{
		return memory[FIRST_PAGE + S++];
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

#ifdef _DEBUG
	std::map<uint8_t, int> instructionCounts;
#endif
};
