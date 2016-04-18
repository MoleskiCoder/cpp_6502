#pragma once

#include <vector>
#include <map>

#define FIRST_PAGE 0x100

#define FETCH_ADDR_ZEROPAGE		fetchByte()
#define FETCH_ADDR_ABSOLUTE		fetchWord()
#define FETCH_ADDR_ZEROPAGEX	getWord(memory[fetchByte() + X])
#define FETCH_ADDR_ZEROPAGEY	getWord(memory[fetchByte()]) + Y
#define FETCH_ADDR_ABSOLUTEX	fetchWord() + X
#define FETCH_ADDR_ABSOLUTEY	fetchWord() + Y

#define ZEROPAGE	memory[FETCH_ADDR_ZEROPAGE]
#define ABSOLUTE	memory[FETCH_ADDR_ABSOLUTE]
#define ZEROPAGEX	memory[FETCH_ADDR_ZEROPAGEX]
#define ZEROPAGEY	memory[FETCH_ADDR_ZEROPAGEY]
#define ABSOLUTEX	memory[FETCH_ADDR_ABSOLUTEX]
#define ABSOLUTEY	memory[FETCH_ADDR_ABSOLUTEY]

class mos6502
{
public:
	mos6502();
	~mos6502();

	void readRom(std::string path, size_t offset);
	void clearMemory();
	void resetRegisters();

	void run(uint16_t offset);

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
	void JSR();
	void RTS();
	void PHA();
	void PLA();
	void DEX();
	void DEY();
	void TXS();
	void INX();
	void INY();

	uint8_t ASL(uint8_t data);

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

	uint8_t readByte_Immediate();
	uint8_t readByte_ZeroPage();
	uint8_t readByte_Absolute();
	uint8_t readByte_ZeroPageX();
	uint8_t readByte_ZeroPageY();
	uint8_t readByte_AbsoluteX();
	uint8_t readByte_AbsoluteY();

	void writeByte_ZeroPage(uint8_t value);
	void writeByte_Absolute(uint8_t value);
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

	//

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

#ifdef _DEBUG
	std::map<uint8_t, int> instructionCounts;
#endif
};
