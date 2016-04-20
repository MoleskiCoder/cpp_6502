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
	PC = 0;
	X = Y = A = P = 0;
	S = 0xff;
}

//

uint8_t mos6502::getByte(uint16_t offset)
{
	return memory[offset];
}

uint16_t mos6502::getWord(uint16_t offset)
{
	auto low = memory[offset];
	auto high = memory[offset + 1];
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

uint8_t mos6502::readByteArgument_Immediate()
{
	auto value = fetchByte();
#ifdef _DEBUG
	printf("#%02x", value);
#endif
	return value;
}

uint8_t mos6502::readByteArgument_ZeroPage()
{
	auto value = fetchByte();
#ifdef _DEBUG
	printf("$%02x", value);
#endif
	return memory[value];
}

int8_t mos6502::readByteArgument_ImmediateDisplacement()
{
	auto displacment = (int8_t)fetchByte();
#ifdef _DEBUG
	printf("$%04x", PC + displacment);
#endif
	return displacment;
}

uint8_t mos6502::readByteArgument_ZeroPageX()
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("$%02x,X", zp);
#endif
	return memory[lowByte(zp + X)];
}

uint8_t mos6502::readByteArgument_ZeroPageY()
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("%02x,Y", zp);
#endif
	return memory[lowByte(zp + Y)];
}

uint8_t mos6502::readByteArgument_AbsoluteX()
{
	auto base = fetchWord();
#ifdef _DEBUG
	printf("$%04x,X", base);
#endif
	return memory[base + X];
}

uint8_t mos6502::readByteArgument_AbsoluteY()
{
	auto base = fetchWord();
#ifdef _DEBUG
	printf("$%04x,Y", base);
#endif
	return memory[base + Y];
}

uint8_t mos6502::readByteArgument_Absolute()
{
	auto address = fetchWord();
#ifdef _DEBUG
	printf("$%04x", address);
#endif
	return memory[address];
}

uint8_t mos6502::readByteArgument_IndexedIndirectX()
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("($%02x,X)", zp);
#endif
	return memory[getWord(zp + X)];
}

uint8_t mos6502::readByteArgument_IndirectIndexedY()
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("($%02x),Y", zp);
#endif
	return memory[getWord(zp) + Y];
}

//

uint8_t mos6502::readByte_ZeroPage()
{
	return ZEROPAGE;
}

uint8_t mos6502::readByte_Absolute()
{
	return ABSOLUTE;
}

uint8_t mos6502::readByte_IndexedIndirectX()
{
	return INDEXEDINDIRECTX;
}

uint8_t mos6502::readByte_IndirectIndexedY()
{
	return INDIRECTINDEXEDY;
}

uint8_t mos6502::readByte_ZeroPageX()
{
	return ZEROPAGEX;
}

uint8_t mos6502::readByte_ZeroPageY()
{
	return ZEROPAGEY;
}

uint8_t mos6502::readByte_AbsoluteX()
{
	return ABSOLUTEX;
}

uint8_t mos6502::readByte_AbsoluteY()
{
	return ABSOLUTEY;
}

//

void mos6502::writeByte_ZeroPage(uint8_t value)
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("$%02x", zp);
#endif
	memory[zp] = value;
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	auto address = fetchWord();
#ifdef _DEBUG
	printf("$%04x", address);
#endif
	memory[address] = value;
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("($%02x,X)", zp);
#endif
	memory[getWord(lowByte(zp + X))] = value;
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("($%02x),Y", zp);
#endif
	memory[getWord(zp) + Y] = value;
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("$%02x,X", zp);
#endif
	memory[lowByte(zp + X)] = value;
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	auto zp = fetchByte();
#ifdef _DEBUG
	printf("$%02x,Y", zp);
#endif
	memory[lowByte(zp + Y)] = value;
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	auto base = fetchWord();
#ifdef _DEBUG
	printf("$%04x,X", base);
#endif
	memory[(uint16_t)(base + X)] = value;
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	auto base = fetchWord();
#ifdef _DEBUG
	printf("$%04x,Y", base);
#endif
	memory[(uint16_t)(base + Y)] = value;
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
	P &= ~(F_Z | F_V | F_N);

	if (A & data)
		P |= F_Z;

	if (data & F_V)
		P |= F_V;

	if (data & F_N)
		P |= F_N;
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
	uint8_t carry = (P & F_C ? 0 : 1);

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
	uint8_t carry = (P & F_C ? 0 : 1);
	uint16_t difference = A - data - carry;

	P &= ~(F_Z | F_V | F_N | F_C);

	if (!((uint8_t)difference))
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

void mos6502::LDX(uint8_t data)
{
	X = data;
	reflectFlags_ZeroNegative(X);
}

void mos6502::BPL(int8_t data)
{
	if (!(P & F_N))
		PC += data;
}

void mos6502::BEQ(int8_t data)
{
	if (P & F_Z)
		PC += data;
}

void mos6502::BNE(int8_t data)
{
	if (!(P & F_Z))
		PC += data;
}

void mos6502::BCC(int8_t data)
{
	if (!(P & F_C))
		PC += data;
}

void mos6502::BCS(int8_t data)
{
	if (P & F_C)
		PC += data;
}

void mos6502::DEC(uint16_t offset)
{
	int8_t content = memory[offset];
	memory[offset] = --content;
	reflectFlags_ZeroNegative(content);
}

void mos6502::DEX()
{
	reflectFlags_ZeroNegative(--X);
}

void mos6502::DEY()
{
	reflectFlags_ZeroNegative(--Y);
}

void mos6502::INC(uint16_t offset)
{
	int8_t content = memory[offset];
	memory[offset] = ++content;
	reflectFlags_ZeroNegative(content);
}

void mos6502::INX()
{
	reflectFlags_ZeroNegative(++X);
}

void mos6502::INY()
{
	reflectFlags_ZeroNegative(++Y);
}

void mos6502::JSR()
{
	auto destination = fetchWord();
#ifdef _DEBUG
	printf("$%04x", destination);
#endif
	pushWord(PC + 1);
	PC = destination;
}

void mos6502::RTS()
{
	PC = popWord();
}

void mos6502::PHA()
{
	pushByte(A);
}

void mos6502::PLA()
{
	A = popByte();
	reflectFlags_ZeroNegative(A);
}

void mos6502::TXS()
{
	S = X;
}

void mos6502::TAY()
{
	Y = A;
	reflectFlags_ZeroNegative(Y);
}

uint8_t mos6502::ASL(uint8_t data)
{
	P &= ~(F_N | F_Z | F_C);

	uint8_t result = data << 1;
	if (!result)
		P |= F_Z;
	else
		if ((uint8_t)result < 0)
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

void mos6502::ROL(uint16_t offset)
{
	auto contents = memory[offset];
	memory[offset] = ROL(contents);
}

void mos6502::CLC()
{
	P &= ~F_C;
}

void mos6502::BRK()
{
	pushWord(PC);
	P |= F_B;
	PC = getWord(0xfffe);
}

//

void mos6502::step()
{
#ifdef _DEBUG
	printf("\n");
	printf("PC=%04x:", PC);
	printf("P=%02x, A=%02x, X=%02x, Y=%02x, S=%02x	", P, A, X, Y, S);
#endif

	auto current = fetchByte();

#ifdef _DEBUG
	printf("%02x", current);
#endif

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
				DISS_PREFIX(BRK);
				BRK();
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(BPL);
				BPL(readByteArgument_ImmediateDisplacement());
				break;

			case 0b110:	 // CLC
				DISS_PREFIX(CLC);
				CLC();
				break;

			default:
				assert(false && "unknown BPL/CLC addressing mode");
			}
			break;

		case 0b001:	//	BIT
			switch (addressing_mode) {
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(BIT);
				BIT(readByteArgument_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(BIT);
				BIT(readByteArgument_Absolute());
				break;

			case 0b000: // JSR
				DUMP_DBYTE(PC);
				DISS_PREFIX(JSR);
				JSR();
				break;

			default:
				assert(false && "unknown BIT/JSR addressing mode");
			}
			break;

		case 0b010:
			switch (addressing_mode)
			{
			case 0b010: // PHA
				DISS_PREFIX(PHA);
				PHA();
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(JMP);
				PC = fetchWord();
				break;
			default:
				assert(false && "unknown PHA/JMP instruction");
			}
			break;

		case 0b011:
			switch (addressing_mode)
			{
			case 0b000: // RTS
				DISS_PREFIX(RTS);
				RTS();
				break;
			case 0b010:	// PLA
				DISS_PREFIX(PLA);
				PLA();
				break;
			case 0b011: // JMP (indirect)
				DUMP_DBYTE(PC);
				DISS_PREFIX(JMP);
				{
					auto address = fetchWord();
#ifdef _DEBUG
					printf("($%04x)", address);
#endif
					PC = getWord(memory[address]);
				}
				break;
			default:
				assert(false && "unknown RTS/PLA/JMP addressing_mode");
			}
			break;

		case 0b100:	//	DEY
			switch (addressing_mode)
			{
			case 0b010:	// Implied DEY
				DISS_PREFIX(DEY);
				DEY();
				break;

			case 0b100: // BCC rel
				DUMP_BYTE(PC);
				DISS_PREFIX(BCC);
				BCC(readByteArgument_ImmediateDisplacement());
				break;

			case 0b001: // STY zero page
				DUMP_BYTE(PC);
				DISS_PREFIX(STY);
				writeByte_ZeroPage(Y);
				break;
			case 0b011: // STY absolute
				DUMP_DBYTE(PC);
				DISS_PREFIX(STY);
				writeByte_Absolute(Y);
				break;
			case 0b101: // STY zero page, X
				DUMP_BYTE(PC);
				DISS_PREFIX(STY);
				writeByte_ZeroPageX(Y);
				break;

			case 0b110: // TAY
				DISS_PREFIX(TAY);
				TAY();
				break;

			default:
				assert(false && "unknown TEY/BCC/STY addressing mode");
			}
			break;

		case 0b101:
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDY);
				LDY(readByteArgument_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDY);
				LDY(readByteArgument_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDY);
				LDY(readByteArgument_Absolute());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDY);
				LDY(readByteArgument_ZeroPageX());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDY);
				LDY(readByteArgument_AbsoluteX());
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(BCS);
				BCS(readByteArgument_ImmediateDisplacement());
				break;

			default:
				assert(false && "unknown LDY/BCS addressing mode");
			}
			break;

		case 0b110:	//	CPY
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(CPY);
				CPY(readByteArgument_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(CPY);
				CPY(readByteArgument_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(CPY);
				CPY(readByteArgument_Absolute());
				break;

			case 0b100:	// BNE
				DUMP_BYTE(PC);
				DISS_PREFIX(BNE);
				BNE(readByteArgument_ImmediateDisplacement());
				break;

			case 0b010:	// INY
				DISS_PREFIX(INY);
				INY();
				break;

			default:
				assert(false && "unknown CPY/BNE/INY addressing mode");
			}
			break;

		case 0b111:	//	CPX
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(CPX);
				CPX(readByteArgument_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(CPX);
				CPX(readByteArgument_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(CPX);
				CPX(readByteArgument_Absolute());
				break;

			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(BEQ);
				BEQ(readByteArgument_ImmediateDisplacement());
				break;

			case 0b010:
				DISS_PREFIX(INX);
				INX();
				break;

			default:
				assert(false && "unknown CPX/BEQ/INX addressing mode");
			}
			break;

		default:
			assert(false && "unknown opcode in 00 classification");
		}
		break;

	case 0b01:

		switch (op_code) {

		case 0b000:	//	ORA
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ORA);
				ORA(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown ORA addressing mode");
			}
			break;

		case 0b001:	//	AND
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(AND);
				AND(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown AND addressing mode");
			}
			break;

		case 0b010:	//	EOR
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(EOR);
				EOR(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown EOR addressing mode");
			}
			break;

		case 0b011:	//	ADC
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ADC);
				EOR(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(ADC);
				ADC(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown ADC addressing mode");
			}
			break;

		case 0b100:	//	STA
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(STA);
				writeByte_IndexedIndirectX(A);
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(STA);
				writeByte_ZeroPage(A);
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(STA);
				writeByte_Absolute(A);
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(STA);
				writeByte_IndirectIndexedY(A);
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(STA);
				writeByte_ZeroPageX(A);
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(STA);
				writeByte_AbsoluteY(A);
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(STA);
				writeByte_AbsoluteX(A);
				break;
			default:
				assert(false && "unknown STA addressing mode");
			}
			break;

		case 0b101:	//	LDA
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDA);
				LDA(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown LDA addressing mode");
			}
			break;

		case 0b110:	//	CMP
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(CMP);
				CMP(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown CMP addressing mode");
			}
			break;

		case 0b111:	//	SBC
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_IndexedIndirectX());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_ZeroPage());
				break;
			case 0b010:
				DUMP_BYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_Immediate());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_Absolute());
				break;
			case 0b100:
				DUMP_BYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_IndirectIndexedY());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_ZeroPageX());
				break;
			case 0b110:
				DUMP_DBYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_AbsoluteY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(SBC);
				SBC(readByteArgument_AbsoluteX());
				break;
			default:
				assert(false && "unknown SBC addressing mode");
			}
			break;

		default:
			assert(false && "unknown opcode in 01 classification");
		}
		break;


	case 0b10:

		switch (op_code) {

		case 0b000:	//	ASL
			switch (addressing_mode)
			{
			case 0b010:	// ASL A
				DISS_PREFIX(ASL);
				A = ASL(A);
				break;
			default:
				assert(false && "unknown ASL addressing mode");
			}
			break;

		case 0b001:	//	ROL
			switch (addressing_mode)
			{
			case 0b010:	// ROL A
				DISS_PREFIX(ROL);
				A = ROL(A);
				break;
			case 0b111: // ROL abs,X
				DUMP_DBYTE(PC);
				DISS_PREFIX(ROL);
				ROL(FETCH_ADDR_ABSOLUTEX);
				break;

			default:
				assert(false && "unknown ROL addressing mode");
			}
			break;

		case 0b010:	//	LSR
			DISS_PREFIX(LSR);
			assert(false && "LSR not implemented");
			break;

		case 0b011:	//	ROR
			DISS_PREFIX(ROR);
			assert(false && "ROR not implemented");
			break;

		case 0b100:	//	STX
			switch (addressing_mode) {
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(STX);
				writeByte_ZeroPage(X);
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(STX);
				writeByte_Absolute(X);
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(STX);
				writeByte_ZeroPageY(X);
				break;

			case 0b110:	// TXS
				DISS_PREFIX(TXS);
				TXS();
				break;

			default:
				assert(false && "unknown STX/TXS addressing mode");
			}
			break;

		case 0b101:	//	LDX
			switch (addressing_mode) {
			case 0b000:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDX);
				LDX(readByteArgument_Immediate());
				break;
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDX);
				LDX(readByteArgument_ZeroPage());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDX);
				LDX(readByteArgument_Absolute());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(LDX);
				LDX(readByteArgument_ZeroPageY());
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(LDX);
				LDX(readByteArgument_AbsoluteY());
				break;
			default:
				assert(false && "unknown LDX addressing mode");
			}
			break;

		case 0b110:	//	DEC
			switch (addressing_mode) {
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(DEC);
				DEC(fetchByte());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(DEC);
				DEC(fetchWord());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(DEC);
				DEC(FETCH_ADDR_ZEROPAGEX);
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(DEC);
				DEC(FETCH_ADDR_ABSOLUTEX);
				break;

			case 0b010:	// DEX
				DISS_PREFIX(DEX);
				DEX();
				break;

			default:
				assert(false && "unknown DEC/DEX addressing mode");
			}
			break;

		case 0b111:	//	INC
			switch (addressing_mode) {
			case 0b001:
				DUMP_BYTE(PC);
				DISS_PREFIX(INC);
				INC(fetchByte());
				break;
			case 0b011:
				DUMP_DBYTE(PC);
				DISS_PREFIX(INC);
				INC(fetchWord());
				break;
			case 0b101:
				DUMP_BYTE(PC);
				DISS_PREFIX(INC);
				INC(FETCH_ADDR_ZEROPAGEX);
				break;
			case 0b111:
				DUMP_DBYTE(PC);
				DISS_PREFIX(INC);
				INC(FETCH_ADDR_ABSOLUTEX);
				break;
			default:
				assert(false && "unknown INC addressing mode");
			}
			break;

		default:
			assert(false && "unknown opcode in 10 classification");
		}
		break;

	default:
		assert(false && "unknown classification");
	}
}

void mos6502::run() {

	for (;;)
	{
		step();
	}
}
