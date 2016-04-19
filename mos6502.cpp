#include "stdafx.h"
#include "mos6502.h"

#include <fstream>
#include <iterator>
#include <algorithm>
#include <assert.h>

mos6502::mos6502()
:	memory(0xffff)
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

	std::copy(buffer.begin(), buffer.end() - 1, memory.begin() + offset);
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
	auto value = readByte_Immediate();
#ifdef _DEBUG
	printf("#%02x", value);
#endif
	return value;
}

int8_t mos6502::readByteArgument_ImmediateDisplacement()
{
	auto displacment = (int8_t)readByte_Immediate();
#ifdef _DEBUG
	printf("%d", displacment);
#endif
	return displacment;
}

uint8_t mos6502::readByteArgument_AbsoluteX()
{
	auto base = FETCH_ADDR_ABSOLUTE;
#ifdef _DEBUG
	printf("$%04x,X", base);
#endif
	return memory[base + X];
}

uint8_t mos6502::readByteArgument_AbsoluteY()
{
	auto base = FETCH_ADDR_ABSOLUTE;
#ifdef _DEBUG
	printf("$%04x,Y", base);
#endif
	return memory[base + Y];
}

uint8_t mos6502::readByteArgument_Absolute()
{
	auto address = FETCH_ADDR_ABSOLUTE;
#ifdef _DEBUG
	printf("$%04x", address);
#endif
	return memory[address];
}

//

uint8_t mos6502::readByte_Immediate()
{
	return fetchByte();
}

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
	auto zp = FETCH_ADDR_ZEROPAGE;
#ifdef _DEBUG
	printf("$%02x", zp);
#endif
	memory[zp] = value;
}

void mos6502::writeByte_Absolute(uint8_t value)
{
	ABSOLUTE = value;
}

void mos6502::writeByte_IndexedIndirectX(uint8_t value)
{
	INDEXEDINDIRECTX = value;
}

void mos6502::writeByte_IndirectIndexedY(uint8_t value)
{
	INDIRECTINDEXEDY = value;
}

void mos6502::writeByte_ZeroPageX(uint8_t value)
{
	ZEROPAGEX = value;
}

void mos6502::writeByte_ZeroPageY(uint8_t value)
{
	ZEROPAGEY = value;
}

void mos6502::writeByte_AbsoluteX(uint8_t value)
{
	ABSOLUTEX = value;
}

void mos6502::writeByte_AbsoluteY(uint8_t value)
{
	auto base = fetchWord();
#ifdef _DEBUG
	printf("$%04x,Y", base);
#endif
	auto address = (uint16_t)(base + Y);
	memory[address] = value;
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
	printf("JSR %04x", destination);
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

//

void mos6502::run(uint16_t offset) {

	auto class_mask = 0b11;
	auto addressing_mode_mask = 0b11100;
	auto op_code_mask = 0b11100000;

	PC = offset;

	for (;;)
	{
		auto current = fetchByte();

#ifdef _DEBUG
		instructionCounts[current]++;
#endif

		auto classification = current & class_mask;
		auto addressing_mode = (current & addressing_mode_mask) >> 2;
		auto op_code = (current & op_code_mask) >> 5;

#ifdef _DEBUG
		printf("\nINS=%02x: PC=%04x, A=%02x, X=%02x, Y=%02x, S=%02x	", current, PC - 1, A, X, Y, S);
#endif

		switch (classification)
		{
		case 0b00:
			/*
			000	#immediate
			001	zero page
			011	absolute
			101	zero page,X
			111	absolute,X
			*/
			switch (op_code) {

			case 0b000:	// BPL
				switch (addressing_mode) {
				case 0b100:
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
					DISS_PREFIX(BIT);
					BIT(readByte_ZeroPage());
					break;
				case 0b011:
					DISS_PREFIX(BIT);
					BIT(readByteArgument_Absolute());
					break;

				case 0b000: // JSR
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
					DISS_PREFIX(JMP);
					PC = ABSOLUTE;
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
					DISS_PREFIX(BCC);
					BCC(readByteArgument_ImmediateDisplacement());
					break;

				case 0b001: // STY zero page
					DISS_PREFIX(STY);
					writeByte_ZeroPage(Y);
					break;
				case 0b011: // STY absolute
					DISS_PREFIX(STY);
					writeByte_Absolute(Y);
					break;
				case 0b101: // STY zero page, X
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
					DISS_PREFIX(LDY);
					LDY(readByteArgument_Immediate());
					break;
				case 0b001:
					DISS_PREFIX(LDY);
					LDY(readByte_ZeroPage());
					break;
				case 0b011:
					DISS_PREFIX(LDY);
					LDY(readByteArgument_Absolute());
					break;
				case 0b101:
					DISS_PREFIX(LDY);
					LDY(readByte_ZeroPageX());
					break;
				case 0b111:
					DISS_PREFIX(LDY);
					LDY(readByteArgument_AbsoluteX());
					break;

				case 0b100:
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
					DISS_PREFIX(CPY);
					CPY(readByteArgument_Immediate());
					break;
				case 0b001:
					DISS_PREFIX(CPY);
					CPY(readByte_ZeroPage());
					break;
				case 0b011:
					DISS_PREFIX(CPY);
					CPY(readByteArgument_Absolute());
					break;

				case 0b100:	// BNE
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
					DISS_PREFIX(CPX);
					CPX(readByteArgument_Immediate());
					break;
				case 0b001:
					DISS_PREFIX(CPX);
					CPX(readByte_ZeroPage());
					break;
				case 0b011:
					DISS_PREFIX(CPX);
					CPX(readByteArgument_Absolute());
					break;

				case 0b100:
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

			/*
			000	(zero page, X)
			001	zero page
			010	#immediate
			011	absolute
			100	(zero page), Y
			101	zero page, X
			110	absolute, Y
			111	absolute, X
			*/

			switch (op_code) {

			case 0b000:	//	ORA
				DISS_PREFIX(ORA);
				switch (addressing_mode) {
				case 0b000:
					ORA(readByte_IndexedIndirectX());
					break;
				case 0b001:
					ORA(readByte_ZeroPage());
					break;
				case 0b010:
					ORA(readByteArgument_Immediate());
					break;
				case 0b011:
					ORA(readByteArgument_Absolute());
					break;
				case 0b100:
					ORA(readByte_IndirectIndexedY());
					break;
				case 0b101:
					ORA(readByte_ZeroPageX());
					break;
				case 0b110:
					ORA(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					ORA(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown ORA addressing mode");
				}
				break;

			case 0b001:	//	AND
				DISS_PREFIX(AND);
				switch (addressing_mode) {
				case 0b000:
					AND(readByte_IndexedIndirectX());
					break;
				case 0b001:
					AND(readByte_ZeroPage());
					break;
				case 0b010:
					AND(readByteArgument_Immediate());
					break;
				case 0b011:
					AND(readByteArgument_Absolute());
					break;
				case 0b100:
					AND(readByte_IndirectIndexedY());
					break;
				case 0b101:
					AND(readByte_ZeroPageX());
					break;
				case 0b110:
					AND(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					AND(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown AND addressing mode");
				}
				break;

			case 0b010:	//	EOR
				DISS_PREFIX(EOR);
				switch (addressing_mode) {
				case 0b000:
					EOR(readByte_IndexedIndirectX());
					break;
				case 0b001:
					EOR(readByte_ZeroPage());
					break;
				case 0b010:
					EOR(readByteArgument_Immediate());
					break;
				case 0b011:
					EOR(readByteArgument_Absolute());
					break;
				case 0b100:
					EOR(readByte_IndirectIndexedY());
					break;
				case 0b101:
					EOR(readByte_ZeroPageX());
					break;
				case 0b110:
					EOR(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					EOR(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown EOR addressing mode");
				}
				break;

			case 0b011:	//	ADC
				DISS_PREFIX(ADC);
				switch (addressing_mode) {
				case 0b000:
					ADC(readByte_IndexedIndirectX());
					break;
				case 0b001:
					ADC(readByte_ZeroPage());
					break;
				case 0b010:
					ADC(readByteArgument_Immediate());
					break;
				case 0b011:
					ADC(readByteArgument_Absolute());
					break;
				case 0b100:
					ADC(readByte_IndirectIndexedY());
					break;
				case 0b101:
					ADC(readByte_ZeroPageX());
					break;
				case 0b110:
					EOR(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					ADC(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown ADC addressing mode");
				}
				break;

			case 0b100:	//	STA
				DISS_PREFIX(STA);
				switch (addressing_mode) {
				case 0b000:
					writeByte_IndexedIndirectX(A);
					break;
				case 0b001:
					writeByte_ZeroPage(A);
					break;
				case 0b011:
					writeByte_Absolute(A);
					break;
				case 0b100:
					writeByte_IndirectIndexedY(A);
					break;
				case 0b101:
					writeByte_ZeroPageX(A);
					break;
				case 0b110:
					writeByte_AbsoluteY(A);
					break;
				case 0b111:
					writeByte_AbsoluteX(A);
					break;
				default:
					assert(false && "unknown STA addressing mode");
				}
				break;

			case 0b101:	//	LDA
				DISS_PREFIX(LDA);
				switch (addressing_mode) {
				case 0b000:
					LDA(readByte_IndexedIndirectX());
					break;
				case 0b001:
					LDA(readByte_ZeroPage());
					break;
				case 0b010:
					LDA(readByteArgument_Immediate());
					break;
				case 0b011:
					LDA(readByteArgument_Absolute());
					break;
				case 0b100:
					LDA(readByte_IndirectIndexedY());
					break;
				case 0b101:
					LDA(readByte_ZeroPageX());
					break;
				case 0b110:
					LDA(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					LDA(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown LDA addressing mode");
				}
				break;

			case 0b110:	//	CMP
				DISS_PREFIX(CMP);
				switch (addressing_mode) {
				case 0b000:
					CMP(readByte_IndexedIndirectX());
					break;
				case 0b001:
					CMP(readByte_ZeroPage());
					break;
				case 0b010:
					CMP(readByteArgument_Immediate());
					break;
				case 0b011:
					CMP(readByteArgument_Absolute());
					break;
				case 0b100:
					CMP(readByte_IndirectIndexedY());
					break;
				case 0b101:
					CMP(readByte_ZeroPageX());
					break;
				case 0b110:
					CMP(readByteArgument_AbsoluteY());
					break;
				case 0b111:
					CMP(readByteArgument_AbsoluteX());
					break;
				default:
					assert(false && "unknown CMP addressing mode");
				}
				break;

			case 0b111:	//	SBC
				DISS_PREFIX(SBC);
				switch (addressing_mode) {
				case 0b000:
					SBC(readByte_IndexedIndirectX());
					break;
				case 0b001:
					SBC(readByte_ZeroPage());
					break;
				case 0b010:
					SBC(readByteArgument_Immediate());
					break;
				case 0b011:
					SBC(readByteArgument_Absolute());
					break;
				case 0b100:
					SBC(readByte_IndirectIndexedY());
					break;
				case 0b101:
					SBC(readByte_ZeroPageX());
					break;
				case 0b110:
					SBC(readByteArgument_AbsoluteY());
					break;
				case 0b111:
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
			/*
			000	#immediate
			001	zero page
			010	accumulator
			011	absolute
			101	zero page,X
			111	absolute,X
			*/
			switch (op_code) {

			case 0b000:	//	ASL
				DISS_PREFIX(ASL);
				switch (addressing_mode)
				{
				case 0b010:	// ASL A
					A = ASL(A);
					break;
				default:
					assert(false && "unknown ASL addressing mode");
				}
				break;

			case 0b001:	//	ROL
				DISS_PREFIX(ROL);
				switch (addressing_mode)
				{
				case 0b010:	// ROL A
					A = ROL(A);
					break;
				case 0b111: // ROL abs,X
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
					DISS_PREFIX(STX);
					writeByte_ZeroPage(X);
					break;
				case 0b011:
					DISS_PREFIX(STX);
					writeByte_Absolute(X);
					break;
				case 0b101:
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
				DISS_PREFIX(LDX);
				switch (addressing_mode) {
				case 0b000:
					LDX(readByteArgument_Immediate());
					break;
				case 0b001:
					LDX(readByte_ZeroPage());
					break;
				case 0b011:
					LDX(readByteArgument_Absolute());
					break;
				case 0b101:
					LDX(readByte_ZeroPageY());
					break;
				case 0b111:
					LDX(readByteArgument_AbsoluteY());
					break;
				default:
					assert(false && "unknown LDX addressing mode");
				}
				break;

			case 0b110:	//	DEC
				switch (addressing_mode) {
				case 0b001:
					DISS_PREFIX(DEC);
					DEC(FETCH_ADDR_ZEROPAGE);
					break;
				case 0b011:
					DISS_PREFIX(DEC);
					DEC(FETCH_ADDR_ABSOLUTE);
					break;
				case 0b101:
					DISS_PREFIX(DEC);
					DEC(FETCH_ADDR_ZEROPAGEX);
					break;
				case 0b111:
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
				DISS_PREFIX(INC);
				switch (addressing_mode) {
				case 0b001:
					INC(FETCH_ADDR_ZEROPAGE);
					break;
				case 0b011:
					INC(FETCH_ADDR_ABSOLUTE);
					break;
				case 0b101:
					INC(FETCH_ADDR_ZEROPAGEX);
					break;
				case 0b111:
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
}
