#include "stdafx.h"
#include "Profiler.h"

using namespace std::placeholders;

Profiler::Profiler(System6502& targetProcessor, Disassembly& disassemblerTarget, Symbols& symbolsTarget, bool instructions, bool addresses)
:	processor(targetProcessor),
	disassembler(disassemblerTarget),
	symbols(symbolsTarget),
	countInstructions(instructions),
	profileAddresses(addresses)
{
	instructionCounts.fill(0);
	addressProfiles.fill(0);
	addressCounts.fill(0);

	if (countInstructions || profileAddresses)
		processor.ExecutingInstruction.push_back(std::bind(&Profiler::Processor_ExecutingInstruction, this, _1));
	if (profileAddresses)
		processor.ExecutedInstruction.push_back(std::bind(&Profiler::Processor_ExecutedInstruction, this, _1));

	BuildAddressScopes();
}

void Profiler::Generate() {
	FireDelegates(StartingOutput);
	EmitProfileInformation();
	FireDelegates(StartingOutput);
}

void Profiler::EmitProfileInformation() {

	{
		FireDelegates(StartingLineOutput);
		// For each memory address
		for (auto address = 0; address < 0x10000; ++address) {
			// If there are any cycles associated
			auto cycles = addressProfiles[address];
			if (cycles > 0) {
				// Dump a profile/disassembly line
				auto source = disassembler.Disassemble(address);
				FireDelegates(EmitLine, ProfileLineEventArgs(source, cycles));
			}
		}
		FireDelegates(FinishedLineOutput);
	}

	{
		FireDelegates(StartingScopeOutput);
		for (auto& scopeCycle : scopeCycles) {
			auto name = scopeCycle.first;
			auto cycles = scopeCycle.second;
			auto namedAddress = (size_t)symbols.getAddresses().find(name)->second;
			auto count = addressCounts[namedAddress];
			FireDelegates(EmitScope, ProfileScopeEventArgs(name, cycles, count));
		}
		FireDelegates(FinishedScopeOutput);
	}
}

void Profiler::Processor_ExecutingInstruction(const AddressEventArgs& addressEvent) {
	if (profileAddresses) {
		priorCycleCount = processor.getCycles();
		addressCounts[addressEvent.getAddress()]++;
	}
	if (countInstructions)
		++instructionCounts[addressEvent.getCell()];
}

void Profiler::Processor_ExecutedInstruction(const AddressEventArgs& addressEvent) {
	if (profileAddresses) {
		auto cycles = processor.getCycles() - priorCycleCount;
		addressProfiles[addressEvent.getAddress()] += cycles;
		auto addressScope = addressScopes[addressEvent.getAddress()];
		if (!addressScope.empty()) {
			if (scopeCycles.find(addressScope) == scopeCycles.end())
				scopeCycles[addressScope] = 0;
			scopeCycles[addressScope] += cycles;
		}
	}
}

void Profiler::BuildAddressScopes() {
	for (auto& label : symbols.getLabels()) {
		auto address = label.first;
		auto key = label.second;
		auto scope = symbols.getScopes().find(key);
		if (scope != symbols.getScopes().end()) {
			for (uint16_t i = address; i < address + scope->second; ++i) {
				addressScopes[i] = key;
			}
		}
	}
}