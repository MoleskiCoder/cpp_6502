#pragma once

#include <array>
#include <map>
#include <cstdint>
#include <functional>

#include <system6502.h>

#include "Disassembly.h"
#include "Symbols.h"

#include "ProfileLineEventArgs.h"
#include "ProfileScopeEventArgs.h"

class Profiler
{
public:
	std::array<uint64_t, 0x100> instructionCounts;
	std::array<uint64_t, 0x10000> addressProfiles;
	std::array<uint64_t, 0x10000> addressCounts;

	std::array<std::string, 0x10000> addressScopes;
	std::map<std::string, uint64_t> scopeCycles;

	System6502& processor;
	const Disassembly& disassembler;
	const Symbols& symbols;

	bool countInstructions;
	bool profileAddresses;

	uint64_t priorCycleCount = 0;

	Profiler(System6502& processor, Disassembly& disassembler, Symbols& symbols, bool countInstructions, bool profileAddresses);

	typedef std::function<void(const ProfileScopeEventArgs&)> profile_scope_event_f;
	typedef std::function<void(const ProfileLineEventArgs&)> profile_line_event_f;
	typedef std::function<void()> void_f;

	typedef std::vector<profile_scope_event_f> profile_scope_signal_t;
	typedef std::vector<profile_line_event_f> profile_line_signal_t;
	typedef std::vector<void_f> void_signal_t;

	void_signal_t StartingOutput;
	void_signal_t FinishedOutput;

	void_signal_t StartingLineOutput;
	void_signal_t FinishedLineOutput;

	profile_line_signal_t EmitLine;

	void_signal_t StartingScopeOutput;
	void_signal_t FinishedScopeOutput;

	profile_scope_signal_t EmitScope;

	void Generate();

private:

	static void FireDelegates(const profile_scope_signal_t& delegates, const ProfileScopeEventArgs& e) {
		if (!delegates.empty())
			for (auto& delegate : delegates)
				delegate(e);
	}

	static void FireDelegates(const profile_line_signal_t& delegates, const ProfileLineEventArgs& e) {
		if (!delegates.empty())
			for (auto& delegate : delegates)
				delegate(e);
	}

	static void FireDelegates(const void_signal_t& delegates) {
		if (!delegates.empty())
			for (auto& delegate : delegates)
				delegate();
	}

	void EmitProfileInformation();

	void Processor_ExecutingInstruction(const AddressEventArgs& addressEvent);
	void Processor_ExecutedInstruction(const AddressEventArgs& addressEvent);

	void BuildAddressScopes();
};
