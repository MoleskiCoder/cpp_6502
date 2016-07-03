#include "stdafx.h"

#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

#include <Configuration.h>
#include <Controller.h>

int main() {

#ifdef SUDOKU
	Configuration configuration("C:\\github\\cs\\cs_6502\\sudoku.json");
#endif
#ifdef TEST_SUITE1
	Configuration configuration("C:\\github\\cs\\cs_6502\\test_suite_one.json");
#endif
#ifdef TEST_SUITE2
	Configuration configuration("C:\\github\\cs\\cs_6502\\test_suite_two.json");
#endif
#ifdef TEST_SUITE_65C02
	Configuration configuration("C:\\github\\cs\\cs_6502\\test_suite_65c02.json");
#endif
#ifdef EHBASIC
	Configuration configuration("C:\\github\\cs\\cs_6502\\ehbasic.json");
#endif
#ifdef TALI_FORTH
	Configuration configuration("C:\\github\\cs\\cs_6502\\tali.json");
#endif
#ifdef BBC_FORTH
	Configuration configuration("C:\\github\\cs\\cs_6502\\bbc_forth.json");
#endif

	Controller controller(configuration);

	controller.Configure();

	{
		boost::timer::auto_cpu_timer cpu_timer;
		controller.Start();
	}

	auto hertz = controller.speed * controller.processor->Mega;

	auto cycles = controller.processor->getCycles();
	auto heldCycles = controller.processor->getHeldCycles();

	auto start = controller.startTime;
	auto finish = controller.finishTime;

	auto elapsedTime = finish - start;
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsedTime).count();

	auto cyclesPerSecond = cycles / seconds;
	auto simulatedElapsed = cycles / hertz;
	auto speedup = cyclesPerSecond / hertz;

	auto cycleDifference = cycles - heldCycles;
	auto holdProportion = (double)cycles / cycleDifference;

	auto hostHertz = controller.hostSpeed * controller.processor->Mega;
	auto cyclesPerHostCycle = hostHertz / (cyclesPerSecond * holdProportion);

	std::cout << std::endl << "** Stopped PC=" << std::setw(4) << std::setfill('0') << controller.processor->getPC();

#ifdef TEST_SUITE1
	auto test = controller.processor->GetByte(0x0210);
	if (test == 0xff)
		std::cout << std::endl << "** success!!";
	else
		std::cout << std::endl << "** " << std::hex << (int)test << " failed!!";
#endif

#ifdef TEST_SUITE2
	auto test = controller.processor->GetByte(0x0200);
	std::cout << std::endl << "** Test=" << std::hex << (int)test;
#endif

	std::cout << std::endl << std::endl << "Time taken " << seconds << std::endl;
	std::cout << std::endl << std::endl << "Cycles per second " << cyclesPerSecond << std::endl;
	std::cout << std::endl << std::endl << "Speedup over " << controller.speed << "Mhz 6502 " << speedup << std::endl;
	std::cout << std::endl << std::endl << "Simulated cycles used " << cycles << std::endl;
	std::cout << std::endl << std::endl << "Held cycles " << heldCycles << std::endl;
	std::cout << std::endl << std::endl << "Held cycle difference " << cycleDifference << std::endl;
	std::cout << std::endl << std::endl << "Held proportion " << holdProportion << std::endl;
	std::cout << std::endl << std::endl << "Cycles per host cycle (code efficiency!) " << cyclesPerHostCycle << std::endl;
	std::cout << std::endl << std::endl << "Simulated time taken " << simulatedElapsed << std::endl;
}