#pragma once

#include <assert.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>

//#define TEST_SUITE1
//#define TEST_SUITE2
//#define EHBASIC
#define SUDOKU_ASSEMBLE

#ifdef EHBASIC
#define MEMORYMAP_CONSOLE_IO
#define MEMORYMAP_CONSOLE_I 0xf004
#define MEMORYMAP_CONSOLE_O 0xf001
#endif

#ifdef SUDOKU_ASSEMBLE
#define MEMORYMAP_CONSOLE_IO
#define MEMORYMAP_CONSOLE_I 0xe004
#define MEMORYMAP_CONSOLE_O 0xe001
#endif

#define COUNT_INSTRUCTIONS
#define PROFILE