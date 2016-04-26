#pragma once

#include <assert.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>

//#define TEST_SUITE1
//#define TEST_SUITE2
#define EHBASIC

#ifdef EHBASIC
#define MEMORYMAP_CONSOLE_IO
#define MEMORYMAP_CONSOLE_I 0xF004
#define MEMORYMAP_CONSOLE_O 0xF001
#endif