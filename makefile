SHELL = /bin/sh

.SUFFIXES:
.SUFFIXES: .cc .o

CXXFLAGS += -std=c++11 -O3 -march=native

all : cpp_6502

#Solver.o : Solver.cc

#SudokuGrid.o : SudokuGrid.cc

#sudoku : Solver.o SudokuGrid.o

#cpp_6502.o : cpp_6502.cc

mos6502.o : mos6502.cc

system6502.o : system6502.cc

#cpp_6502 : cpp_6502.o system6502.o mos6502.o
cpp_6502 : system6502.o mos6502.o

.PHONY : clean
clean : 
	rm -fv *.exe *.o
