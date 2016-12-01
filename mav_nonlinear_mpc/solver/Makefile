#
#    This file was auto-generated using the ACADO Toolkit.
#    
#    While ACADO Toolkit is free software released under the terms of
#    the GNU Lesser General Public License (LGPL), the generated code
#    as such remains the property of the user who used ACADO Toolkit
#    to generate this code. In particular, user dependent data of the code
#    do not inherit the GNU LGPL license. On the other hand, parts of the
#    generated code that are a direct copy of source code from the
#    ACADO Toolkit or the software tools it is based on, remain, as derived
#    work, automatically covered by the LGPL license.
#    
#    ACADO Toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#    

UNAME := $(shell uname)

LDLIBS = -lm -lstdc++
ifeq ($(UNAME), Linux)
	LDLIBS += -lrt
endif

CCACHE_APP := $(shell which ccache 2>/dev/null)

CFLAGS = -O3 -finline-functions -I. -I./qpoases
CXXFLAGS = -O3 -finline-functions -I. -I./qpoases -I./qpoases/INCLUDE -I./qpoases/SRC
CC     = $(CCACHE_APP) gcc
CXX    = $(CCACHE_APP) g++

OBJECTS = \
	./qpoases/SRC/Bounds.o \
	./qpoases/SRC/Constraints.o \
	./qpoases/SRC/CyclingManager.o \
	./qpoases/SRC/Indexlist.o \
	./qpoases/SRC/MessageHandling.o \
	./qpoases/SRC/QProblem.o \
	./qpoases/SRC/QProblemB.o \
	./qpoases/SRC/SubjectTo.o \
	./qpoases/SRC/Utils.o \
	./qpoases/SRC/EXTRAS/SolutionAnalysis.o \
	@MODULE_NAME@_qpoases_interface.o \
	@MODULE_NAME@_integrator.o \
	@MODULE_NAME@_solver.o \
	@MODULE_NAME@_auxiliary_functions.o

.PHONY: all
all: lib@MODULE_NAME@_exported_rti.a test

test: ${OBJECTS} test.o

@MODULE_NAME@_qpoases_interface.o   : @MODULE_NAME@_qpoases_interface.hpp
@MODULE_NAME@_solver.o              : @MODULE_NAME@_common.h
@MODULE_NAME@_integrator.o          : @MODULE_NAME@_common.h
@MODULE_NAME@_auxiliary_functions.o : @MODULE_NAME@_common.h \
                              @MODULE_NAME@_auxiliary_functions.h
test.o                      : @MODULE_NAME@_common.h \
                              @MODULE_NAME@_qpoases_interface.hpp \
                              @MODULE_NAME@_auxiliary_functions.h

lib@MODULE_NAME@_exported_rti.a: ${OBJECTS}
	ar r $@ $?

${OBJECTS} : @MODULE_NAME@_qpoases_interface.hpp

.PHONY : clean
clean :
	-rm -f *.o *.a ./qpoases/*.o ./qpoases/SRC/*.o ./qpoases/SRC/*.a test