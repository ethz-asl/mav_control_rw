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
	acado_qpoases_interface.o \
	acado_integrator.o \
	acado_solver.o \
	acado_auxiliary_functions.o

.PHONY: all
all: libacado_exported_rti.a test

test: ${OBJECTS} test.o

acado_qpoases_interface.o   : acado_qpoases_interface.hpp
acado_solver.o              : acado_common.h
acado_integrator.o          : acado_common.h
acado_auxiliary_functions.o : acado_common.h \
                              acado_auxiliary_functions.h
test.o                      : acado_common.h \
                              acado_qpoases_interface.hpp \
                              acado_auxiliary_functions.h

libacado_exported_rti.a: ${OBJECTS}
	ar r $@ $?

${OBJECTS} : acado_qpoases_interface.hpp

.PHONY : clean
clean :
	-rm -f *.o *.a ./qpoases/*.o ./qpoases/SRC/*.o ./qpoases/SRC/*.a test
