
# - Finds Openblas and all dependencies
# Once done this will define
#  Openblas_FOUND - System has Openblas
#  Openblas_INCLUDE_DIRS - The Openblas include directories
#  Openblas_LIBRARIES - The libraries needed to use Openblas

find_library(Openblas_LIBRARY
  NAMES libopenblas.so libopenblas.lib libopenblas.dylib
  PATHS ${Openblas_ROOT}/lib
    /usr/lib/openblas-base
    /usr/local/lib
)

# brew installed openblas library on os x is hard to find and depends on the installed version.
if(Openblas_LIBRARY STREQUAL "Openblas_LIBRARY-NOTFOUND")
  file(GLOB Openblas_LIBRARY /usr/local/Cellar/openblas/*/lib/libopenblas.dylib)
endif()

if(NOT OPENBLAS_IGNORE_HEADERS)
  find_path(Openblas_INCLUDE_DIR
    NAMES openblas_config.h
    PATHS ${Openblas_ROOT}/include
      /usr/include
      /usr/local/include
      /usr/local/Cellar
      /usr/local/opt/openblas/include
)
endif()

if((Openblas_LIBRARY STREQUAL "Openblas_LIBRARY-NOTFOUND") OR (Openblas_INCLUDE_DIR STREQUAL "Openblas_INCLUDE_DIR-NOTFOUND"))
  set(Openblas_ROOT "" CACHE PATH "Path to the root of a Openblas installation")
  set(Openblas_FOUND 0)
  message(WARNING "Openblas not found. Please try specifying Openblas_ROOT")
else()
  set(Openblas_FOUND 1)
  set(Openblas_INCLUDE_DIRS ${Openblas_INCLUDE_DIR})
  set(Openblas_LIBRARIES ${Openblas_LIBRARY})
  if(CMAKE_COMPILER_IS_GNUCC)
    set(Openblas_LIBRARIES ${Openblas_LIBRARIES} gfortran pthread)
  endif()
endif()
