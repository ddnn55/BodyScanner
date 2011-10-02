# - Try to find GMP
# Once done, this will define
#
#  GMP_FOUND - system has GMP
#  GMP_INCLUDE_DIRS - the GMP include directories
#  GMP_LIBRARIES - link these to use GMP

IF( GMP_FOUND )
   # in cache already
   SET( GMP_FIND_QUIETLY TRUE )
ENDIF()

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(GMP_PKGCONF GMP)

# Include dir
find_path(gmp_INCLUDE_DIR
  NAMES gmp.h
  PATHS ${GMP_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(gmpxx_LIBRARY
  NAMES gmpxx
  PATHS ${GMP_PKGCONF_LIBRARY_DIRS}
)

find_library(gmp_LIBRARY
  NAMES gmp
  PATHS ${GMP_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(GMP_PROCESS_INCLUDES gmp_INCLUDE_DIR)
set(GMP_PROCESS_LIBS gmp_LIBRARY gmpxx_LIBRARY)
libfind_process(GMP)
