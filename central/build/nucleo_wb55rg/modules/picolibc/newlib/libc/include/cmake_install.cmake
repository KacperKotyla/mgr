# Install script for directory: C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Zephyr-Kernel")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Users/Mis/.zephyr_ide/toolchains/zephyr-sdk-0.16.6/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/Mis/test123/central/build/nucleo_wb55rg/modules/picolibc/newlib/libc/include/sys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/Mis/test123/central/build/nucleo_wb55rg/modules/picolibc/newlib/libc/include/machine/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/Mis/test123/central/build/nucleo_wb55rg/modules/picolibc/newlib/libc/include/ssp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/Mis/test123/central/build/nucleo_wb55rg/modules/picolibc/newlib/libc/include/rpc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/Mis/test123/central/build/nucleo_wb55rg/modules/picolibc/newlib/libc/include/arpa/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/alloca.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/_ansi.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/argz.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/ar.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/assert.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/byteswap.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/cpio.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/ctype.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/devctl.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/dirent.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/elf.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/endian.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/envlock.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/envz.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/errno.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/fastmath.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/fcntl.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/fenv.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/fnmatch.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/getopt.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/glob.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/grp.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/iconv.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/ieeefp.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/inttypes.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/langinfo.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/libgen.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/limits.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/locale.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/malloc.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/math.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/memory.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/newlib.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/paths.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/picotls.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/pwd.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/regdef.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/regex.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/sched.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/search.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/setjmp.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/signal.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/spawn.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/stdint.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/stdlib.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/string.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/strings.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/_syslist.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/tar.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/termios.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/threads.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/time.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/unctrl.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/unistd.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/utime.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/utmp.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/wchar.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/wctype.h"
    "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/wordexp.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "C:/Users/Mis/test123/external/modules/lib/picolibc/newlib/libc/include/complex.h")
endif()

