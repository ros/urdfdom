# Check the OS type.

# CMake does not distinguish Linux from other Unices.
STRING (REGEX MATCH "Linux" URDF_OS_LINUX ${CMAKE_SYSTEM_NAME})
# Nor *BSD
STRING (REGEX MATCH "BSD" URDF_OS_BSD ${CMAKE_SYSTEM_NAME})
# Or Solaris. I'm seeing a trend, here
STRING (REGEX MATCH "SunOS" URDF_OS_SOLARIS ${CMAKE_SYSTEM_NAME})

# Windows is easy (for once)
IF (WIN32)
    SET (URDF_OS_WIN TRUE BOOL INTERNAL)
ENDIF (WIN32)

# Check if it's an Apple OS
IF (APPLE)
    # Check if it's OS X or another MacOS (that's got to be pretty unlikely)
    STRING (REGEX MATCH "Darwin" URDF_OS_OSX ${CMAKE_SYSTEM_NAME})
    IF (NOT URDF_OS_OSX)
        SET (URDF_OS_MACOS TRUE BOOL INTERNAL)
    ENDIF (NOT URDF_OS_OSX)
ENDIF (APPLE)

# QNX
IF (QNXNTO)
    SET (URDF_OS_QNX TRUE BOOL INTERNAL)
ENDIF (QNXNTO)

IF (URDF_OS_LINUX)
    MESSAGE (STATUS "Operating system is Linux")
ELSEIF (URDF_OS_BSD)
    MESSAGE (STATUS "Operating system is BSD")
ELSEIF (URDF_OS_WIN)
    MESSAGE (STATUS "Operating system is Windows")
ELSEIF (URDF_OS_OSX)
    MESSAGE (STATUS "Operating system is Apple MacOS X")
ELSEIF (URDF_OS_MACOS)
    MESSAGE (STATUS "Operating system is Apple MacOS (not OS X)")
ELSEIF (URDF_OS_QNX)
    MESSAGE (STATUS "Operating system is QNX")
ELSEIF (URDF_OS_SOLARIS)
    MESSAGE (STATUS "Operating system is Solaris")
ELSE (URDF_OS_LINUX)
    MESSAGE (STATUS "Operating system is generic Unix")
ENDIF (URDF_OS_LINUX)
