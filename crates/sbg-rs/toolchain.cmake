# Configure the appropriate target for cross-compilation
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Compile targets as static libraries.
# Important for cross-compilation as shared libraries may not be supported.
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Set the appropriate cross-compiler binaries
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
