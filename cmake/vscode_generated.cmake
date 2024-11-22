# This is converter generated file, and shall not be touched by user
#
# Use CMakeLists.txt to apply user changes
cmake_minimum_required(VERSION 3.22)

# Core MCU flags, CPU, instruction set and FPU setup
set(cpu_PARAMS ${cpu_PARAMS}
    -mthumb

    # Other parameters
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m7
	-mfpu=fpv5-sp-d16
	-mfloat-abi=hard
)

# Linker script
set(linker_script_SRC ${linker_script_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f756zgtx_FLASH.ld
)

# Sources
set(sources_SRCS ${sources_SRCS}

)

# Include directories
set(include_c_DIRS ${include_c_DIRS}

)
set(include_cxx_DIRS ${include_cxx_DIRS}
    
)
set(include_asm_DIRS ${include_asm_DIRS}
    
)

# Symbols definition
set(symbols_c_SYMB ${symbols_c_SYMB}

)
set(symbols_cxx_SYMB ${symbols_cxx_SYMB}
    
)
set(symbols_asm_SYMB ${symbols_asm_SYMB}
    
)

# Link directories
set(link_DIRS ${link_DIRS}
    
)

# Link libraries
set(link_LIBS ${link_LIBS}
    
)

# Compiler options
set(compiler_OPTS ${compiler_OPTS})

# Linker options
set(linker_OPTS ${linker_OPTS})
