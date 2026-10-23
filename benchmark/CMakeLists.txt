cmake_minimum_required(VERSION 3.28)
project(lib_bench C CXX)

# 抑制 Conan Preset 模式下 CMAKE_TOOLCHAIN_FILE 的"未使用变量"警告
# （toolchain 已由 CMakePresets.json include() 加载，此变量仅供 cmake 内部使用）
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" CACHE STRING "" FORCE)


# ──────────────── 从 Conan 传入的变量 ────────────────
#   MCU_C_FLAGS          : CPU 相关编译选项（分号分隔列表）
#   ALGO_FLASH_ORIGIN    : 算法 Slot Flash 起始地址（仅 baremetal）
#   ALGO_RAM_ORIGIN      : 算法 Slot RAM 起始地址（仅 baremetal）
#   LIB_NAME             : 外部库名称
#   TARGET_MCU           : 目标 CPU 型号

message(STATUS "========================================")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "  Linux/A-core Benchmark Build")
    message(STATUS "  Target CPU     : ${TARGET_MCU}")
    message(STATUS "  System         : ${CMAKE_SYSTEM_NAME}")
    message(STATUS "  CPU Flags      : (injected via conan_toolchain.cmake)")
else()
    message(STATUS "  MCU Benchmark Build")
    message(STATUS "  Target CPU     : ${TARGET_MCU}")
    message(STATUS "  System         : ${CMAKE_SYSTEM_NAME}")
    message(STATUS "  Flash Origin   : ${ALGO_FLASH_ORIGIN}")
    message(STATUS "  RAM Origin     : ${ALGO_RAM_ORIGIN}")
    message(STATUS "  CPU Flags      : ${MCU_C_FLAGS}")
endif()
message(STATUS "========================================")


# ──────────────── 查找外部库 ────────────────
find_package(${LIB_NAME} REQUIRED)

# ──────────────── 公共源文件 ────────────────
set(BENCH_SOURCES
    bench_entry.cpp
    core/het_bench_core.c
)


# ════════════════════════════════════════════════════════════════
#  分支 A：A 核 / Linux（带操作系统，标准可执行文件）
# ════════════════════════════════════════════════════════════════
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")

    add_executable(benchmark
        ${BENCH_SOURCES}
        port/linux/bench_main_linux.cpp
    )

    target_link_libraries(benchmark PRIVATE
        ${LIB_NAME}::${LIB_NAME}_cpp
        m                                  # libm（clock_gettime 等）
    )

    target_include_directories(benchmark PRIVATE
        ${${LIB_NAME}_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Linux 交叉编译标志（-mfloat-abi / -mfpu 等）已由 Conan CMakeToolchain
    # 通过 tools.build:cflags 注入 conan_toolchain.cmake，无需在此手动传递。

    target_compile_options(benchmark PRIVATE
        -O2
        -ffunction-sections
        -fdata-sections
        -Wall
        -Wextra
    )

    target_link_options(benchmark PRIVATE
        -Wl,--gc-sections
    )

    # Size 信息
    find_program(_SIZE_TOOL NAMES size
        arm-none-linux-gnueabihf-size
        aarch64-none-linux-gnu-size)
    if(_SIZE_TOOL)
        add_custom_command(TARGET benchmark POST_BUILD
            COMMAND ${_SIZE_TOOL} --format=berkeley $<TARGET_FILE:benchmark>
            COMMENT "[SIZE] Binary size summary"
        )
    endif()

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo
            "Output: ${CMAKE_CURRENT_BINARY_DIR}/benchmark"
        VERBATIM
    )


# ════════════════════════════════════════════════════════════════
#  分支 B：Cortex-M / 裸机（无操作系统，Flash Slot 模块）
# ════════════════════════════════════════════════════════════════
else()

    # ──────────────── 处理链接脚本 ────────────────
    set(LD_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/algo_module_generated.ld")
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/algo_module.ld.in"
        "${LD_GENERATED}"
        @ONLY
    )

    # 强制在链接命令末尾追加 -lm -lgcc（解决顺序问题）
    set(CMAKE_C_STANDARD_LIBRARIES "-lm -lgcc")

    add_executable(benchmark
        ${BENCH_SOURCES}
    )

    target_link_libraries(benchmark PRIVATE
        ${LIB_NAME}::${LIB_NAME}_cpp
    )

    target_include_directories(benchmark PRIVATE
        ${${LIB_NAME}_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    foreach(FLAG IN LISTS MCU_C_FLAGS)
        target_compile_options(benchmark PRIVATE ${FLAG})
    endforeach()

    target_compile_options(benchmark PRIVATE
        -ffreestanding
        -fno-builtin
        -fno-exceptions          # 裁机不需要 C++ 异常，去掉 .ARM.exidx 展开表（消除 __exidx_start/__exidx_end 依赖）
        -fno-rtti                # 去掉运行时类型信息，进一步减小二进制
        -ffunction-sections
        -fdata-sections
        -Os
        -Wall
        -Wextra
    )

    target_link_options(benchmark PRIVATE
        -nostartfiles
        --specs=nosys.specs      # 提供 newlib syscall 空实现（_exit/_kill/_getpid 等）
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/benchmark.map
        -T ${LD_GENERATED}
    )

    foreach(FLAG IN LISTS MCU_C_FLAGS)
        target_link_options(benchmark PRIVATE ${FLAG})
    endforeach()

    # ──────────────── 后处理：生成 bin/hex ────────────────
    set(OBJCOPY "${CMAKE_OBJCOPY}")
    if(NOT OBJCOPY OR OBJCOPY MATCHES "-NOTFOUND$")
        find_program(OBJCOPY NAMES arm-none-eabi-objcopy REQUIRED)
    endif()

    set(OBJDUMP "${CMAKE_OBJDUMP}")
    if(NOT OBJDUMP OR OBJDUMP MATCHES "-NOTFOUND$")
        find_program(OBJDUMP NAMES arm-none-eabi-objdump REQUIRED)
    endif()

    set(SIZE_TOOL "${CMAKE_C_COMPILER}")
    string(REPLACE "-gcc" "-size" SIZE_TOOL "${SIZE_TOOL}")
    if(NOT EXISTS "${SIZE_TOOL}")
        get_filename_component(_TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
        find_program(SIZE_TOOL
            NAMES arm-none-eabi-size
            HINTS "${_TOOLCHAIN_BIN_DIR}"
            REQUIRED
        )
    endif()

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${OBJCOPY} -O binary
            $<TARGET_FILE:benchmark>
            ${CMAKE_CURRENT_BINARY_DIR}/benchmark.bin
        COMMENT "[BIN] Generating benchmark.bin"
    )

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${OBJCOPY} -O ihex
            $<TARGET_FILE:benchmark>
            ${CMAKE_CURRENT_BINARY_DIR}/benchmark.hex
        COMMENT "[HEX] Generating benchmark.hex"
    )

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${OBJDUMP} -d -S
            $<TARGET_FILE:benchmark>
            > ${CMAKE_CURRENT_BINARY_DIR}/benchmark.dis
        COMMENT "[DIS] Generating disassembly"
    )

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${SIZE_TOOL} --format=berkeley $<TARGET_FILE:benchmark>
        COMMENT "[SIZE] Binary size summary"
    )

    add_custom_command(TARGET benchmark POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo
            "Output: ${CMAKE_CURRENT_BINARY_DIR}/benchmark.bin"
        COMMAND ${CMAKE_COMMAND} -E echo
            "Flash: openocd -f interface/stlink.cfg -f target/xxx.cfg -c 'program benchmark.bin ${ALGO_FLASH_ORIGIN} verify exit'"
    )

endif()

