cmake_minimum_required(VERSION 3.7)
project(unicorn)


add_subdirectory(qemu)

set(UNICORN_INCLUDE_DIR
    include
)

set(MSVC_INCLUDE_DIR
    msvc/unicorn
)

set(UNICORN_EXTERNAL_SOURCEFILES 
    ${MSVC_INCLUDE_DIR}/config-host.h
    ${MSVC_INCLUDE_DIR}/qapi-types.h
    ${MSVC_INCLUDE_DIR}/qapi-types.c
    ${MSVC_INCLUDE_DIR}/qapi-visit.h
    ${MSVC_INCLUDE_DIR}/qapi-visit.c
    ${UNICORN_INCLUDE_DIR}/unicorn/arm64.h
    ${UNICORN_INCLUDE_DIR}/unicorn/platform.h
    ${UNICORN_INCLUDE_DIR}/unicorn/unicorn.h
    ${UNICORN_INCLUDE_DIR}/unicorn/x86.h
    ${UNICORN_INCLUDE_DIR}/list.h
    ${UNICORN_INCLUDE_DIR}/qemu.h
    ${UNICORN_INCLUDE_DIR}/uc_priv.h
    uc.c
    list.c
    qemu/arm.h
    qemu/m68k.h
    qemu/mips.h
    qemu/sparc.h
)

set(UNICORN_INCLUDE_DIRS
    qemu
    qemu/tcg
    qemu/include
    ${UNICORN_INCLUDE_DIR}
    ${MSVC_INCLUDE_DIR}
)


set(UNICORN_SOURCEFILES 
    qemu/accel.c
    qemu/glib_compat.c
    qemu/qemu-log.c
    qemu/qemu-timer.c
    qemu/tcg-runtime.c
    qemu/vl.c
    qemu/vl.h
    
    qemu/hw/core/machine.c
    qemu/hw/core/qdev.c
    
    qemu/qapi/qapi-dealloc-visitor.c
    qemu/qapi/qapi-visit-core.c
    qemu/qapi/qmp-input-visitor.c
    qemu/qapi/qmp-output-visitor.c
    qemu/qapi/string-input-visitor.c
    
    qemu/qobject/qbool.c
    qemu/qobject/qdict.c
    qemu/qobject/qerror.c
    qemu/qobject/qfloat.c
    qemu/qobject/qint.c
    qemu/qobject/qlist.c
    qemu/qobject/qstring.c
    
    qemu/qom/container.c
    qemu/qom/cpu.c
    qemu/qom/object.c
    qemu/qom/qom-qobject.c
    
    qemu/util/aes.c
    qemu/util/bitmap.c
    qemu/util/bitops.c
    qemu/util/crc32c.c
    qemu/util/cutils.c
    qemu/util/error.c
    qemu/util/getauxval.c
    qemu/util/host-utils.c
    qemu/util/module.c
    qemu/util/oslib-win32.c
    qemu/util/qemu-error.c
    qemu/util/qemu-thread-win32.c
    qemu/util/qemu-timer-common.c
    
    
    qemu/include/config.h
    qemu/include/elf.h
    qemu/include/glib_compat.h
    qemu/include/qemu-common.h
    
    qemu/include/exec/address-spaces.h
    qemu/include/exec/cpu_ldst.h
    qemu/include/exec/cpu_ldst_template.h
    qemu/include/exec/cpu-all.h
    qemu/include/exec/cpu-common.h
    qemu/include/exec/cpu-defs.h
    qemu/include/exec/cputlb.h
    qemu/include/exec/exec-all.h
    qemu/include/exec/gen-icount.h
    qemu/include/exec/helper-gen.h
    qemu/include/exec/helper-head.h
    qemu/include/exec/helper-proto.h
    qemu/include/exec/helper-tcg.h
    qemu/include/exec/hwaddr.h
    qemu/include/exec/ioport.h
    qemu/include/exec/memory.h
    qemu/include/exec/memory-internal.h
    qemu/include/exec/ram_addr.h
    
    qemu/include/fpu/softfloat.h
    
    
    qemu/include/hw/arm/arm.h
    
    qemu/include/hw/cpu/icc_bus.h
    
    qemu/include/hw/i386/apic.h
    qemu/include/hw/i386/apic_internal.h
    qemu/include/hw/i386/pc.h
    
    qemu/include/hw/m68k/m68k.h
    
    qemu/include/hw/mips/cpudevs.h
    qemu/include/hw/mips/mips.h
    
    qemu/include/hw/sparc/sparc.h
    
    qemu/include/hw/boards.h
    qemu/include/hw/hw.h
    qemu/include/hw/qdev.h
    qemu/include/hw/qdev-core.h
    
    qemu/include/qapi/dealloc-visitor.h
    qemu/include/qapi/error.h
    qemu/include/qapi/qmp-input-visitor.h
    qemu/include/qapi/qmp-output-visitor.h
    qemu/include/qapi/string-input-visitor.h
    qemu/include/qapi/visitor.h
    qemu/include/qapi/visitor-impl.h
    
    qemu/include/qapi/qmp/qbool.h
    qemu/include/qapi/qmp/qdict.h
    qemu/include/qapi/qmp/qerror.h
    qemu/include/qapi/qmp/qfloat.h
    qemu/include/qapi/qmp/qint.h
    qemu/include/qapi/qmp/qjson.h
    qemu/include/qapi/qmp/qlist.h
    qemu/include/qapi/qmp/qobject.h
    qemu/include/qapi/qmp/qstring.h
    qemu/include/qapi/qmp/types.h
    
    qemu/include/qemu/aes.h
    qemu/include/qemu/atomic.h
    qemu/include/qemu/bitmap.h
    qemu/include/qemu/bitops.h
    qemu/include/qemu/bswap.h
    qemu/include/qemu/compiler.h
    qemu/include/qemu/crc32c.h
    qemu/include/qemu/host-utils.h
    qemu/include/qemu/int128.h
    qemu/include/qemu/log.h
    qemu/include/qemu/module.h
    qemu/include/qemu/osdep.h
    qemu/include/qemu/queue.h
    qemu/include/qemu/range.h
    qemu/include/qemu/thread.h
    qemu/include/qemu/thread-posix.h
    qemu/include/qemu/thread-win32.h
    qemu/include/qemu/timer.h
    qemu/include/qemu/typedefs.h
    
    qemu/include/qom/cpu.h
    qemu/include/qom/object.h
    qemu/include/qom/qom-qobject.h
    
    qemu/include/sysemu/accel.h
    qemu/include/sysemu/cpus.h
    qemu/include/sysemu/memory_mapping.h
    qemu/include/sysemu/os-win32.h
    qemu/include/sysemu/sysemu.h
)

foreach(source IN LISTS UNICORN_SOURCEFILES)
    get_filename_component(source_path "${source}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()


add_library(${PROJECT_NAME} ${UNICORN_SOURCEFILES} ${UNICORN_EXTERNAL_SOURCEFILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${UNICORN_INCLUDE_DIRS})
#target_compile_options(${PROJECT_NAME} PRIVATE ${SOFTMMU_CFLAGS})
#add_compile_definitions(${SOFTMMU_CFLAGS})

add_definitions(
    /FI"x86_64.h"
    /DWIN32
    /DNDEBUG
    /D_LIB
    /D__x86_64__
    /D_CRT_SECURE_NO_WARNINGS
    /Dinline=__inline
    /D__func__=__FUNCTION__
    /DWIN32_LEAN_AND_MEAN
    /DUNICORN_HAS_X86
    /Gy
    /Oi 
    /MT 
)