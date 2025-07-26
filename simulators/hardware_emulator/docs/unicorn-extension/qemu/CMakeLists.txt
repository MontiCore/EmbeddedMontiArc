project(unicorn)


#FILE(GLOB LOCAL_SOURCE "*.h" "*.c")

set(SOFTMMU_NAME x86_64)

set(ROOT_DIR ..)

set(UNICORN_INCLUDE_DIR
    ${ROOT_DIR}/include
)

set(MSVC_INCLUDE_DIR
    ../msvc/unicorn
)

set(SOFTMMU_EXTERNAL_SOURCEFILES 
    ${MSVC_INCLUDE_DIR}/config-host.h
    ${MSVC_INCLUDE_DIR}/${SOFTMMU_NAME}-softmmu/config-target.h
)

set(SOFTMMU_INCLUDE_DIRS
    .
    tcg
    tcg/i386
    target-i386
    include
    ${UNICORN_INCLUDE_DIR}
    ${MSVC_INCLUDE_DIR}
    ${MSVC_INCLUDE_DIR}/x86_64-softmmu
)

set(SOFTMMU_SOURCEFILES 
    cpu-exec.c
    cpus.c
    cputlb.c
    exec.c
    ioport.c
    memory.c
    memory_mapping.c
    translate-all.c
    x86_64.h
    
    tcg/optimize.c
    tcg/tcg.c
    tcg/tcg.h
    tcg/tcg-be-ldst.h
    tcg/tcg-be-null.h
    tcg/tcg-op.h
    tcg/tcg-opc.h
    tcg/tcg-runtime.h
    
    #tcg/i386/tcg-target.c
    tcg/i386/tcg-target.h
    
    target-i386/arch_memory_mapping.c
    target-i386/cc_helper.c
    target-i386/cc_helper_template.h
    target-i386/cpu.c
    target-i386/cpu.h
    target-i386/cpu-qom.h
    target-i386/excp_helper.c
    target-i386/fpu_helper.c
    target-i386/helper.c
    target-i386/helper.h
    target-i386/int_helper.c
    target-i386/mem_helper.c
    target-i386/misc_helper.c
    target-i386/ops_sse.h
    target-i386/ops_sse_header.h
    target-i386/seg_helper.c
    target-i386/shift_helper_template.h
    target-i386/smm_helper.c
    target-i386/svm.h
    target-i386/svm_helper.c
    target-i386/topology.h
    target-i386/translate.c
    target-i386/unicorn.c
    target-i386/unicorn.h
    
    hw/i386/pc.c
    hw/i386/pc_piix.c
    hw/intc/apic.c
    hw/intc/apic_common.c
    
    fpu/softfloat.c
    fpu/softfloat-macros.h
    fpu/softfloat-specialize.h
)



foreach(source IN LISTS SOFTMMU_SOURCEFILES)
    get_filename_component(source_path "${source}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()


add_library(${SOFTMMU_NAME}-softmmu ${SOFTMMU_SOURCEFILES} ${SOFTMMU_EXTERNAL_SOURCEFILES})
target_include_directories(${SOFTMMU_NAME}-softmmu PUBLIC ${SOFTMMU_INCLUDE_DIRS})
#target_compile_options(${SOFTMMU_NAME}-softmmu PRIVATE ${SOFTMMU_CFLAGS})
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
    /DNEED_CPU_H
    /DWIN32_LEAN_AND_MEAN
    /Gy
    /Oi 
    /MT 
)

