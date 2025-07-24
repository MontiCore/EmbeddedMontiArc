project(qemu)


#FILE(GLOB LOCAL_SOURCE "*.h" "*.c")

add_library(x86_64-softmmu STATIC)
target_include_directories(x86_64-softmmu PRIVATE
    .
    ../include
    include
    tcg
    tcg/i386
    target-i386
    ../msvc/unicorn
    ../msvc/unicorn/x86_64-softmmu
)
target_compile_definitions(x86_64-softmmu PRIVATE
    WIN32
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
    _LIB
    #__i386__ or __x86_64__
    __x86_64__
    _CRT_SECURE_NO_WARNINGS
    inline=__inline
    __func__=__FUNCTION__
    NEED_CPU_H
    WIN32_LEAN_AND_MEAN
)
target_compile_options(x86_64-softmmu PRIVATE
    /FI"x86_64.h"
)

set(x86_64_softmmu_SOURCES
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
target_sources(x86_64-softmmu PRIVATE ${x86_64_softmmu_SOURCES})

source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}"
  PREFIX "src"
  FILES ${x86_64_softmmu_SOURCES}
)










# set(SOFTMMU_EXTERNAL_SOURCEFILES 
#     ${MSVC_INCLUDE_DIR}/config-host.h
#     ${MSVC_INCLUDE_DIR}/${SOFTMMU_NAME}-softmmu/config-target.h
# )

# add_definitions(
#     /Gy
#     /Oi 
#     /MT 
# )

