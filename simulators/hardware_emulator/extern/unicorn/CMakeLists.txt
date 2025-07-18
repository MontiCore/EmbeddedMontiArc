cmake_minimum_required(VERSION 3.7)
project(unicorn)


if (CMAKE_HOST_WIN32)
  # For Windows -> reproduce the VS project in 'msvc'



  add_subdirectory(qemu)


  add_library(unicorn_static STATIC)
  target_link_libraries(unicorn_static PUBLIC x86_64-softmmu)
  target_include_directories(unicorn_static PUBLIC include)
  target_include_directories(unicorn_static PRIVATE 
      qemu
      qemu/include
      qemu/tcg
      msvc/unicorn
  )
  target_compile_definitions(unicorn_static PRIVATE 
    WIN32
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
    _WINDOWS
    _USRDLL
    UNICORN_DLL_EXPORTS
    UNICORN_SHARED
    _CRT_SECURE_NO_WARNINGS
    inline=__inline
    __func__=__FUNCTION__
    __x86_64__
    UNICORN_HAS_X86
    WIN32_LEAN_AND_MEAN
  )
  target_link_libraries(unicorn_static PRIVATE
  kernel32.lib
  user32.lib
  gdi32.lib
  winspool.lib
  comdlg32.lib
  advapi32.lib
  shell32.lib
  ole32.lib
  oleaut32.lib
  uuid.lib
  odbc32.lib
  odbccp32.lib
  )
  target_link_libraries(unicorn_static PUBLIC Dbghelp Shlwapi)



  set(unicorn_SOURCES 
      uc.c
      list.c
      qemu/arm.h
      qemu/m68k.h
      qemu/mips.h
      qemu/sparc.h
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


  set(unicorn_MSCV_SOURCES 
      msvc/unicorn/config-host.h
      msvc/unicorn/qapi-types.h
      msvc/unicorn/qapi-types.c
      msvc/unicorn/qapi-visit.h
      msvc/unicorn/qapi-visit.c
  )
  target_sources(unicorn_static PRIVATE ${unicorn_SOURCES})

  set(unicorn_INCLUDES
      include/unicorn/arm64.h
      include/unicorn/platform.h
      include/unicorn/unicorn.h
      include/unicorn/x86.h
      include/list.h
      include/qemu.h
      include/uc_priv.h
  )
  target_sources(unicorn_static PRIVATE ${unicorn_SOURCES} ${unicorn_INCLUDES} ${unicorn_MSCV_SOURCES})

  source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}"
    PREFIX "src"
    FILES ${unicorn_SOURCES}
  )

  source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PREFIX "includes"
    FILES ${unicorn_INCLUDES}
  )

  source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/msvc/unicorn"
    PREFIX "msvc"
    FILES ${unicorn_MSCV_SOURCES}
  )

  # add_definitions(
  #     /Gy
  #     /Oi 
  #     /MT
  # )



else()
  find_package(Threads)

  # For Linux -> use make
  # add_custom_target(unicorn_make 
  #   COMMAND ./make.sh
  #   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  # )
  # add_dependencies(unicorn_static unicorn_make)

  add_library(unicorn_static INTERFACE)

  target_link_directories(unicorn_static INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_BUILD_TYPE}")
  target_link_libraries(unicorn_static INTERFACE unicorn pthread dl) # Should use "libunicorn" instead of "unicorn" ?
  target_include_directories(unicorn_static INTERFACE include)


endif()