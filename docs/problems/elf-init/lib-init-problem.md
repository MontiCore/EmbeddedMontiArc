## Problem

scenario: ema_ap_tc-1-1-1_emu.json

autopilot::buffer is not initialized.
'_init' does not seem to load anything

=> Find the 'entry point' used to initialized the dll

(elf 'entry point' ?, '.init' section ?)

- _init address: 0000000000078b88
- entry_point address: 0x7c3d0 => calls abort after 2 instructions???

- debugger with breakpoint on Buffer constructor?
- Check dlopen code ????

## Resources

- https://stevens.netmeister.org/631/elf.html
    - .init, .fini, .preinit_array, .init_array and .fini_array sections
- https://www.keil.com/support/man/docs/ARMLINK/armlink_chunkpge1362065909059.htm
- https://www.bfilipek.com/2018/02/staticvars.html

## TO SORT NOTES

- https://blog.packagecloud.io/eng/2016/04/05/the-definitive-guide-to-linux-system-calls/
- https://itanium-cxx-abi.github.io/cxx-abi/abi.html#mangling


## Tasks

- [ ] Test DLL with static C++ object
  - [ ] Debug version
  - [ ] Same build as the autopilot (CMake)
- [ ] Program with dlopen()
  - [ ] Debug version

## load_test call stack

- https://github.com/lattera/glibc/blob/master/elf/dl-init.c

lib_test.so!Foo::Foo(Foo * const this) (/home/jean/dev/build_environment/load_test/src/lib.cpp:16)
lib_test.so!__static_initialization_and_destruction_0(int __initialize_p, int __priority) (/home/jean/dev/build_environment/load_test/src/lib.cpp:8)
lib_test.so!_GLOBAL__sub_I_lib.cpp(void)() (/home/jean/dev/build_environment/load_test/src/lib.cpp:17)
ld-linux-x86-64.so.2!call_init(char ** env, char ** argv, int argc, struct link_map * l) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-init.c:72)
ld-linux-x86-64.so.2!_dl_init(struct link_map * main_map, int argc, char ** argv, char ** env) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-init.c:119)
ld-linux-x86-64.so.2!dl_open_worker(void * a) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-open.c:522)
libc.so.6!__GI__dl_catch_exception(struct dl_exception * exception, void (*)(void *) operate, void * args) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-error-skeleton.c:196)
ld-linux-x86-64.so.2!_dl_open(const char * file, int mode, const void * caller_dlopen, Lmid_t nsid, int argc, char ** argv, char ** env) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-open.c:605)
libdl.so.2!dlopen_doit(void * a) (/build/glibc-S9d2JN/glibc-2.27/dlfcn/dlopen.c:66)
libc.so.6!__GI__dl_catch_exception(struct dl_exception * exception, void (*)(void *) operate, void * args) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-error-skeleton.c:196)
libc.so.6!__GI__dl_catch_error(const char ** objname, const char ** errstring, _Bool * mallocedp, void (*)(void *) operate, void * args) (/build/glibc-S9d2JN/glibc-2.27/elf/dl-error-skeleton.c:215)
libdl.so.2!_dlerror_run(void (*)(void *) operate, void * args) (/build/glibc-S9d2JN/glibc-2.27/dlfcn/dlerror.c:162)
libdl.so.2!__dlopen(const char * file, int mode) (/build/glibc-S9d2JN/glibc-2.27/dlfcn/dlopen.c:87)
main(int argc, char ** argv) (/home/jean/dev/build_environment/load_test/src/main.cpp:9)

## What dl-init.c does

- Check "DT_INIT" Dynamic section entry ??
- If present, call with (argc, argv, env)
- Checks "DT_INIT_ARRAY" dynamic section entrIES
- Uses "DT_INIT_ARRAYSZ" to get number of entries
- Calls every address in the same way (argc, argv, env)


## Doing the same

- https://en.wikipedia.org/wiki/Executable_and_Linkable_Format
- Check 'PT_DYNAMIC' Program header ? or 'SHT_DYNAMIC' Section header
- Check what "DT_INIT" is in the elf file
  - DT_INIT: value of 'd_tag' in Elf64_Dyn (= Dynamic section entry.)
  - `#define DT_INIT		12		/* Address of init function */`
  - `#define DT_INIT_ARRAY	25		/* Array with addresses of init fct */`
  - `#define DT_INIT_ARRAYSZ	27		/* Size in bytes of DT_INIT_ARRAY */`