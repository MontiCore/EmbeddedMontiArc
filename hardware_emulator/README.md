# Hardware Emulator

The Hardware Emulator aims at emulating Windows DLL and Linux Archive functions in order to have a runtime evaluation of the execution time.

It uses the [Unicorn Emulator][1] wrapper library (based on [QEMU][2]) to emulate x86 code. 
The code for DLLs is placed in memory using information from the [pe-parse][3] library, 
a [Portable Executable][4] parser.

[Zydis][5] is used to disassemble instructions, and in combination with 
*code execution* and *memory access* hooks provided by unicorn allows to see memory writes, 
reads, register changes and the executed code.

## DLL Emulation

Library code makes calls to other libraries (mostly `KERNEL32.DLL` for Windows). 
These are detected by the emulator and it returns `0` every time unless there is a `SysCall`
registered for this API call. Those SysCalls are manually registered and simulate the
actual system call. (Ex: `load_library_ex(), get_proc_address(), get_proc_heap(), heap_alloc(), get_cmd_line(), get_module_handle()`)


[1]: https://www.unicorn-engine.org/
[2]: https://www.qemu.org/
[3]: https://github.com/trailofbits/pe-parse
[4]: https://docs.microsoft.com/fr-fr/windows/desktop/Debug/pe-format
[5]: https://github.com/zyantific/zydis