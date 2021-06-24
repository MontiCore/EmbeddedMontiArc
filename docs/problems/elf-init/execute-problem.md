
## Execution problem

- In second entry in .init_array table
- After a couple calls => wants to jump to address of this symbol: `std::locale::_S_initialize_once`
- But jumps to 0x0 => runs random code ?
- Address in emulator of the pointer: 0x0000000000371C98 (ema_autopilot_lib.so:seg1 F:0X00000000171C98)
- Uses *Label* ?? _ZNSt6locale18_S_initialize_onceEv => points to address 0x93bd0 ??

readelf -s (symbols):

- Symbol table '.dynsym' contains 4573 entries:
-  Num:    Value          Size Type    Bind   Vis      Ndx Name
-  388: 0000000000093bd0    64 FUNC    GLOBAL DEFAULT   12 _ZNSt6locale18_S_initialize_onceEv
- Symbol table '.symtab' contains 5445 entries:
-  Num:    Value          Size Type    Bind   Vis      Ndx Name
- 4634: 0000000000093bd0    64 FUNC    GLOBAL DEFAULT   12 _ZNSt6locale18_S_initialize_onceEv

readelf -r (relocations):
- Relocation section '.rela.dyn' at offset 0x608a0 contains 3240 entries:
-     Offset             Info             Type               Symbol's Value  Symbol's Name + Addend
- 0000000000371c98  0000018400000006 R_X86_64_GLOB_DAT      0000000000093bd0 _ZNSt6locale18_S_initialize_onceEv + 0
