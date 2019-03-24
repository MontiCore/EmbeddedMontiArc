javac HardwareEmulatorInterface.java RMIManager.java -h .
del /q "emulator_server.h"
ren "simulator_integration_HardwareEmulatorInterface.h" "emulator_server.h"
copy "emulator_server.h" "..\hardware_emulator\src\emulator"