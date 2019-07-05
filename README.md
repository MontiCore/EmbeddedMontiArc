# Hardware emulator

The **RMIModelServer** README contains general development information regarding the HardwareEmulator in the simulation context.

## Build

This project builds upon [unicorn](unicorn/README.md), [pe-parse](pe-parse/README.md) and [Zydis](zydis/README.md). It requires `cmake`, (`python`), `java` and a valid `JAVA_PATH`.

The dependencies are pre-compiled under the [libs](hardware_emulator/libs) folder. To re-build them use [build_dependencies.bat](scripts/build_dependencies.bat) or [build_dependencies.sh](scripts/build_dependencies.sh).

### Windows

To build the Hardware Emulator with Visual Studio start [scripts\build.bat](scripts\build.bat) inside the *hardware_emulator* project. Be sure to have the visual studio tools in the PATH or start the build in the Visual Studio Command Prompt.

```
(VsDevCmd.bat)
git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/hardware_emulator.git
cd hardware_emulator\hardware_emulator
..\scripts\build.bat
```

The open the Visual Studio under the `build` folder and compile the project. 

Alternatively you can directly compile the project with the `build_release.bat` script.

### Linux

To build the project under linux run the following:

```
git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/hardware_emulator.git
cd hardware_emulator/hardware_emulator
../scripts/build.sh
```

You can pass *Debug* to the `build.sh` script to compile a Debug version.

## Install

Put the resulting `HardwareEmulator` library (.dll or .so) in the installation folder of the [RMIModelServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer).

## Test

The `hardware-emulator-test` can be executed in the hardware_emulator/bin folder locally to test the emulation of sample programs and a basic autopilot emulation.