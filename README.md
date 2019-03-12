# Hardware emulator

## Build

This project builds upon [unicorn](unicorn/README.md), [pe-parse](pe-parse/README.md) and (Zydis)[zydis/README.md]. It requires `cmake`,`python`, `java` and a valid `JAVA_PATH`.

### Windows

To build the Hardware Emulator with Visual Studio start [docs/build_release.bat](docs/build_release.bat). Be sure to have the visual studio tools in the PATH or start the build in the Visual Studio Command Prompt.

```
(VsDevCmd.bat)
git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/hardware_emulator.git
cd hardware_emulator
docs\build_release.bat
```

To update the CMake project call `..\docs\build.bat` from within the project directory.

The dependencies are pre-compiled under the [libs](hardware_emulator/libs) folder. To re-build them use [build_dependencies.bat](docs/build_dependencies.bat).

### Linux

To build all projects for Linux:

```
git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/hardware_emulator.git
cd hardware_emulator
docs/build_all.sh
```

## Install

Put the resulting `HardwareEmulator` library (.dll or .so) in the PATH of the [RMIModelServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer) or in its folder before starting it.

## Test

The `hardware-emulator-test` can be executed in the hardware_emulator/bin folder.