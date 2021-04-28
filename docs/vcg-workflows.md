# VCG Usage

Make sure everything is configured as described in [Setup](setup.md).

The following assumes you have the `build_environment` folder as your working directory.

> **Note**: All the following examples can be tested with the `sample_autopilot`. (Except EMA, test with `test_emadl_ap`/`test_emam_ap`.)

> **Note 2**: (Almost) all script have some help information using `<script> --help`.

## Building a program for the vcg

```bash
# Go into the compilation container
start.bat or ./start.sh
# In the container
./build.sh <project_folder>
# In a shell with the SSH setup
./upload.sh <project_folder> <vcg*>
```

## Running a program

```bash
# 2-way port forwarding (from your localhost to that of the raspberry to that of the VCG)
./connect.sh <port>
# In the raspberry shell
./connect.sh <port> <vcg*>
# Attach to the container
lxcattach appcont
# Run the program
/data/pdata/programs/<program> <port>
ctrl+c to stop
ctrl+d to leave containers/shells
```

### Stuck container

If the program in a container cannot be stopped, it is sufficient to restart the container. For this, call the following in the VCG shell:

```bash
lxcstop appcont
lxcstart appcont
```

## Running a simulation

To connect the simulation with the VCG, make sure there is a `VCG` component in the scenario's vehicle, described as follows:

```json
{
    "type": "vcg",
    "name": "VCG",
    "communication": {
        "type": "tcp",
        "host": "::1",
        "port": <port>
    },
    "time": "measured",
    "cycle_duration": [0,100000000]
}
```

Where `<port>` is the same port used in the `connect` script and program calls. `"host"` should be your localhost (maybe you need `"localhost"` instead of the IPv6 localhost `"::1"`).

## Running in TCP-mode locally

You can build and run in TCP mode locally.

```bash
# Build the project for your computer
build_local.bat or ./build_local.sh
# Run the program
<project_dir>/local_build/<program_name> # (.exe for windows)
```

Then just start a simulation with the same VCG properties in the scenario as specified above. (This works transparently since in both case the program is available on the localhost, with port forwarding in the case of the VCG).

## EMA Workflow

Part of the following workflow might be changed to the *stream test* maven plugin at some point.

### ema_config

The following scripts use the contents of a `ema_config` file inside the project folder to specify generation and build parameters for the EMA model. The entries are:

- `MODEL_DIR`: The relative folder containing the EMA model file structure.
- `MODEL_NAME`: The package reference of the main autopilot component. (Note that the package uses **lowercase** even if the file/component name starts with a capital letter.)
- `OUTPUT_NAME`: The name of the generated adapter.
- `BUILD_CONFIG`: Release or Debug (for the C++ compilation). (Currently only used by `ema_build_local`).
- `DLL_DIR`: If building *locally* and this is different than `"."`, the `ema_build_local` script will directly copy the `DynamicInterface` adapter to the specified folder. This can be used to directly copy to the *basic-simulator* install folder and be used with a scenario similar to `EMA_native.json`.
- `GENERATOR`: If specifies, tell CMake which generator to use.

### Generation & VCG build

Make sure that the path/version to the EMA...2CPP project are set in `config`.

```bash
# Generate the code
./emadl_generate.sh or ./emam_generate.sh
# Go into the compilation container
./start.sh
# In the container
./ema_build.sh <project_folder>
# The server adapter is then name '<OUTPUT_NAME>Adapter'
# In a shell with the SSH setup
./ema_upload.sh <project_folder> <vcg*>
```

Running the program on the VCG is the same as above.

### Local EMA build

In order to build the EMA project for the *local* machine, use the `ema_build_local` script after the `emadl_generate` step. Then the adapter can be run in server mode locally. The executable in in the `<project>/target/local_build` directory (`Debug`/`Release` folder for MSVC).
