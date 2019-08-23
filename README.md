<!-- (c) https://github.com/MontiCore/monticore -->
[![Linux Build][travis-image]][travis-url]
[![Windows Build][appveyor-image]][appveyor-url]
[![Coverage Codecov][codecov-image]][codecov-url]
[![Coverage Coveralls][coveralls-image]][coveralls-url]
# EMAM2Wasm

This project compiles EmbeddedMontiArc models to WebAssembly and generates a fully functional web interface to the model.
Try it out at: https://embeddedmontiarc.github.io/EMAM2Wasm/

![image](https://user-images.githubusercontent.com/33121464/38885248-ac4aedfe-4272-11e8-935c-afa3f0f1a4c1.png)

## Getting started

Download the latest release from [https://github.com/EmbeddedMontiArc/EMAM2Wasm/releases](https://github.com/EmbeddedMontiArc/EMAM2Wasm/releases).
Extract the archive and run 
``` bash
$ scripts\setup.bat
```

To compile a model, use
``` bash
$ java -jar emam2wasm [options]
```

## Command Line Options

This project uses Spring to inject configuration parameters. Each parameter has to be provided in the form `--<option>=<value>`.

### Required:

Option | Explaination
-------- | -------------
model-path | Path to the base model directory
model | Full model name (package + component name)
spring.profiles.active | Set the active spring profiles. Available are <ul><li>`compiler`: Compile EMAM to WebAssembly</li><li>`setup`: Download and install emscripten, download Armadillo</li></ul>

### Optional:

Option | Explaination | Default value
-------- | ------------- | ---------------
target | Path to which all files will be generated.  `target` has precedence over `cpp-dir`, `wasm-dir` and `web-dir`  | 
cpp-dir | Path to which all C++ files will be generated | Current directory
wasm-dir | Path to which all WebAssembly files will be generated | Current directory
web-dir | Path to which all Javascript and HTML files will be generated | Current directory
include | List of paths that will be passed to emscripten as `-I"<path>"` | empty
library | List of paths that will be passed to emscripten as `-L"<path>"` | empty
options | List of options of the form `<option>=[0\|1]` | WASM=1, LINKABLE=1, EXPORT_ALL=1, ALLOW_MEMORY_GROWTH=1
flag | List of flags that will be passed to emscripten as `-<flag>` | std=c++11
bind | Either `true` or `false`. If `true`, `--bind` is passed to emscripten | `true`
algebraic-optimization | Either `true` or `false`. Sets the `useAlgebraicOptimizations` flag in [EMAM2Cpp](https://github.com/EmbeddedMontiArc/EMAM2Cpp) | `true`
generate-tests | Either `true` or `false`. Sets the `generateTests` flag in [EMAM2Cpp](https://github.com/EmbeddedMontiArc/EMAM2Cpp) | `true`
emscripten.execute.command | The command that needs to be executed to invoke emscripten | By default, emam2wasm will look for the *.emscripten* configuration file and use the path that is defined there
emscripten.execute.binary.name | The name of the emscripten binary to use | *emcc.bat*

[travis-image]: https://img.shields.io/travis/EmbeddedMontiArc/EMAM2Wasm.svg?branch=master&label=linux
[travis-url]: https://travis-ci.org/EmbeddedMontiArc/EMAM2Wasm
[appveyor-image]: https://img.shields.io/appveyor/ci/sbrunecker/emam2wasm/master.svg?label=windows
[appveyor-url]: https://ci.appveyor.com/project/sbrunecker/emam2wasm
[codecov-image]: https://img.shields.io/codecov/c/github/EmbeddedMontiArc/EMAM2Wasm/master.svg?label=all%20tests
[codecov-url]: https://codecov.io/gh/EmbeddedMontiArc/EMAM2Wasm/branch/master
[coveralls-image]: https://img.shields.io/coveralls/EmbeddedMontiArc/EMAM2Wasm/master.svg?label=unit%20tests
[coveralls-url]: https://coveralls.io/github/EmbeddedMontiArc/EMAM2Wasm?branch=master
