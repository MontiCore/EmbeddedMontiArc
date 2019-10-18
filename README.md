# vscode-ema-linter
A Visual Studio Code extension that lints EmbeddedMontiArc Languages

## Install

### Dependencies

- Java 8 or higher
- Maven

Make sure that Java and Maven can be found in your PATH by executing in a terminal:
```bash
> java -version
java version "1.8.0_141"
...

> mvn -version
Apache Maven 3.3.3 ...
```

### Extension

Download: [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/vscode-ema-linter/-/jobs/artifacts/master/raw/ema-linter.vsix?job=LinuxBuild) and install it by:
1. Opening vscode
2. Navigating to `Extensions`(View>Extensions)
3. Navigate to `Install from VSIX...`(...>Install from VSIX...)
4. Select the downloaded File

## Usage
Open a EmbeddedMontiArc project as a folder in Visual Studio Code.
Syntax and CoCo errors will now be shown.