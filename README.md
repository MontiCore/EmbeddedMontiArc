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

## Development
### Structure
This extension spawns one Langauge Server(written in Java) for each supported language via Maven and communicates with it via a network port.
For EmbeddedMontiArc Languages the Language Servers are implemented in [ema-lsp](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/ema-lsp) with the help of [monticore-lsp-commons](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/monticore-lsp-commons).

### Building
Run `./build.sh` with a bash shell.

Alternatively: 
1. run `npm install` to download the dependencies
2. run `node scripts/createSyntax.js ./settings/global.json` to generate syntax highlighting and activation events for the extension. Warning: this modifies the `package.json` file!
3. run `vsce package -o "ema-linter.vsix"` to create the installable `.vsix` file.

### Adding a new Language

1. Implement the Language Server in [ema-lsp](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/ema-lsp)
2. Create a new subdirectory in the settings folder for your language
    - constants.json: settings for the MavenLanguageClient. Fields:
        - languageName: Name of the new Language(e.g. EmbeddedMontiArc)
        - fileExtension: File extension of the new Language(e.g. `.ema`)
        - pomRoot: relative path(from the **project root**) to the directory that contains the `pom.xml`
        - relativeMvnSettingsPath: relative path(from the **pom.xml**) to the `settings.xml`
        - useRunningServer:
            - true => no Language Server will be spawned for the Language, instead a connection will be established on the port specified in manualPort. 
            - false => a Language Server will be spawned via maven and the port will be parsed from `stdout` of the Language Server
        - manualPort: port that will be used with useRunningServer = true
        - bufferSize: size of the buffer used to parse the port from `stdout` of the Language Server, as well as logging of errors
        - keywordsFile: relative path(from the **project root**) which will be used to generate syntax highlighting for keywords of the language
    - pom.xml: a Maven file to start the Language Server(e.g. `mvn exec:java -s ../settings.xml`)
    - keywords.txt: a list of language keywords seperated by a newline(file can be renamed in constants.json)
3. Register the Language by adding the relative path to the `constants.json` in [global.json](settings/global.json)::clientOptions
4. Re-run: `node scripts/createSyntax.js ./settings/global.json`