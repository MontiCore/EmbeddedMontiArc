# EmbeddedMontiArcMath debugger plugin for vscode

## Usage
Open a project containing EMAM components and Steam tests as a folder and then navigate to one of your components. You can now navigate to one of your components and click on

```▶️ Run stream test for current component```

![Gif of CodeLens](./docu/CodeLens.gif "Simple debug workflow")



You can also add a debug configuration to your `launch.config`:

![Gif of LaunchConfig](./docu/LaunchConfig.gif "Simple debug workflow")


## Install
Download the latest build from [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/emam-debugger-vscode/-/jobs/artifacts/master/raw/emam-debug.vsix?job=LinuxBuild) and install it by:
1. Opening vscode
2. Navigating to `Extensions`(View>Extensions)
3. Navigate to `Install from VSIX...`(...>Install from VSIX...)
4. Select the downloaded File

## Update
1. Uninstall via the Extension view
2. Restart
3. Install new version as described above

## Building
Change the download link in [build.sh](build.sh#L3) to match the EMAM2CPP(jar with dependencies) version you need.

Then run `./build.sh` on a machine with:
- npm
- vsce

Alternatively you can copy your local version of EMAM2CPP(jar with dependencies) to `resources/emam-generator.jar` and then run
```bash
npm install
vsce package -o "emam-debug.vsix"
```