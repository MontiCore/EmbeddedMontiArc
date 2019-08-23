/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.emam2wasm.wasm.WasmJsNameProvider;
import de.monticore.lang.monticar.emam2wasm.wasm.WasmStep;
import de.monticore.lang.monticar.emscripten.Emscripten;
import de.monticore.lang.monticar.emscripten.EmscriptenCommand;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import de.monticore.lang.monticar.setup.action.SetupAction;
import java.io.IOException;
import java.net.URL;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledOnOs;
import org.junit.jupiter.api.condition.OS;
import org.junit.jupiter.api.extension.ExtendWith;

class AutoSetupIT {

  private static final Path SOME_C_FILE = Paths
      .get("src/integration-test/resources/setup/hello_world.c");

  private static final Path EMSDK_WIN = Paths.get("./emsdk-master/emsdk.bat");
  private static final String EMSCRIPTEN_URL = "https://github.com/juj/emsdk/archive/master.zip";
  private static final String EMSCRIPTEN = "emscripten";
  private static final String EMSCRIPTEN_ARCHIVE_NAME = "emscripten.zip";
  private static final String EMCC_WIN = "emcc.bat";
  private static final String BRANCH = "emsdk-master";

  private static final String DOCKER_INSTALL_COMMAND =
      "docker run -dit --name emscripten -v $(pwd):/src trzeci/emscripten:sdk-incoming-64bit bash";
  private static final String DOCKER_EMSCRIPTEN_CALL = "docker exec -i emscripten emcc";

  @SafeVarargs
  private final <T> T[] arrayof(T... elements) {
    return elements;
  }

  @EnabledOnOs(OS.WINDOWS)
  @Nested
  class WhenOnWindows {

    private Path dir;
    private Path downloadPath;
    private Path extractionPath;
    private URL emscriptenUrl;
    private String emsdkCommand;
    private String[] emscriptenUpdate;
    private String[] emscriptenInstall;
    private String[] emscriptenActivate;
    private Configuration configuration;

    void setUp(Path dir) throws Exception {
      this.dir = dir;
      this.downloadPath = dir.resolve(EMSCRIPTEN_ARCHIVE_NAME);
      this.extractionPath = dir.resolve(EMSCRIPTEN);
      this.emscriptenUrl = new URL(EMSCRIPTEN_URL);
      this.emsdkCommand = "\"" + extractionPath.resolve(EMSDK_WIN)
          .toAbsolutePath().normalize().toString() + "\"";
      this.emscriptenUpdate = new String[]{emsdkCommand, "update"};
      this.emscriptenInstall = new String[]{emsdkCommand, "install", "latest"};
      this.emscriptenActivate = new String[]{emsdkCommand, "activate", "latest"};
      this.configuration = emscriptenWindowsConfig();
    }

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldSetupEmscripten(Path dir) throws Exception {
      setUp(dir);
      AutoSetup autoSetup = new AutoSetup(listof(configuration));

      autoSetup.setup();

      //find subdirectory, full path looks something like emscripten/emsdk-master/emscripten/1.37.36
      Path binaryDir = findSubDirectory(extractionPath.resolve(BRANCH).resolve(EMSCRIPTEN));
      String emcc = binaryDir.resolve(EMCC_WIN).toAbsolutePath().normalize().toString();

      assertThat(runEmscriptenCommand(emcc)).isZero();
    }

    private int runEmscriptenCommand(String emscripten) throws IOException, InterruptedException {
      ProcessBuilder pb = new ProcessBuilder();
      pb.inheritIO();
      pb.command(emscripten, "--version");
      return pb.start().waitFor();
    }

    private BasicConfiguration emscriptenWindowsConfig() {
      return new BasicConfiguration(
          new DownloadAction(emscriptenUrl, downloadPath),
          new ExtractionAction(downloadPath, extractionPath),
          new SetupAction(extractionPath, emscriptenWindowsCommands()));
    }

    private List<String[]> emscriptenWindowsCommands() {
      List<String[]> commands = new ArrayList<>();
      commands.add(emscriptenUpdate);
      commands.add(emscriptenInstall);
      commands.add(emscriptenActivate);
      return commands;
    }

    private Emscripten emscripten(String emscriptenCommand) {
      return new EmscriptenCommand(Shell.CMD, emscriptenCommand);
    }

    private Path findSubDirectory(Path path) throws IOException {
      try (DirectoryStream<Path> stream = Files.newDirectoryStream(path)) {
        for (Path p : stream) {
          if (Files.isDirectory(p)) {
            return p;
          }
        }
      }
      return null;
    }
  }

  private EmscriptenCommandBuilderFactory commandBuilderFactory(Emscripten emscripten) {
    EmscriptenCommandBuilderFactory commandBuilderFactory = new EmscriptenCommandBuilderFactory();
    commandBuilderFactory.setEmscripten(emscripten);
    return commandBuilderFactory;
  }

  @Nested
  @EnabledOnOs({OS.LINUX, OS.MAC})
  class WhenOnOtherOS {

    private SetupAction action;
    private Configuration configuration;

    @BeforeEach
    void setUp() {
      this.action = new SetupAction(commands());
      this.configuration = new BasicConfiguration(action);
    }

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldSetupEmscripten(Path dir) {
      AutoSetup autoSetup = new AutoSetup(listof(configuration));

      autoSetup.setup();

      WasmStep wasmStep = new WasmStep(commandBuilderFactory(emscripten(DOCKER_EMSCRIPTEN_CALL)),
          Paths.get("."), new WasmJsNameProvider());
      Path wasmFile = wasmStep.compile(SOME_C_FILE);

      assertThat(wasmFile).exists();
    }

    private List<String[]> commands() {
      Shell shell = Shell.BASH;
      List<String[]> commands = new ArrayList<>();
      commands.add(new String[]{
          shell.getShellCommand(),
          shell.getExecuteSwitch(),
          DOCKER_INSTALL_COMMAND});
      return commands;
    }

    private Emscripten emscripten(String emscriptenCommand) {
      return new EmscriptenCommand(Shell.BASH, emscriptenCommand);
    }
  }

  @SafeVarargs
  private final <T> List<T> listof(T... elements) {
    return Arrays.asList(elements);
  }

}
