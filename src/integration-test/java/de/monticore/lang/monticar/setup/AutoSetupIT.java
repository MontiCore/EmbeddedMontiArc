package de.monticore.lang.monticar.setup;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.emam2wasm.wasm.WasmJsNameProvider;
import de.monticore.lang.monticar.emam2wasm.wasm.WasmStep;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.net.URL;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class AutoSetupIT {

  private static final Path SOME_C_FILE = Paths
      .get("src/integration-test/resources/setup/hello_world.c");
  private static final String EMSCRIPTEN_URL = "https://github.com/juj/emsdk/archive/master.zip";
  private static final String OS_NAME = System.getProperty("os.name").toLowerCase();
  private static final String EMSCRIPTEN = "emscripten";
  private static final String EMSCRIPTEN_ARCHIVE_NAME = "emscripten.zip";
  private static final String WINDOWS = "windows";
  private static final String EMCC_WIN = "emcc.bat";
  private static final String EMCC_LINUX = "emcc";
  private static final String BRANCH = "emsdk-master";

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldSetupEmscripten(Path dir) throws IOException {
    Path emscriptenDownloadPath = dir.resolve(EMSCRIPTEN_ARCHIVE_NAME);
    Path emscriptenDir = dir.resolve(EMSCRIPTEN);

    Configuration configuration = new EmscriptenConfiguration(new URL(EMSCRIPTEN_URL),
        emscriptenDownloadPath, emscriptenDir, OS_NAME);
    AutoSetup autoSetup = new AutoSetup(listof(configuration));

    autoSetup.setup();

    //find subdirectory, full path looks something like emscripten/emsdk-master/emscripten/1.37.36
    Path emscriptenBinaryDir = findSubDirectory(
        emscriptenDir.resolve(BRANCH).resolve(EMSCRIPTEN));
    Path emscriptenBinary = emscriptenBinaryDir.resolve(emcc());

    WasmStep wasmStep = new WasmStep(
        commandBuilderFactory("\"" + emscriptenBinary.toAbsolutePath().toString() + "\""), dir,
        new WasmJsNameProvider());
    Path wasmFile = wasmStep
        .compile(SOME_C_FILE);

    assertThat(wasmFile).exists();
  }

  private EmscriptenCommandBuilderFactory commandBuilderFactory(String emscripten) {
    EmscriptenCommandBuilderFactory commandBuilderFactory = new EmscriptenCommandBuilderFactory();
    commandBuilderFactory.setEmscripten(emscripten);
    return commandBuilderFactory;
  }

  private String emcc() {
    return OS_NAME.startsWith(WINDOWS) ? EMCC_WIN : EMCC_LINUX;
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

  @SafeVarargs
  private final <T> List<T> listof(T... elements) {
    return Arrays.asList(elements);
  }

}
