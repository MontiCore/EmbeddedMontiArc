package de.monticore.lang.monticar.setup;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.cli.EmscriptenSetupConfig;
import de.monticore.lang.monticar.emam2wasm.wasm.WasmJsNameProvider;
import de.monticore.lang.monticar.emam2wasm.wasm.WasmStep;
import de.monticore.lang.monticar.emscripten.Emscripten;
import de.monticore.lang.monticar.emscripten.EmscriptenCommand;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.junit.jupiter.SpringExtension;

@ExtendWith(SpringExtension.class)
@ContextConfiguration(classes = EmscriptenSetupConfig.class,
    initializers = {OSContextInitializer.class})
@ActiveProfiles(profiles = "setup")
class AutoSetupIT {

  private static final Path SOME_C_FILE = Paths
      .get("src/integration-test/resources/setup/hello_world.c");

  @Autowired
  private Configuration emscriptenConfiguration;

  @Autowired
  private Supplier<String> emscriptenCommandSupplier;

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldSetupEmscriptenLinux(Path dir) {
    AutoSetup autoSetup = new AutoSetup(listof(emscriptenConfiguration));

    autoSetup.setup();

    WasmStep wasmStep = new WasmStep(
        commandBuilderFactory(emscripten(emscriptenCommandSupplier.get())),
        dir, new WasmJsNameProvider());
    Path wasmFile = wasmStep.compile(SOME_C_FILE);

    assertThat(wasmFile).exists();
  }

  private EmscriptenCommandBuilderFactory commandBuilderFactory(Emscripten emscripten) {
    EmscriptenCommandBuilderFactory commandBuilderFactory = new EmscriptenCommandBuilderFactory();
    commandBuilderFactory.setEmscripten(emscripten);
    return commandBuilderFactory;
  }

  private Emscripten emscripten(String emscriptenCommand) {
    return new EmscriptenCommand(Shell.CMD, emscriptenCommand);
  }

  @SafeVarargs
  private final <T> List<T> listof(T... elements) {
    return Arrays.asList(elements);
  }

}
