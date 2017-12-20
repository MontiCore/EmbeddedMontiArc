package de.monticore.lang.monticar.emam2wasm;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.emam2cppwrapper.GeneratorCPPWrapper;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.util.FileUtils;
import de.monticore.lang.monticar.util.TextFile;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class Emam2WasmMediatorTest {

  private static final Path EMA2CPP_BASE_DIR = Paths.get("src/test/resources/ema2cpp");
  private static final Path CPP_GENERATION_TARGET_DIR = EMA2CPP_BASE_DIR.resolve("cpp");
  private static final String MODEL_WITH_MATRIX_FULL_NAME = "models.model";

  private static final Path CPP2WASM_BASE_DIR = Paths.get("src/test/resources/cpp2wasm");
  private static final Path WASM_GENERATION_TARGET_DIR = CPP2WASM_BASE_DIR.resolve("wasm");

  @Nested
  class Ema2Cpp {

    Emam2WasmMediator mediator;

    @BeforeEach
    void setUp() {
      mediator = new Emam2WasmMediator(new GeneratorCPPWrapper(new GeneratorCPP()),
          EMA2CPP_BASE_DIR);
    }

    @Test
    void whenModelWithMatrix() throws IOException {
      TextFile cppFile = mediator.ema2cpp(MODEL_WITH_MATRIX_FULL_NAME);

      assertThat(cppFile.getPath()).isRegularFile();
    }

    @AfterEach
    void cleanUp() {
      FileUtils.delete(CPP_GENERATION_TARGET_DIR);
    }
  }

  @Disabled
  @Nested
  class Cpp2Wasm {
    //TODO: This requires emscripten to be installed automatically
    //see https://github.com/jirutka/emscripten-travis-example

    Emam2WasmMediator mediator;

    @BeforeEach
    void setUp() {
      mediator = new Emam2WasmMediator(new GeneratorCPPWrapper(new GeneratorCPP()),
          CPP2WASM_BASE_DIR);
    }

    @Test
    void whenGivenCppFiles() throws IOException, InterruptedException {
      TextFile jsFile = mediator.cpp2wasm();

      assertThat(jsFile.getPath()).isRegularFile();
    }

    @AfterEach
    void cleanUp() {
      FileUtils.delete(WASM_GENERATION_TARGET_DIR);
    }
  }
}