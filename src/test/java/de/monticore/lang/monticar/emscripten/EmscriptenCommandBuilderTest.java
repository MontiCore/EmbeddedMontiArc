package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

//Path variables have to be used in assertions to build expected string
//because of unix vs. windows file seperators
class EmscriptenCommandBuilderTest {

  private static final Path INCLUDE_ARMADILLO = Paths.get("./armadillo/include");
  private static final Path INCLUDE_BLAS = Paths.get("./blas");
  private static final Option WASM_OPTION = new Option("WASM", true);
  private static final Option LINKABLE_OPTION = new Option("LINKABLE", true);
  private static final Optimization SOME_LEVEL = Optimization.O3;
  private String emscripten;
  private Path file;

  @BeforeEach
  void setUp() {
    emscripten = "./emscripten";
    file = Paths.get("model.cpp");
  }

  @SafeVarargs
  private final <T> List<T> listof(T... elements) {
    return Arrays.asList(elements);
  }

  @Nested
  class ToList {

    @Test
    void whenSimpleCommand() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      assertThat(builder.toList()).isEqualTo(listof(emscripten, "model.cpp"));
    }

    @Test
    void whenCommandWithOption() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.addOption(WASM_OPTION);

      assertThat(builder.toList())
          .isEqualTo(listof(emscripten, "model.cpp", "-s", "WASM=1"));
    }

    @Test
    void whenCommandWithMultipleOptions() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.addOption(WASM_OPTION);
      builder.addOption(LINKABLE_OPTION);

      assertThat(builder.toList())
          .isEqualTo(
              listof(emscripten, "model.cpp", "-s", "WASM=1", "-s", "LINKABLE=1"));
    }

    @Test
    void whenCommandWithInclude() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);

      assertThat(builder.toList())
          .isEqualTo(
              listof(emscripten, "model.cpp",
                  "-I\"" + INCLUDE_ARMADILLO.toString() + "\""));
    }

    @Test
    void whenCommandWithMultipleIncludes() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);
      builder.include(INCLUDE_BLAS);

      assertThat(builder.toList())
          .isEqualTo(
              listof(emscripten, "model.cpp",
                  "-I\"" + INCLUDE_ARMADILLO.toString() + "\"",
                  "-I\"" + INCLUDE_BLAS.toString() + "\""));
    }

    @Test
    void whenCommandWithOptimization() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.setOptimization(SOME_LEVEL);

      assertThat(builder.toList()).isEqualTo(listof(emscripten, "model.cpp", "-O3"));
    }

    @Test
    void whenCommandWithBind() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.setBind(true);

      assertThat(builder.toList())
          .isEqualTo(listof(emscripten, "model.cpp", "--bind"));
    }

    @Test
    void whenFullCommand() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);
      builder.addOption(WASM_OPTION);
      builder.setOptimization(SOME_LEVEL);
      builder.setBind(true);

      assertThat(builder.toList())
          .isEqualTo(listof(emscripten, "model.cpp",
              "-I\"" + INCLUDE_ARMADILLO.toString() + "\"", "-s", "WASM=1", "-O3", "--bind"));
    }
  }

  @Nested
  class ToString {

    @Test
    void whenSimpleCommand() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      assertThat(builder.toString()).isEqualTo(emscripten + " model.cpp");
    }

    @Test
    void whenCommandWithOption() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.addOption(WASM_OPTION);

      assertThat(builder.toString()).isEqualTo(emscripten + " model.cpp -s WASM=1");
    }

    @Test
    void whenCommandWithMultipleOptions() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.addOption(WASM_OPTION);
      builder.addOption(LINKABLE_OPTION);

      assertThat(builder.toString())
          .isEqualTo(emscripten + " model.cpp -s WASM=1 -s LINKABLE=1");
    }

    @Test
    void whenCommandWithInclude() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);

      assertThat(builder.toString())
          .isEqualTo(
              emscripten + " model.cpp -I\"" + INCLUDE_ARMADILLO.toString() + "\"");
    }

    @Test
    void whenCommandWithMultipleIncludes() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);
      builder.include(INCLUDE_BLAS);

      assertThat(builder.toString())
          .isEqualTo(
              emscripten + " model.cpp -I\"" + INCLUDE_ARMADILLO.toString() + "\" -I\""
                  + INCLUDE_BLAS.toString() + "\"");
    }

    @Test
    void whenCommandWithOptimization() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.setOptimization(SOME_LEVEL);

      assertThat(builder.toString()).isEqualTo(emscripten + " model.cpp -O3");
    }

    @Test
    void whenCommandWithBind() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.setBind(true);

      assertThat(builder.toString()).isEqualTo(emscripten + " model.cpp --bind");
    }

    @Test
    void whenFullCommand() {
      EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder(emscripten, file);

      builder.include(INCLUDE_ARMADILLO);
      builder.addOption(WASM_OPTION);
      builder.setOptimization(SOME_LEVEL);
      builder.setBind(true);
      builder.setStd("c++11");
      builder.setOutput("module.js");

      assertThat(builder.toString())
          .isEqualTo(
              emscripten + " model.cpp -o module.js -I\"" + INCLUDE_ARMADILLO.toString()
                  + "\" -s WASM=1 -O3 --bind -std=c++11");
    }
  }
}