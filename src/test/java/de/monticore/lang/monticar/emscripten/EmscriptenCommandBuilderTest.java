package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import nl.jqno.equalsverifier.EqualsVerifier;
import nl.jqno.equalsverifier.Warning;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

//Path variables have to be used in assertions to build expected string
//because of unix vs. windows file seperators
class EmscriptenCommandBuilderTest {

  private static final String EMSCRIPTEN = "./emscripten";
  private static final Path FILE = Paths.get("model.cpp");
  private static final Path INCLUDE_ARMADILLO = Paths.get("./armadillo/include");
  private static final Path INCLUDE_BLAS = Paths.get("./blas");
  private static final Option WASM_OPTION = new Option("WASM", true);
  private static final Option LINKABLE_OPTION = new Option("LINKABLE", true);
  private static final Optimization SOME_LEVEL = Optimization.O3;
  private static final String EMPTY_STRING = "";

  @SafeVarargs
  private final <T> List<T> listof(T... elements) {
    return Arrays.asList(elements);
  }

  @Test
  void equalsShouldAdhereToSpecification() {
    EqualsVerifier.forClass(EmscriptenCommandBuilder.class).suppress(Warning.NONFINAL_FIELDS)
        .verify();
  }

  @Nested
  class ToList {

    @Nested
    class ShouldThrowException {

      @Test
      void whenEmscriptenAndFileAreNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(builder::toList);
      }

      @Test
      void whenEmscriptenIsNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();
        builder.setFile(FILE);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(builder::toList);
      }

      @Test
      void whenFileIsNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();
        builder.setEmscripten(EMSCRIPTEN);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(builder::toList);
      }
    }

    @Nested
    class ShouldReturnCommandList {

      EmscriptenCommandBuilder builder;

      @BeforeEach
      void setUp() {
        builder = new EmscriptenCommandBuilder();
        builder.setEmscripten(EMSCRIPTEN);
        builder.setFile(FILE);
      }

      @Test
      void whenSimpleCommand() {

        assertThat(builder.toList()).isEqualTo(listof(EMSCRIPTEN, "model.cpp"));
      }

      @Test
      void whenCommandWithOption() {
        builder.addOption(WASM_OPTION);

        assertThat(builder.toList())
            .isEqualTo(listof(EMSCRIPTEN, "model.cpp", "-s", "WASM=1"));
      }

      @Test
      void whenCommandWithMultipleOptions() {
        builder.addOption(WASM_OPTION);
        builder.addOption(LINKABLE_OPTION);

        assertThat(builder.toList())
            .isEqualTo(
                listof(EMSCRIPTEN, "model.cpp", "-s", "WASM=1", "-s", "LINKABLE=1"));
      }

      @Test
      void whenCommandWithInclude() {
        builder.include(INCLUDE_ARMADILLO);

        assertThat(builder.toList())
            .isEqualTo(
                listof(EMSCRIPTEN, "model.cpp",
                    "-I\"" + INCLUDE_ARMADILLO.toString() + "\""));
      }

      @Test
      void whenCommandWithMultipleIncludes() {
        builder.include(INCLUDE_ARMADILLO);
        builder.include(INCLUDE_BLAS);

        assertThat(builder.toList())
            .isEqualTo(
                listof(EMSCRIPTEN, "model.cpp",
                    "-I\"" + INCLUDE_ARMADILLO.toString() + "\"",
                    "-I\"" + INCLUDE_BLAS.toString() + "\""));
      }

      @Test
      void whenCommandWithOptimization() {
        builder.setOptimization(SOME_LEVEL);

        assertThat(builder.toList()).isEqualTo(listof(EMSCRIPTEN, "model.cpp", "-O3"));
      }

      @Test
      void whenCommandWithBind() {
        builder.setBind(true);

        assertThat(builder.toList())
            .isEqualTo(listof(EMSCRIPTEN, "model.cpp", "--bind"));
      }

      @Test
      void whenFullCommand() {
        builder.include(INCLUDE_ARMADILLO);
        builder.addOption(WASM_OPTION);
        builder.setOptimization(SOME_LEVEL);
        builder.setBind(true);

        assertThat(builder.toList())
            .isEqualTo(listof(EMSCRIPTEN, "model.cpp",
                "-I\"" + INCLUDE_ARMADILLO.toString() + "\"", "-s", "WASM=1", "-O3", "--bind"));
      }

      @Test
      void whenReferenceDirectoryPresent() {
        builder.setReferenceOutputDir(Paths.get("src"));
        builder.include(INCLUDE_ARMADILLO);

        assertThat(builder.toList()).isEqualTo(
            listof(EMSCRIPTEN, Paths.get("../").resolve("model.cpp").toString(),
                "-I\"" + Paths.get("../").resolve(INCLUDE_ARMADILLO).normalize().toString()
                    + "\""));
      }
    }
  }

  @Nested
  class ToString {

    @Nested
    class ShouldReturnEmptyString {

      @Test
      void WhenEmscriptenIsNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();
        builder.setFile(FILE);

        assertThat(builder.toString()).isEqualTo(EMPTY_STRING);
      }

      @Test
      void WhenFileIsNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();
        builder.setEmscripten(EMSCRIPTEN);

        assertThat(builder.toString()).isEqualTo(EMPTY_STRING);
      }

      @Test
      void WhenEmscriptenAndFileAreNotSet() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThat(builder.toString()).isEqualTo(EMPTY_STRING);
      }
    }

    @Nested
    class ShouldReturnCommand {

      EmscriptenCommandBuilder builder;

      @BeforeEach
      void setUp() {
        builder = new EmscriptenCommandBuilder();
        builder.setEmscripten(EMSCRIPTEN);
        builder.setFile(FILE);
      }

      @Test
      void whenSimpleCommand() {
        assertThat(builder.toString()).isEqualTo(EMSCRIPTEN + " model.cpp");
      }

      @Test
      void whenCommandWithOption() {
        builder.addOption(WASM_OPTION);

        assertThat(builder.toString()).isEqualTo(EMSCRIPTEN + " model.cpp -s WASM=1");
      }

      @Test
      void whenCommandWithMultipleOptions() {
        builder.addOption(WASM_OPTION);
        builder.addOption(LINKABLE_OPTION);

        assertThat(builder.toString())
            .isEqualTo(EMSCRIPTEN + " model.cpp -s WASM=1 -s LINKABLE=1");
      }

      @Test
      void whenCommandWithInclude() {
        builder.include(INCLUDE_ARMADILLO);

        assertThat(builder.toString())
            .isEqualTo(
                EMSCRIPTEN + " model.cpp -I\"" + INCLUDE_ARMADILLO.toString() + "\"");
      }

      @Test
      void whenCommandWithMultipleIncludes() {
        builder.include(INCLUDE_ARMADILLO);
        builder.include(INCLUDE_BLAS);

        assertThat(builder.toString())
            .isEqualTo(
                EMSCRIPTEN + " model.cpp -I\"" + INCLUDE_ARMADILLO.toString() + "\" -I\""
                    + INCLUDE_BLAS.toString() + "\"");
      }

      @Test
      void whenCommandWithOptimization() {
        builder.setOptimization(SOME_LEVEL);

        assertThat(builder.toString()).isEqualTo(EMSCRIPTEN + " model.cpp -O3");
      }

      @Test
      void whenCommandWithBind() {
        builder.setBind(true);

        assertThat(builder.toString()).isEqualTo(EMSCRIPTEN + " model.cpp --bind");
      }

      @Test
      void whenFullCommand() {
        builder.include(INCLUDE_ARMADILLO);
        builder.addOption(WASM_OPTION);
        builder.setOptimization(SOME_LEVEL);
        builder.setBind(true);
        builder.setStd("c++11");
        builder.setOutput("module.js");

        assertThat(builder.toString())
            .isEqualTo(
                EMSCRIPTEN + " model.cpp -o module.js -I\"" + INCLUDE_ARMADILLO.toString()
                    + "\" -s WASM=1 -O3 --bind -std=c++11");
      }
    }
  }
}