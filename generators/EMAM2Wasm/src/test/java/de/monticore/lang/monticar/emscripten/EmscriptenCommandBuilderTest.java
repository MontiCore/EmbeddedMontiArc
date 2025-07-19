/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

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

  private static final Path FILE = Paths.get("model.cpp");
  private static final Path INCLUDE_ARMADILLO = Paths.get("./armadillo/include");
  private static final Path INCLUDE_BLAS = Paths.get("./blas");
  private static final Option WASM_OPTION = new Option("WASM", true);
  private static final Option LINKABLE_OPTION = new Option("LINKABLE", true);
  private static final Optimization SOME_LEVEL = Optimization.O3;
  private static final String EMPTY_STRING = "";
  private static final String SOME_FLAG = "DARMA_DONT_USE_WRAPPER";
  private static final String EMSCRIPTEN = "emscripten";
  private static final Path SOME_LIBRARY = Paths.get("./lib");
  private static final Path OTHER_LIBRARY = Paths.get("./some/other/lib");

  private Emscripten emscripten;

  @BeforeEach
  void setUp() {
    Path emscriptenPath = mock(Path.class);
    when(emscriptenPath.toAbsolutePath()).thenReturn(emscriptenPath);
    when(emscriptenPath.normalize()).thenReturn(emscriptenPath);
    when(emscriptenPath.toString()).thenReturn(EMSCRIPTEN);
    emscripten = new EmscriptenBinary(emscriptenPath);
  }
  
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
  class SetEmscripten {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenEmscriptenIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.setEmscripten(null));
      }
    }
  }

  @Nested
  class SetFile {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenFileIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.setFile(null));
      }
    }
  }

  @Nested
  class SetOptimization {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenOptimizationIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.setOptimization(null));
      }
    }
  }

  @Nested
  class SetOutput {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenOutputIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.setOutput(null));
      }
    }
  }

  @Nested
  class SetReferenceOutputDir {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenEmscriptenIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.setReferenceOutputDir(null));
      }
    }
  }

  @Nested
  class Include {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenIncludeIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.include(null));
      }
    }
  }

  @Nested
  class AddLibrary {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenLibraryIsNull() {
        EmscriptenCommandBuilder builder = new EmscriptenCommandBuilder();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> builder.addLibrary(null));
      }
    }
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
        builder.setEmscripten(emscripten);

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
        builder.setEmscripten(emscripten);
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
      void whenCommandWithLibrary() {
        builder.addLibrary(SOME_LIBRARY);

        assertThat(builder.toList())
            .isEqualTo(
                listof(EMSCRIPTEN, "model.cpp",
                    "-L\"" + SOME_LIBRARY.toString() + "\""));
      }

      @Test
      void whenCommandWithMultipleLibraries() {
        builder.addLibrary(SOME_LIBRARY);
        builder.addLibrary(OTHER_LIBRARY);

        assertThat(builder.toList())
            .isEqualTo(
                listof(EMSCRIPTEN, "model.cpp",
                    "-L\"" + SOME_LIBRARY.toString() + "\"",
                    "-L\"" + OTHER_LIBRARY.toString() + "\""));
      }

      @Test
      void whenCommandWithFlag() {
        builder.addFlag("DARMA_DONT_USE_WRAPPER");

        assertThat(builder.toList()).isEqualTo(
            listof(EMSCRIPTEN, "model.cpp", "-" + SOME_FLAG));
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
        builder.addLibrary(SOME_LIBRARY);
        builder.addOption(WASM_OPTION);
        builder.setOptimization(SOME_LEVEL);
        builder.setBind(true);

        assertThat(builder.toList())
            .isEqualTo(listof(EMSCRIPTEN, "model.cpp",
                "-I\"" + INCLUDE_ARMADILLO.toString() + "\"",
                "-L\"" + SOME_LIBRARY.toString() + "\"",
                "-s", "WASM=1", "-O3", "--bind"));
      }

      @Test
      void whenReferenceDirectoryPresent() {
        builder.setReferenceOutputDir(Paths.get("src"));
        builder.include(INCLUDE_ARMADILLO);

        assertThat(builder.toList()).isEqualTo(listof(
            EMSCRIPTEN,
            Paths.get("../").resolve("model.cpp").toString(),
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
        builder.setEmscripten(emscripten);

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
        builder.setEmscripten(emscripten);
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
      void whenCommandWithLibrary() {
        builder.addLibrary(SOME_LIBRARY);

        assertThat(builder.toString())
            .isEqualTo(EMSCRIPTEN + " model.cpp -L\"" + SOME_LIBRARY.toString() + "\"");
      }

      @Test
      void whenCommandWithMultipleLibraries() {
        builder.addLibrary(SOME_LIBRARY);
        builder.addLibrary(OTHER_LIBRARY);

        assertThat(builder.toString())
            .isEqualTo(EMSCRIPTEN + " model.cpp " +
                "-L\"" + SOME_LIBRARY.toString() + "\" " +
                "-L\"" + OTHER_LIBRARY.toString() + "\"");
      }

      @Test
      void whenCommandWithFlag() {
        builder.addFlag(SOME_FLAG);

        assertThat(builder.toString())
            .isEqualTo(EMSCRIPTEN + " model.cpp " + "-" + SOME_FLAG);
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
        builder.addLibrary(SOME_LIBRARY);
        builder.addOption(WASM_OPTION);
        builder.setOptimization(SOME_LEVEL);
        builder.setBind(true);
        builder.setOutput("module.js");
        builder.addFlag(SOME_FLAG);

        assertThat(builder.toString())
            .isEqualTo(
                EMSCRIPTEN + " model.cpp -o module.js "
                    + "-I\"" + INCLUDE_ARMADILLO.toString() + "\" "
                    + "-L\"" + SOME_LIBRARY.toString() + "\" "
                    + "-s WASM=1 -DARMA_DONT_USE_WRAPPER -O3 --bind");
      }
    }
  }
}
