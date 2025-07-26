/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.wasm;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class WasmJsNameProviderTest {

  private static final Path NULL_PATH = null;
  private static final Path SOME_CPP_FILE_PATH = Paths.get("some_file.cpp");
  private static final Path SOME_CPP_RELATIVE_PATH = Paths.get("some/directory/some_file.cpp");
  private static final Path SOME_CPP_ABSOLUTE_PATH = Paths.get("some/directory/some_file.cpp")
      .toAbsolutePath();
  private static final String EXPECTED_NAME = "some_file";
  private static final Path EXPECTED_FILE_PATH = Paths.get("some_file.js");

  private WasmJsNameProvider nameProvider;

  @BeforeEach
  void setUp() {
    nameProvider = new WasmJsNameProvider();
  }

  @Nested
  class GetName {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenPathIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getName(NULL_PATH));
      }
    }

    @Nested
    class ShouldReturnBaseName {

      @Test
      void whenOnlyFileName() {
        assertThat(nameProvider.getName(SOME_CPP_FILE_PATH)).isEqualTo(EXPECTED_NAME);
      }

      @Test
      void whenRelativePath() {
        assertThat(nameProvider.getName(SOME_CPP_RELATIVE_PATH)).isEqualTo(EXPECTED_NAME);
      }

      @Test
      void whenAbsolutePath() {
        assertThat(nameProvider.getName(SOME_CPP_ABSOLUTE_PATH)).isEqualTo(EXPECTED_NAME);

      }
    }
  }

  @Nested
  class GetFilePath {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenPathIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getName(NULL_PATH));
      }
    }

    @Nested
    class ShouldReturnFilePath {

      @Test
      void whenOnlyFileName() {
        assertThat(nameProvider.getFilePath(SOME_CPP_FILE_PATH)).isEqualTo(EXPECTED_FILE_PATH);
      }

      @Test
      void whenRelativePath() {
        assertThat(nameProvider.getFilePath(SOME_CPP_RELATIVE_PATH)).isEqualTo(EXPECTED_FILE_PATH);
      }

      @Test
      void whenAbsolutePath() {
        assertThat(nameProvider.getFilePath(SOME_CPP_ABSOLUTE_PATH)).isEqualTo(EXPECTED_FILE_PATH);

      }
    }

  }

}
