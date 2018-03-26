package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class EmscriptenBinaryTest {

  private static final Path NULL_BINARY = null;
  private static final Path RELATIVE_BIN_PATH = Paths.get("./emscripten");

  @Nested
  class WhenBinaryIsNull {

    @Test
    void shouldThrowPreconditionViolationException() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenBinary(NULL_BINARY));
    }
  }

  @Nested
  class WhenBinaryPathIsRelative {

    @Test
    void shouldReturnAbsolutePath() {
      EmscriptenBinary emscripten = new EmscriptenBinary(RELATIVE_BIN_PATH);

      assertThat(emscripten.getCommand())
          .containsExactly(RELATIVE_BIN_PATH.toAbsolutePath().toString());
    }
  }

}