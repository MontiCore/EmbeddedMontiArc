/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class EmscriptenBinaryTest {

  private static final Path NULL_PATH = null;
  private static final Path EMSCRIPTEN_PATH = Paths.get("emscripten");
  private static final String OPTION_ONE = "one";
  private static final String OPTION_TWO = "two";
  private static final String OPTION_THREE = "three";
  private static final String EXPECTED_EMSCRIPTEN_CALL = EMSCRIPTEN_PATH.toAbsolutePath()
      .normalize().toString();
  private static final String[] EXPECTED_COMMAND_WITH_ONE_OPTION =
      {EXPECTED_EMSCRIPTEN_CALL, OPTION_ONE};
  private static final String[] EXPECTED_COMMAND_WITH_MULTIPLE_OPTIONS =
      {EXPECTED_EMSCRIPTEN_CALL, OPTION_ONE, OPTION_TWO, OPTION_THREE};

  @Nested
  class ShouldThrowPreconditionViolationException {

    @Test
    void whenEPathToEmscriptenlIsNull() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenBinary(NULL_PATH));
    }

  }

  @Nested
  class ShouldPlaceOptionsSeparatelyInArray {

    private EmscriptenBinary emscriptenBinary;

    @BeforeEach
    void setUp() {
      emscriptenBinary = new EmscriptenBinary(EMSCRIPTEN_PATH);
    }

    @Test
    void whenNoOptionsSupplied() {
      assertThat(emscriptenBinary.getCommand())
          .containsExactly(EXPECTED_EMSCRIPTEN_CALL);
    }

    @Test
    void whenOneOptionSupplied() {
      assertThat(emscriptenBinary.getCommand(OPTION_ONE))
          .containsExactly(EXPECTED_COMMAND_WITH_ONE_OPTION);
    }

    @Test
    void whenMultipleOptionsSupplied() {
      assertThat(emscriptenBinary.getCommand(OPTION_ONE, OPTION_TWO, OPTION_THREE))
          .containsExactly(EXPECTED_COMMAND_WITH_MULTIPLE_OPTIONS);
    }
  }


}
