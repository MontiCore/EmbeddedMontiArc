/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class EmscriptenCommandTest {

  private static final Shell NULL_SHELL = null;
  private static final String SOME_EMSCRIPTEN_CALL = "emscripten";
  private static final Shell SHELL = Shell.BASH;
  private static final String NULL_CALL = null;
  private static final String EMPTY_CALL = "";
  private static final String OPTION_ONE = "one";
  private static final String OPTION_TWO = "two";
  private static final String OPTION_THREE = "three";
  private static final String[] EXPECTED_COMMAND_WITHOUT_OPTIONS =
      {"/bin/bash", "-c", "emscripten"};
  private static final String[] EXPECTED_COMMAND_WITH_ONE_OPTION =
      {"/bin/bash", "-c", "emscripten one"};
  private static final String[] EXPECTED_COMMAND_WITH_MULTIPLE_OPTIONS =
      {"/bin/bash", "-c", "emscripten one two three"};

  @Nested
  class ShouldThrowPreconditionViolationException {

    @Test
    void whenShellIsNull() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenCommand(NULL_SHELL, SOME_EMSCRIPTEN_CALL));
    }

    @Test
    void whenEmscriptenCallIsNull() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenCommand(SHELL, NULL_CALL));
    }

    @Test
    void whenEmscriptenCallIsEmpty() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenCommand(SHELL, EMPTY_CALL));
    }
  }

  @Nested
  class ShouldJoinOptionsWithEmscriptenCall {

    private EmscriptenCommand emscriptenCommand;

    @BeforeEach
    void setUp() {
      emscriptenCommand = new EmscriptenCommand(SHELL, SOME_EMSCRIPTEN_CALL);
    }

    @Test
    void whenNoOptionsSupplied() {
      assertThat(emscriptenCommand.getCommand())
          .containsExactly(EXPECTED_COMMAND_WITHOUT_OPTIONS);
    }

    @Test
    void whenOneOptionSupplied() {
      assertThat(emscriptenCommand.getCommand(OPTION_ONE))
          .containsExactly(EXPECTED_COMMAND_WITH_ONE_OPTION);
    }

    @Test
    void whenMultipleOptionsSupplied() {
      assertThat(emscriptenCommand.getCommand(OPTION_ONE, OPTION_TWO, OPTION_THREE))
          .containsExactly(EXPECTED_COMMAND_WITH_MULTIPLE_OPTIONS);
    }
  }

}
