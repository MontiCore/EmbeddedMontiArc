/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class OptionTest {

  private static final String NULL_STRING = null;
  private static final String EMPTY_STRING = "";
  private static final String ANY_STRING = "some_option";
  private static final boolean ANY_BOOLEAN_VALUE = false;

  @Nested
  class ShouldThrowException {

    @Test
    void whenInitializedWithNullString() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new Option(NULL_STRING, ANY_BOOLEAN_VALUE));
    }

    @Test
    void whenInitializedWithEmptyString() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new Option(EMPTY_STRING, ANY_BOOLEAN_VALUE));
    }
  }

  @Nested
  class ToString {

    @Nested
    class ShouldGiveOptionString {

      @Nested
      class WhenGivenAnyString {

        @Test
        void whenOptionNotEnabled() {
          Option option = new Option(ANY_STRING, false);

          assertThat(option.toString()).isEqualTo("some_option=0");
        }

        @Test
        void whenOptionEnabled() {
          Option option = new Option(ANY_STRING, true);

          assertThat(option.toString()).isEqualTo("some_option=1");
        }
      }
    }
  }
}
