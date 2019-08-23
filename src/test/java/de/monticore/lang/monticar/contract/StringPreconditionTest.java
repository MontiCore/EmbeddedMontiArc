/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotEmpty;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class StringPreconditionTest {

  private static final Class<Precondition.PreconditionViolationException> VIOLATION_EXCEPTION_TYPE
      = Precondition.PreconditionViolationException.class;
  private static final String VIOLATION_MESSAGE = "VIOLATION MESSAGE";

  private static final String NULL_STRING = null;
  private static final String EMPTY_STRING = "";
  private static final String BLANK_STRING = " \t ";
  private static final String NOT_BLANK_STRING = "SOME STRING";

  @Nested
  class RequiresNotEmpty {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullStringIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(NULL_STRING));
      }

      @Test
      void whenEmptyStringIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(EMPTY_STRING));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullStringIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(NULL_STRING,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyStringIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(EMPTY_STRING,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenBlankStringIsGiven() {
        assertThat(requiresNotEmpty(BLANK_STRING)).isEqualTo(BLANK_STRING);
      }

      @Test
      void whenNotBlankStringIsGiven() {
        assertThat(requiresNotEmpty(NOT_BLANK_STRING)).isEqualTo(NOT_BLANK_STRING);
      }
    }
  }

  @Nested
  class RequiresNotBlank {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullStringIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotBlank(NULL_STRING));
      }

      @Test
      void whenEmptyStringIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotBlank(EMPTY_STRING));
      }

      @Test
      void whenBlankStringIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotBlank(BLANK_STRING));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullStringIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotBlank(NULL_STRING,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyStringIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotBlank(EMPTY_STRING,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenBlankStringIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotBlank(BLANK_STRING,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenNotBlankStringIsGiven() {
        assertThat(requiresNotBlank(NOT_BLANK_STRING)).isEqualTo(NOT_BLANK_STRING);
      }
    }
  }

}
