/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class JsWrapperNameProviderTest {

  private static final Symbol NULL_SYMBOL = null;
  private static final String SOME_SYMBOL_NAME = "ModelName";
  private static final Path EXPECTED_SYMBOL_PATH = Paths.get("ModelName_wrapper.js");

  @Nested
  class GetName {

    @Nested
    class WhenSymbolIsNull {

      @Test
      void shouldThrowPreconditionViolationException() {
        JsWrapperNameProvider nameProvider = new JsWrapperNameProvider();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getName(NULL_SYMBOL));
      }
    }
  }

  @Nested
  class GetFilePath {

    @Nested
    class WhenSymbolIsNull {

      @Test
      void shouldThrowPreconditionViolationException() {
        JsWrapperNameProvider nameProvider = new JsWrapperNameProvider();

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getFilePath(NULL_SYMBOL));
      }
    }

    @Nested
    class WhenSymbolIsNotNull {

      private Symbol model;

      @BeforeEach
      void setUp() {
        model = mock(Symbol.class);
        when(model.getName()).thenReturn(SOME_SYMBOL_NAME);
      }

      @Test
      void shouldReturnFilePathWithExtension() {
        JsWrapperNameProvider nameProvider = new JsWrapperNameProvider();

        assertThat(nameProvider.getFilePath(model)).isEqualTo(EXPECTED_SYMBOL_PATH);
      }
    }
  }

}
