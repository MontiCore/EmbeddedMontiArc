/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.cpp;

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

class CppMainNameProviderTest {

  private static final Symbol NULL_SYMBOL = null;
  private static final String SOME_SYMBOL_NAME = "someSymbolName";
  private static final String EXPECTED_SYMBOL_NAME = SOME_SYMBOL_NAME;
  private static final Path EXPECTED_FILE_PATH = Paths.get("someSymbolName.cpp");

  private CppMainNameProvider nameProvider;
  private Symbol symbol;

  @BeforeEach
  void setUp() {
    nameProvider = new CppMainNameProvider();
    symbol = mock(Symbol.class);
    when(symbol.getName()).thenReturn(SOME_SYMBOL_NAME);
  }

  @Nested
  class GetName {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenSymbolIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getName(NULL_SYMBOL));
      }
    }

    @Nested
    class ShouldReturnSymbolName {

      @Test
      void whenSymbolIsNotNull() {
        assertThat(nameProvider.getName(symbol)).isEqualTo(EXPECTED_SYMBOL_NAME);
      }
    }
  }

  @Nested
  class GetFilePath {

    @Nested
    class ShouldThrowPreconditionViolationException {

      @Test
      void whenSymbolIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> nameProvider.getName(NULL_SYMBOL));
      }
    }

    @Nested
    class ShouldReturnFilePath {

      @Test
      void whenOnlyFileName() {
        assertThat(nameProvider.getFilePath(symbol)).isEqualTo(
            CppMainNameProviderTest.EXPECTED_FILE_PATH);
      }
    }

  }

}
