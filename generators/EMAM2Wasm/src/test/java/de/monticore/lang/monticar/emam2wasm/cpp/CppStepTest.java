/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.cpp;

import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class CppStepTest {

  private static final String CPP_MAIN_BASE_NAME = "main";
  private static final String CPP_EXTENSION = "cpp";
  private static final String CPP_MAIN_NAME = CPP_MAIN_BASE_NAME + "." + CPP_EXTENSION;
  private static final Path TARGET_PATH = Paths.get("target/");
  private static final Path CPP_MAIN_FILE = TARGET_PATH.resolve(CPP_MAIN_NAME);
  private static final Symbol NULL_MODEL = null;

  private CppCompiler<Symbol> cppCompiler;
  private CppNameProvider nameProvider;

  @Nested
  class CompileToCpp {

    @BeforeEach
    void setUp() {
      cppCompiler = mockCppCompiler();
      nameProvider = mockCppNameProvider();

    }

    @SuppressWarnings("unchecked")
    private CppCompiler<Symbol> mockCppCompiler() {
      CppCompiler<Symbol> cppCompiler = mock(CppCompiler.class);
      when(cppCompiler.compile(any(), any(), any())).thenReturn(CPP_MAIN_FILE);
      return cppCompiler;
    }

    private CppNameProvider mockCppNameProvider() {
      CppNameProvider nameProvider = mock(CppNameProvider.class);
      when(nameProvider.getFileExtension()).thenReturn(CPP_EXTENSION);
      when(nameProvider.getName(any())).thenReturn(CPP_MAIN_BASE_NAME);
      when(nameProvider.getFilePath(any())).thenCallRealMethod();
      return nameProvider;
    }

    private Symbol mockSymbol() {
      return mock(Symbol.class);
    }

    @Nested
    class WhenModelIsNull {

      @Test
      void shouldThrowPreconditionViolationException() {
        CppStep<Symbol> cppStep = new CppStep<>(cppCompiler, TARGET_PATH, nameProvider);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> cppStep.compileToCpp(NULL_MODEL));
      }
    }

    @Nested
    class WhenModelIsNotNull {

      @Test
      void shouldCompileCpp() {
        CppStep<Symbol> cppStep = new CppStep<>(cppCompiler, TARGET_PATH, nameProvider);
        Symbol symbol = mockSymbol();

        cppStep.compileToCpp(symbol);

        verify(cppCompiler).compile(symbol, TARGET_PATH, CPP_MAIN_NAME);
      }
    }

  }

}
