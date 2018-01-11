package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.quicktheories.QuickTheory.qt;
import static org.quicktheories.generators.SourceDSL.integers;
import static org.quicktheories.generators.SourceDSL.strings;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.regex.Pattern;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class CppGeneratorTest {

  private static final Path RESOLVING_BASE_DIR = Paths.get("src/test/resources/cppfilegeneration");
  private static final String ANY_MODEL = "models.scalarPort";

  private static final PortSymbol NULL_PORT = null;
  private static final String NULL_STRING = null;
  private static final String EMPTY_STRING = "";
  private static final String LOWERCASE_SINGLE_CHARACTER = "a";
  private static final String UPPERCASE_SINGLE_CHARACTER = "A";
  private static final String FIRST_LOWERCASE_STRING = "aSdF";
  private static final String FIRST_UPPERCASE_STRING = "ASdF";

  private static final Pattern VARIABLE_NAME_PATTERN = Pattern
      .compile("^[a-zA-Z_$][a-zA-Z_$0-9]*$");
  private static final int CODEPOINT_LOWERCASE_A = 97;
  private static final int CODEPOINT_LOWERCASE_Z = 122;

  private static final String NOT_ARRAY_PORT_NAME = "inport";

  private static final String ANY_FULL_NAME = "some.full.name";
  private static final String EXPECTED_CPP_CLASS_NAME = "some_full_name";

  private boolean isValidVariableName(String name) {
    return VARIABLE_NAME_PATTERN.matcher(name).matches();
  }

  @Nested
  class Generate {

    ExpandedComponentInstanceSymbol model;

    @BeforeEach
    void setUp() {
      ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
      Resolver resolver = resolverFactory.get();
      model = resolver.getExpandedComponentInstanceSymbol(ANY_MODEL);
    }

    @Test
    void shouldGenerateCpp() throws IOException, TemplateException {
      TemplateProcessor templateProcessor = mock(TemplateProcessor.class);
      CppGenerator cppGenerator = new CppGenerator(templateProcessor);

      cppGenerator.generate(model);

      verify(templateProcessor).process(any());
    }
  }

  @Nested
  class IsArray {

    @Nested
    class ShouldReturnTrue {

      @Test
      void withArrayName() {
        qt().forAll(strings().betweenCodePoints(CODEPOINT_LOWERCASE_A, CODEPOINT_LOWERCASE_Z)
                .ofLengthBetween(1, 10)
            , integers().allPositive())
            .assuming((s, i) -> isValidVariableName(s))
            .as((s, i) -> s + "[" + i + "]")
            .check(CppGenerator::isArray);
      }
    }

    @Nested
    class ShouldReturnFalse {

      @Test
      void withInvalidVariableName() {
        qt().forAll(strings().basicLatinAlphabet().ofLengthBetween(1, 10))
            .assuming(s -> !isValidVariableName(s))
            .check(s -> !CppGenerator.isArray(s));
      }

      @Test
      void withNotArrayVariableName() {
        qt().forAll(strings().betweenCodePoints(CODEPOINT_LOWERCASE_A, CODEPOINT_LOWERCASE_Z)
            .ofLengthBetween(1, 10))
            .assuming(s -> isValidVariableName(s))
            .check(s -> !CppGenerator.isArray(s));
      }
    }
  }

  @Nested
  class GetArrayName {

    @Nested
    class ShouldThrowIllegalArgumentException {

      @Test
      void withNotArrayVariableName() {
        assertThatExceptionOfType(IllegalArgumentException.class)
            .isThrownBy(() -> CppGenerator.getArrayName(NOT_ARRAY_PORT_NAME));
      }
    }

    @Nested
    class ShouldReturnArrayName {

      @Test
      void withValidPortName() {
        qt().forAll(strings().betweenCodePoints(CODEPOINT_LOWERCASE_A, CODEPOINT_LOWERCASE_Z)
                .ofLengthBetween(1, 10)
            , integers().allPositive())
            .assuming((s, i) -> isValidVariableName(s))
            .asWithPrecursor((variableName, number) -> variableName + "[" + number + "]")
            .check((variableName, number, portName) ->
                variableName.equals(CppGenerator.getArrayName(portName)));
      }
    }
  }

  @Nested
  class GetCppClassName {

    @Nested
    class WithNameWithoutDots {

      @Test
      void shouldReturnSameName() {
        qt().forAll(strings().basicLatinAlphabet().ofLengthBetween(1, 10))
            .assuming(name -> !name.contains("."))
            .check(name -> name.equals(CppGenerator.getCppClassName(name)));
      }
    }

    @Nested
    class WithComponentFullName {

      @Test
      void shouldReplaceDots() {
        assertThat(CppGenerator.getCppClassName(ANY_FULL_NAME)).isEqualTo(EXPECTED_CPP_CLASS_NAME);
      }
    }
  }

  @Nested
  class GetGetterMethodName {

    @Nested
    class ShouldThrowException {

      @Test
      void whenPortIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getGetterMethodName(NULL_PORT));
      }

      @Test
      void whenPortNameIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getGetterMethodName(NULL_STRING));
      }

      @Test
      void whenPortNameIsEmpty() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getGetterMethodName(EMPTY_STRING));
      }
    }

    @Nested
    class ShouldCapitalizeName {

      @Test
      void whenGivenLowercaseSingleCharacter() {
        assertThat(CppGenerator.getGetterMethodName(LOWERCASE_SINGLE_CHARACTER)).isEqualTo("getA");
      }

      @Test
      void whenGivenUppercaseSingleCharacter() {
        assertThat(CppGenerator.getGetterMethodName(UPPERCASE_SINGLE_CHARACTER)).isEqualTo("getA");
      }

      @Test
      void whenGivenFirstLowercaseString() {
        assertThat(CppGenerator.getGetterMethodName(FIRST_LOWERCASE_STRING)).isEqualTo("getASdF");
      }

      @Test
      void whenGivenFirstUppercaseString() {
        assertThat(CppGenerator.getGetterMethodName(FIRST_UPPERCASE_STRING)).isEqualTo("getASdF");
      }
    }
  }

  @Nested
  class GetSetterMethodName {

    @Nested
    class ShouldThrowException {

      @Test
      void whenPortIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getSetterMethodName(NULL_PORT));
      }

      @Test
      void whenPortNameIsNull() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getSetterMethodName(NULL_STRING));
      }

      @Test
      void whenPortNameIsEmpty() {
        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> CppGenerator.getSetterMethodName(EMPTY_STRING));
      }
    }

    @Nested
    class ShouldCapitalizeName {

      @Test
      void whenGivenLowercaseSingleCharacter() {
        assertThat(CppGenerator.getSetterMethodName(LOWERCASE_SINGLE_CHARACTER)).isEqualTo("setA");
      }

      @Test
      void whenGivenUppercaseSingleCharacter() {
        assertThat(CppGenerator.getSetterMethodName(UPPERCASE_SINGLE_CHARACTER)).isEqualTo("setA");
      }

      @Test
      void whenGivenFirstLowercaseString() {
        assertThat(CppGenerator.getSetterMethodName(FIRST_LOWERCASE_STRING)).isEqualTo("setASdF");
      }

      @Test
      void whenGivenFirstUppercaseString() {
        assertThat(CppGenerator.getSetterMethodName(FIRST_UPPERCASE_STRING)).isEqualTo("setASdF");
      }
    }
  }

}