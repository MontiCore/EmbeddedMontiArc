/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.model;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.util.TextFile;
import java.io.IOException;
import java.io.Reader;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.api.Disabled;

@Disabled
class EmamModelNameProviderTest {

  private static final Path MODEL_PATH = Paths.get("src/test/resources/parser/models");
  private static final Path EMPTY_MODEL = MODEL_PATH.resolve("EmptyModel.emam");
  private static final Path COMPONENT_WITH_INNER_COMPONENT = MODEL_PATH
      .resolve("ComponentWithInnerComponent.emam");
  private static final Path SUB_COMPONENT = MODEL_PATH.resolve("SubComponent.emam");
  private static final Path SUPER_COMPONENT = MODEL_PATH.resolve("SuperComponent.emam");
  private static final Path MODEL_WITH_BLOCK_COMMENT = MODEL_PATH.resolve("ModelWithBlockComment.emam");
  private static final Path MODEL_WITH_LINE_COMMENT = MODEL_PATH.resolve("ModelWithLineComment.emam");
  private static final Path PACKAGE_HIERARCHY = MODEL_PATH.resolve("PackageHierarchy.txt");

  @Nested
  class ParsePackage {

    @Nested
    class WhenIOExceptionIsThrown {

      private EmamModelNameProvider parser;

      @BeforeEach
      void setUp() throws IOException {
        EmbeddedMontiArcMathParser emaParser = mock(EmbeddedMontiArcMathParser.class);
        when(emaParser.parse(any(Reader.class))).thenThrow(IOException.class);

        parser = new EmamModelNameProvider(emaParser);
      }

      @Test
      void shouldThrowUncheckedIOException() {
        assertThatExceptionOfType(UncheckedIOException.class)
            .isThrownBy(() -> parser.getPackage(""));
      }
    }

    @Nested
    @TestInstance(Lifecycle.PER_CLASS)
    class WithEmbeddedMontiArcModel {

      private EmamModelNameProvider parser;

      @BeforeEach
      void setUp() {
        parser = new EmamModelNameProvider(new EmbeddedMontiArcMathParser());
      }

      @ParameterizedTest
      @MethodSource("testData")
      void shouldReturnPackage(String model, String name) {
        assertThat(parser.getPackage(model)).isEqualTo(name);
      }

      private Stream<Arguments> testData() {
        return Stream.of(
            Arguments.of(readFromFile(EMPTY_MODEL), "models"),
            Arguments.of(readFromFile(MODEL_WITH_BLOCK_COMMENT), "models"),
            Arguments.of(readFromFile(MODEL_WITH_LINE_COMMENT), "models"),
            Arguments.of(readFromFile(PACKAGE_HIERARCHY), "a.bc.d.efghi.j")
        );
      }

    }

  }

  @Nested
  class ParseModelName {

    @Nested
    class WhenIOExceptionIsThrown {

      private EmamModelNameProvider parser;

      @BeforeEach
      void setUp() throws IOException {
        EmbeddedMontiArcMathParser emaParser = mock(EmbeddedMontiArcMathParser.class);
        when(emaParser.parse(any(Reader.class))).thenThrow(IOException.class);

        parser = new EmamModelNameProvider(emaParser);
      }

      @Test
      void shouldThrowUncheckedIOException() {
        assertThatExceptionOfType(UncheckedIOException.class)
            .isThrownBy(() -> parser.getName(""));
      }
    }

    @Nested
    @TestInstance(Lifecycle.PER_CLASS)
    class WithEmbeddedMontiArcModel {

      private EmamModelNameProvider parser;

      @BeforeEach
      void setUp() {
        parser = new EmamModelNameProvider(new EmbeddedMontiArcMathParser());
      }

      @ParameterizedTest
      @MethodSource("testData")
      void shouldReturnModelName(String model, String name) {
        assertThat(parser.getName(model)).isEqualTo(name);
      }

      private Stream<Arguments> testData() {
        return Stream.of(
            Arguments.of(readFromFile(EMPTY_MODEL), "EmptyModel"),
            Arguments
                .of(readFromFile(COMPONENT_WITH_INNER_COMPONENT), "ComponentWithInnerComponent"),
            Arguments.of(readFromFile(SUB_COMPONENT), "SubComponent"),
            Arguments.of(readFromFile(SUPER_COMPONENT), "SuperComponent"),
            Arguments.of(readFromFile(MODEL_WITH_BLOCK_COMMENT), "ModelWithBlockComment"),
            Arguments.of(readFromFile(MODEL_WITH_LINE_COMMENT), "ModelWithLineComment"),
            Arguments.of(readFromFile(PACKAGE_HIERARCHY), "PackageHierarchy")
        );
      }

    }
  }

  private String readFromFile(Path path) {
    TextFile file = new TextFile(path);
    return file.read();
  }
}
