package de.monticore.lang.monticar.emam2wasm;

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

class ModelNameParserTest {

  private static final Path MODEL_PATH = Paths.get("src/test/resources/parser/models");
  private static final Path EMPTY_MODEL = MODEL_PATH.resolve("EmptyModel.emam");
  private static final Path COMPONENT_WITH_INNER_COMPONENT = MODEL_PATH
      .resolve("ComponentWithInnerComponent.emam");
  private static final Path SUB_COMPONENT = MODEL_PATH.resolve("SubComponent.emam");
  private static final Path SUPER_COMPONENT = MODEL_PATH.resolve("SuperComponent.emam");

  @Nested
  class ParseModelName {

    @Nested
    class WhenIOExceptionIsThrown {

      private ModelNameParser parser;

      @BeforeEach
      void setUp() throws IOException {
        EmbeddedMontiArcMathParser emaParser = mock(EmbeddedMontiArcMathParser.class);
        when(emaParser.parse(any(Reader.class))).thenThrow(IOException.class);

        parser = new ModelNameParser(emaParser);
      }

      @Test
      void shouldThrowUncheckedIOException() {
        assertThatExceptionOfType(UncheckedIOException.class)
            .isThrownBy(() -> parser.parseModelName(""));
      }
    }

    @Nested
    @TestInstance(Lifecycle.PER_CLASS)
    class WithEmbeddedMontiArcModel {

      private ModelNameParser parser;

      @BeforeEach
      void setUp() {
        parser = new ModelNameParser(new EmbeddedMontiArcMathParser());
      }

      @ParameterizedTest
      @MethodSource("testData")
      void shouldReturnModelName(String model, String name) {
        assertThat(parser.parseModelName(model)).isEqualTo(name);
      }

      private Stream<Arguments> testData() {
        return Stream.of(
            Arguments.of(readFromFile(EMPTY_MODEL), "EmptyModel"),
            Arguments
                .of(readFromFile(COMPONENT_WITH_INNER_COMPONENT), "ComponentWithInnerComponent"),
            Arguments.of(readFromFile(SUB_COMPONENT), "SubComponent"),
            Arguments.of(readFromFile(SUPER_COMPONENT), "SuperComponent")
        );
      }

      private String readFromFile(Path path) {
        TextFile file = new TextFile(path);
        return file.read();
      }
    }
  }
}