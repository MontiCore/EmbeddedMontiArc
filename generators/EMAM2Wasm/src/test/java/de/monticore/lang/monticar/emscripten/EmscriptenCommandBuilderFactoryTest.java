/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class EmscriptenCommandBuilderFactoryTest {

  private static final Path FILE = Paths.get("model.cpp");
  private static final Path OTHER_FILE = Paths.get("other_model.cpp");
  private static final Optimization OPTIMIZATION = Optimization.O0;
  private static final Optimization OTHER_OPTIMIZATION = Optimization.O3;
  private static final boolean BIND = true;
  private static final boolean OTHER_BIND = false;
  private static final String OUTPUT = "module.js";
  private static final String OTHER_OUTPUT = "other_module.js";
  private static final Path INCLUDE_LIBRARY = Paths.get("./some_library");
  private static final Path OTHER_INCLUDE_LIBRARY = Paths.get("./other_library");
  private static final Path SOME_LIBRARY = Paths.get("./lib");
  private static final Path OTHER_LIBRARY = Paths.get("./some/other/lib");
  private static final Option OPTION = new Option("some_option", true);
  private static final Option OTHER_OPTION = new Option("other_option", false);
  private static final String FLAG = "some_flag";
  private static final String OTHER_FLAG = "other_flag";
  private static final String EMSCRIPTEN = "emscripten";

  private Emscripten emscripten;
  private Emscripten otherEmscripten;

  @BeforeEach
  void setUp() {
    Path emscriptenPath = mock(Path.class);
    when(emscriptenPath.toAbsolutePath()).thenReturn(emscriptenPath);
    when(emscriptenPath.normalize()).thenReturn(emscriptenPath);
    when(emscriptenPath.toString()).thenReturn(EMSCRIPTEN);
    emscripten = new EmscriptenBinary(emscriptenPath);

    otherEmscripten = mock(Emscripten.class);
  }

  @Nested
  class WhenNoDefaultValuesSet {

    private void setCommandValues(EmscriptenCommandBuilder commandBuilder) {
      commandBuilder.setEmscripten(emscripten);
      commandBuilder.setFile(FILE);
      commandBuilder.setOptimization(OPTIMIZATION);
      commandBuilder.setBind(BIND);
      commandBuilder.setOutput(OUTPUT);
      commandBuilder.include(INCLUDE_LIBRARY);
      commandBuilder.addOption(OPTION);
    }

    @Nested
    class ShouldProduceRegularBuilder {

      @Test
      void whenNoBuilderValuesSet() {
        EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();
        EmscriptenCommandBuilder factoryBuiltBuilder = builderFactory.getBuilder();
        EmscriptenCommandBuilder regularBuilder = new EmscriptenCommandBuilder();

        assertThat(factoryBuiltBuilder).isEqualTo(regularBuilder);
      }

      @Test
      void whenAllBuilderValuesSet() {
        EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();
        EmscriptenCommandBuilder factoryBuiltBuilder = builderFactory.getBuilder();
        EmscriptenCommandBuilder regularBuilder = new EmscriptenCommandBuilder();

        setCommandValues(factoryBuiltBuilder);
        setCommandValues(regularBuilder);

        assertThat(factoryBuiltBuilder.toString()).isEqualTo(regularBuilder.toString());
      }

    }
  }

  @Nested
  class WhenNonNullValueSet {

    @Nested
    class WhenCallingBuilderMethod {

      @Nested
      class ShouldThrowException {

        @Test
        void setEmscripten() {
          EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();

          builderFactory.setEmscripten(emscripten);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          assertThatExceptionOfType(UnsupportedOperationException.class)
              .isThrownBy(() -> builder.setEmscripten(otherEmscripten));
        }

        @Test
        void setFile() {
          EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();

          builderFactory.setFile(FILE);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          assertThatExceptionOfType(UnsupportedOperationException.class)
              .isThrownBy(() -> builder.setFile(OTHER_FILE));
        }

        @Test
        void setOptimization() {
          EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();

          builderFactory.setOptimization(OPTIMIZATION);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          assertThatExceptionOfType(UnsupportedOperationException.class)
              .isThrownBy(() -> builder.setOptimization(OTHER_OPTIMIZATION));
        }

        @Test
        void setBind() {
          EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();

          builderFactory.setBind(BIND);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          assertThatExceptionOfType(UnsupportedOperationException.class)
              .isThrownBy(() -> builder.setBind(OTHER_BIND));
        }

        @Test
        void setOutput() {
          EmscriptenCommandBuilderFactory builderFactory = new EmscriptenCommandBuilderFactory();

          builderFactory.setOutput(OUTPUT);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          assertThatExceptionOfType(UnsupportedOperationException.class)
              .isThrownBy(() -> builder.setOutput(OTHER_OUTPUT));
        }
      }

      @Nested
      class ShouldAddMoreValues {

        EmscriptenCommandBuilderFactory builderFactory;

        @BeforeEach
        void setUpFactory() {
          builderFactory = new EmscriptenCommandBuilderFactory();
          builderFactory.setEmscripten(emscripten);
          builderFactory.setFile(FILE);
        }

        @Test
        void include() {
          builderFactory.include(INCLUDE_LIBRARY);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          builder.include(OTHER_INCLUDE_LIBRARY);
          assertThat(builder.toString())
              .isEqualTo(String.format("emscripten model.cpp -I\"%s\" -I\"%s\"",
                  INCLUDE_LIBRARY.toString(), OTHER_INCLUDE_LIBRARY));
        }

        @Test
        void addLibrary() {
          builderFactory.addLibrary(SOME_LIBRARY);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          builder.addLibrary(OTHER_LIBRARY);
          assertThat(builder.toString())
              .isEqualTo(String.format("emscripten model.cpp -L\"%s\" -L\"%s\"",
                  SOME_LIBRARY.toString(), OTHER_LIBRARY));
        }

        @Test
        void addOption() {
          builderFactory.addOption(OPTION);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          builder.addOption(OTHER_OPTION);
          assertThat(builder.toString())
              .isEqualTo("emscripten model.cpp -s some_option=1 -s other_option=0");
        }

        @Test
        void addFlag() {
          builderFactory.addFlag(FLAG);
          EmscriptenCommandBuilder builder = builderFactory.getBuilder();

          builder.addFlag(OTHER_FLAG);
          assertThat(builder.toString())
              .isEqualTo("emscripten model.cpp -some_flag -other_flag");
        }
      }
    }
  }


}
