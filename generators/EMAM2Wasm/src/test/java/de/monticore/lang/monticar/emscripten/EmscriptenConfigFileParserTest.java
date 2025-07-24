/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.lang.monticar.util.TextFile;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.AbstractMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class EmscriptenConfigFileParserTest {

  private static final Path SOME_READABLE_FILE = Paths
      .get("src/test/resources/configparser/empty.txt");
  private static final TextFile NULL_FILE = null;
  private static final String SETTING_WITH_QUOTES = " SETTING_WITH_VALUE =  'value' ";
  private static final String SETTING_WITH_SPACES = " SETTING_WITH_SAPCES =  value ";
  private static final String SETTING_WITH_VALUE = "SETTING_WITH_VALUE=value";
  private static final String SETTING_WITHOUT_VALUE = "SETTING_WITHOUT_VALUE";
  private static final String EMPTY_SETTING = "";
  private static final String NULL_SETTING = null;

  @Nested
  class ShouldThrowPreconditionViolationException {

    @Test
    void whenTextFileIsNull() {
      assertThatExceptionOfType(PreconditionViolationException.class)
          .isThrownBy(() -> new EmscriptenConfigFileParser(NULL_FILE));
    }
  }

  @Nested
  @TestInstance(Lifecycle.PER_CLASS)
  class ShouldReturnConfiguration {

    private TextFile file;

    @BeforeEach
    void setUp() {
      file = mock(TextFile.class);
      when(file.getPath()).thenReturn(SOME_READABLE_FILE);
    }

    @ParameterizedTest
    @MethodSource("keyValueConfigs")
    void testMultipleConfigurations(String stringSetting, Entry setting) {
      mockReturn(stringSetting);
      EmscriptenConfigFileParser fileParser = new EmscriptenConfigFileParser(file);

      Map<String, String> configs = fileParser.parseConfigurations();

      assertThat(configs).containsExactly(setting);
    }

    @ParameterizedTest
    @MethodSource("emptyConfigs")
    void whenLineIsNull(String stringSetting) {
      mockReturn(stringSetting);
      EmscriptenConfigFileParser fileParser = new EmscriptenConfigFileParser(file);

      Map<String, String> configs = fileParser.parseConfigurations();

      assertThat(configs).isEmpty();
    }

    private Stream<Arguments> keyValueConfigs() {
      return Stream.of(
          Arguments.of(SETTING_WITHOUT_VALUE, entryof("SETTING_WITHOUT_VALUE", null)),
          Arguments.of(SETTING_WITH_VALUE, entryof("SETTING_WITH_VALUE", "value")),
          Arguments.of(SETTING_WITH_SPACES, entryof("SETTING_WITH_SAPCES", "value")),
          Arguments.of(SETTING_WITH_QUOTES, entryof("SETTING_WITH_VALUE", "value"))
      );
    }

    private Stream<Arguments> emptyConfigs() {
      return Stream.of(
          Arguments.of(NULL_SETTING),
          Arguments.of(EMPTY_SETTING)
      );
    }

    private void mockReturn(String... returnStrings) {
      when(file.read()).thenReturn(String.join("\n", returnStrings));
      when(file.lines()).thenReturn(Stream.of(returnStrings));
    }

    private AbstractMap.SimpleEntry<String, String> entryof(String key, String value) {
      return new AbstractMap.SimpleEntry<>(key, value);
    }
  }
}
