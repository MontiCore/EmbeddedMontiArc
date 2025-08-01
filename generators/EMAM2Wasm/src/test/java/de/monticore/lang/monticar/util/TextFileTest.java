/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.util;

import static org.assertj.core.api.Assertions.assertThat;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class TextFileTest {

  private static final Path SOME_DIRECTORY = Paths.get("src/test/resources/files");
  private static final Path EMPTY_TEXT_FILE = Paths.get("src/test/resources/files/empty.txt");
  private static final Path ONE_LINE_TEXT_FILE = Paths
      .get("src/test/resources/files/not_empty.txt");
  private static final Path MULTIPLE_LINES_TEXT_FILE = Paths
      .get("src/test/resources/files/multiple_lines.txt");

  @Nested
  class Read {

    @Nested
    class WhenFileIsEmpty {

      @Test
      void shouldReturnEmptyString() {
        TextFile textFile = new TextFile(EMPTY_TEXT_FILE);

        assertThat(textFile.read()).isEqualTo("");
      }
    }

    @Nested
    class WhenFileHasOneLine {

      @Test
      void shouldReturnStringWithoutLineBreaks() {
        TextFile textFile = new TextFile(ONE_LINE_TEXT_FILE);

        assertThat(textFile.read()).isEqualTo("Not Empty!");
      }
    }

    @Nested
    class WhenFileHasMultipleLines {

      @Test
      void shouldReturnStringWithLineBreaks() {
        TextFile textFile = new TextFile(MULTIPLE_LINES_TEXT_FILE);

        assertThat(textFile.read()).isEqualTo("First line\nSecond line\nThird line");
      }
    }

  }

}
