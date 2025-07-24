/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.junit;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.util.TextFile;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

public class TemporaryDirectoryExtensionTest {

  private static Set<Path> tempDirectories = new HashSet<>();

  @AfterAll
  static void temporaryDirectoryShouldBeDeleted() {
    tempDirectories.forEach(tempDir -> assertThat(tempDir).doesNotExist());
    tempDirectories.clear();
  }

  private void assertThatDirectoryIsEmpty(Path dir) {
    assertThat(dir).isNotNull();
    assertThat(dir).isDirectory();

    try (DirectoryStream<Path> dirStream = Files.newDirectoryStream(dir)) {
      if (dirStream.iterator().hasNext()) {
        throw new AssertionError(
            String.format("Expecting path:\n  <%s>\nto be empty", dir.toString()));
      }
    } catch (IOException e) {
      throw new AssertionError("Check threw Exception", e);
    }
  }

  @Nested
  @ExtendWith(TemporaryDirectoryExtension.class)
  class WhenExtendingClass {

    @Test
    void temporaryDirectoryShouldExist(Path tempDir) {
      tempDirectories.add(tempDir);

      assertThat(tempDir).isDirectory();
    }

    @ParameterizedTest
    @CsvSource({
        "test.txt, 1",
        "test.txt, 1",
        "test.txt, test",
        "abc.txt, def"})
    void temporaryDirectoryShouldBeEmpty(String fileName, String fileContent, Path tempDir) {
      tempDirectories.add(tempDir);

      assertThatDirectoryIsEmpty(tempDir);
      TextFile file = new TextFile(tempDir.resolve(fileName));
      file.write(fileContent);
    }
  }

  @Nested
  class WhenExtendingMethod {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void temporaryDirectoryShouldExist(Path tempDir) {
      tempDirectories.add(tempDir);

      assertThat(tempDir).isDirectory();
    }

    @ParameterizedTest
    @CsvSource({
        "test.txt, 1",
        "test.txt, 1",
        "test.txt, test",
        "abc.txt, def"})
    @ExtendWith(TemporaryDirectoryExtension.class)
    void temporaryDirectoryShouldBeEmpty(String fileName, String fileContent, Path tempDir) {
      tempDirectories.add(tempDir);

      assertThatDirectoryIsEmpty(tempDir);
      TextFile file = new TextFile(tempDir.resolve(fileName));
      file.write(fileContent);
    }
  }
}
