/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.util;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class FileUtilsTest {

  private static final String FILE_NAME = "file.txt";
  private static final String DIR_IN_NON_EMPTY_DIR_NAME = "another_dir";
  private static final String FILE_IN_NON_EMPTY_DIR_NAME = "file.txt";
  private static final String NON_EXISTING_DIR_NAME = "dir";

  @Nested
  class WhenFile {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldDeleteDirectory(Path dir) throws IOException {
      Path file = dir.resolve(FILE_NAME);
      Files.write(file, "test".getBytes());

      FileUtils.delete(file);
      assertThat(file).doesNotExist();
    }
  }

  @Nested
  class WhenEmptyDirectory {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldDeleteDirectory(Path emptyDir) {
      FileUtils.delete(emptyDir);
      assertThat(emptyDir).doesNotExist();
    }
  }

  @Nested
  class WhenNonEmptyDirectory {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldDeleteDirectory(Path nonEmptyDir) throws IOException {
      Path dirInNonEmptyDir = nonEmptyDir.resolve(DIR_IN_NON_EMPTY_DIR_NAME);
      Files.createDirectories(dirInNonEmptyDir);
      Path fileInNonEmptyDir = dirInNonEmptyDir.resolve(FILE_IN_NON_EMPTY_DIR_NAME);
      Files.write(fileInNonEmptyDir, "test".getBytes());

      FileUtils.delete(nonEmptyDir);
      assertThat(nonEmptyDir).doesNotExist();
    }
  }

  @Nested
  class WhenNonExistingDirectory {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldNotDeleteAnything(Path dir) {
      Path nonExistingDir = dir.resolve(NON_EXISTING_DIR_NAME);

      assertThat(nonExistingDir).doesNotExist();
      FileUtils.delete(nonExistingDir);

      assertThat(dir).isDirectory();
    }
  }

}
