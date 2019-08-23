/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class ExtractionActionTest {

  private static final Path RESOURCE_DIR = Paths.get("src/test/resources/action");
  private static final Path ARCHIVE = RESOURCE_DIR.resolve("archive.zip");
  private static final Path EXPECTED = RESOURCE_DIR.resolve("expected");
  private static final Path SOME_FILE = EXPECTED.resolve("file1.txt");
  private static final String SUBDIR_NAME = "subdir";
  private static final String ARCHIVE_NAME = "archive.zip";

  @Nested
  class WhenArchiveDoesNotExist {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldThrowActionException(Path dir) {
      Action action = new ExtractionAction(dir.resolve(ARCHIVE_NAME), dir);

      assertThatExceptionOfType(ActionException.class).isThrownBy(action::execute);
    }
  }

  @Nested
  class WhenFileIsNotArchive {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldThrowActionException(Path dir) {
      Action action = new ExtractionAction(SOME_FILE, dir);

      assertThatExceptionOfType(IllegalArgumentException.class).isThrownBy(action::execute);
    }
  }

  @Nested
  class WhenFileIsArchive {

    @Nested
    class WhenTargetDirDoesNotExist {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldCreateTargetDirAndExtract(Path dir) {
        Path subdir = dir.resolve(SUBDIR_NAME);
        Action action = new ExtractionAction(ARCHIVE, subdir);

        action.execute();

        DirectoryAssert.assertThat(subdir).hasSameDirectoryContentAs(EXPECTED);
      }
    }

    @Nested
    class WhenTargetDirExists {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldExtractToTargetDir(Path dir) {
        Action action = new ExtractionAction(ARCHIVE, dir);

        action.execute();

        DirectoryAssert.assertThat(dir).hasSameDirectoryContentAs(EXPECTED);
      }
    }
  }

}
