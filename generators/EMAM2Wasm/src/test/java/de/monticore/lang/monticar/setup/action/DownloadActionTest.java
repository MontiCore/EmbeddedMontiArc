/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class DownloadActionTest {

  private static final String SOME_URL = "http://www.se-rwth.de/layout/rwth_se_rgb.png";
  private static final String SOME_NON_EXISTENT_URL = "http://google.de/404";
  private static final String FILE_NAME = "se.png";
  private static final String DIR_NAME = "dir";

  @Nested
  class WhenURLDoesNotExist {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldThrowActionException(Path dir) throws MalformedURLException {
      Action action = new DownloadAction(new URL(SOME_NON_EXISTENT_URL), dir.resolve(FILE_NAME));

      assertThatExceptionOfType(ActionException.class).isThrownBy(action::execute);
    }
  }

  @Nested
  class WhenPathIsDirectory {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldThrowActionException(Path dir) throws IOException {
      Path subdir = Files.createTempDirectory(dir, DIR_NAME);
      Action action = new DownloadAction(new URL(SOME_NON_EXISTENT_URL), subdir);

      assertThatExceptionOfType(ActionException.class).isThrownBy(action::execute);
    }

  }

  @Nested
  class WhenURLExists {

    @Nested
    class WhenPathIsFile {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldDownloadFile(Path dir) throws IOException {
        Action action = new DownloadAction(new URL(SOME_URL), dir.resolve(FILE_NAME));

        action.execute();

        assertThat(dir.resolve(FILE_NAME)).exists();
      }
    }
  }
}
