/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Path;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class FileDownloaderTest {

  private static final String SOME_URL = "https://raw.githubusercontent.com/juj/emsdk/master/emsdk";
  private static final String NON_EXISTENT_URL = "https://raw.githubusercontent.com/juj/emsdk/master/abc";
  private static final String FILE_NAME = "emsdk";

  @Nested
  class WhenURLExists {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldDownloadFile(Path tempDir) throws IOException {
      URL url = new URL(SOME_URL);
      Path file = tempDir.resolve(FILE_NAME);

      FileDownloader.download(url, file);

      assertThat(file).exists();
    }
  }

  @Nested
  class WhenURLDoesNotExists {

    @Test
    @ExtendWith(TemporaryDirectoryExtension.class)
    void shouldThrowException(Path tempDir) throws IOException {
      URL url = new URL(NON_EXISTENT_URL);
      Path file = tempDir.resolve(FILE_NAME);

      assertThatExceptionOfType(IOException.class)
          .isThrownBy(() -> FileDownloader.download(url, file));
    }
  }

}
