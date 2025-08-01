/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.util;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.stream.Stream;

public class FileUtils {

  /**
   * Deletes the file or directory specified in the supplied {@code path}. Directories do not need
   * to be empty. In case a directory is not empty, it will recursively delete the directory's
   * content.
   *
   * @throws UncheckedIOException whenever the underlying delete operations throw a checked
   * {@code IOException}
   */
  public static void delete(Path path) {
    delete(path, 2);
  }

  private static void delete(Path path, int attempts) {
    if (Files.exists(path)) {
      try (Stream<Path> fileStream = Files.walk(path)) {
        fileStream
            .sorted(Comparator.reverseOrder())
            .forEach(p -> {
              try {
                Files.delete(p);
              } catch (IOException e) {
                if (attempts > 0) {
                  //Windows may be just slow, so try again
                  try {
                    Thread.sleep(200);
                  } catch (InterruptedException ignored) {
                  }
                  delete(path, attempts - 1);
                } else {
                  throw new UncheckedIOException(e);
                }
              }
            });
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      }
    }
  }

}
