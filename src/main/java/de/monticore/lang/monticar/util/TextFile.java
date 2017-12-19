package de.monticore.lang.monticar.util;

import static de.monticore.lang.monticar.contract.FilePrecondition.requiresFile;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Collectors;

/**
 * Utility class for text files to perform easy read and write operations. There are no checked
 * exceptions thrown, instead {@code IOException}s are wrapped and rethrown as
 * {@link UncheckedIOException}.
 */
public class TextFile {

  private final Path path;

  public TextFile(Path path) {
    this.path = requiresFile(path);
  }

  public Path getPath() {
    return path;
  }

  /**
   * Checks whether the file already exists.
   *
   * @return {@code true} iff the file already exists
   */
  public boolean isPresent() {
    return Files.exists(path);
  }

  /**
   * Writes a supplied string to a text file. If the file already exists, it deletes the content
   * of the original file and then writes the string into it.
   *
   * @param text string to write to the text file
   * @throws UncheckedIOException whenever the underlying write operations throw a checked
   * {@code IOException}
   */
  public void write(String text) {
    try {
      Files.createDirectories(path.getParent());
      Files.write(path, text.getBytes());
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /**
   * Reads the text file and returns its content in a single string.
   *
   * @return file content as string
   * @throws UncheckedIOException whenever the underlying read operations throw a checked
   * {@code IOException}
   */
  public String read() {
    try {
      return Files.lines(path).collect(Collectors.joining("\n"));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
