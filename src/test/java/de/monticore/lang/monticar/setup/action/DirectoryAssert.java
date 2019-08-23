/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import org.assertj.core.api.AbstractPathAssert;

public class DirectoryAssert extends AbstractPathAssert<DirectoryAssert> {

  public DirectoryAssert(Path actual) {
    super(actual, DirectoryAssert.class);
  }

  public static DirectoryAssert assertThat(Path actual) {
    return new DirectoryAssert(actual);
  }

  public DirectoryAssert hasSameDirectoryContentAs(Path other) {
    try {
      Files.walkFileTree(actual, new SimpleFileVisitor<Path>() {
        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
            throws IOException {
          FileVisitResult result = super.visitFile(file, attrs);

          Path relativePath = actual.relativize(file);
          Path fileInOther = other.resolve(relativePath);

          paths.assertExists(info, fileInOther);

          if (Files.isRegularFile(actual)) {
            hasSameContentAs(fileInOther);
          }
          return result;
        }
      });
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return this;
  }
}
