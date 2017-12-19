package de.monticore.lang.monticar.util;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class FileUtilsTest {

  private static final Path BASE_PATH = Paths.get("src/test/resources/to_be_deleted");
  private static final Path FILE = BASE_PATH.resolve("file.txt");
  private static final Path EMPTY_DIR = BASE_PATH.resolve("empty_dir");
  private static final Path NON_EMPTY_DIR = BASE_PATH.resolve("dir");
  private static final Path DIR_IN_NON_EMPTY_DIR = NON_EMPTY_DIR.resolve("another_dir");
  private static final Path FILE_IN_NON_EMPTY_DIR = DIR_IN_NON_EMPTY_DIR.resolve("file.txt");

  @AfterEach
  void cleanUp() throws IOException {
    Files.deleteIfExists(EMPTY_DIR);
    Files.deleteIfExists(FILE_IN_NON_EMPTY_DIR);
    Files.deleteIfExists(DIR_IN_NON_EMPTY_DIR);
    Files.deleteIfExists(NON_EMPTY_DIR);
    Files.deleteIfExists(FILE);
    Files.deleteIfExists(BASE_PATH);
  }

  @Nested
  class WhenFile {

    @BeforeEach
    void setUp() throws IOException {
      Files.createDirectories(BASE_PATH);
      Files.write(FILE, "test".getBytes());
    }

    @Test
    void shouldDeleteDirectory() {
      FileUtils.delete(FILE);
      assertThat(FILE).doesNotExist();
    }
  }

  @Nested
  class WhenEmptyDirectory {

    @BeforeEach
    void setUp() throws IOException {
      Files.createDirectories(EMPTY_DIR);
    }

    @Test
    void shouldDeleteDirectory() {
      FileUtils.delete(EMPTY_DIR);
      assertThat(EMPTY_DIR).doesNotExist();
    }
  }

  @Nested
  class WhenNonEmptyDirectory {

    @BeforeEach
    void setUp() throws IOException {
      Files.createDirectories(NON_EMPTY_DIR);
      Files.createDirectories(DIR_IN_NON_EMPTY_DIR);
      Files.write(FILE_IN_NON_EMPTY_DIR, "test".getBytes());
    }

    @Test
    void shouldDeleteDirectory() {
      FileUtils.delete(NON_EMPTY_DIR);
      assertThat(NON_EMPTY_DIR).doesNotExist();
    }
  }

}