/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.FilePrecondition.requiresDirectory;
import static de.monticore.lang.monticar.contract.FilePrecondition.requiresExists;
import static de.monticore.lang.monticar.contract.FilePrecondition.requiresFile;
import static de.monticore.lang.monticar.contract.FilePrecondition.requiresNotEmpty;
import static de.monticore.lang.monticar.contract.FilePrecondition.requiresReadable;
import static de.monticore.lang.monticar.contract.FilePrecondition.requiresWritable;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class FilePreconditionTest {

  private static final Class<Precondition.PreconditionViolationException> VIOLATION_EXCEPTION_TYPE
      = Precondition.PreconditionViolationException.class;
  private static final String VIOLATION_MESSAGE = "VIOLATION MESSAGE";

  private static final Path NULL_PATH = null;
  private static final Path EXISTING_DIRECTORY = Paths.get("src/test/resources/files");
  private static final Path EXISTING_FILE = Paths.get("src/test/resources/files/empty.txt");
  private static final Path NON_EXISTING_PATH = Paths.get("src/test/resources/__NOT_EXISTING__");
  private static final Path EMPTY_FILE = Paths.get("src/test/resources/files/empty.txt");
  private static final Path NON_EMPTY_FILE = Paths.get("src/test/resources/files/not_empty.txt");

  @Nested
  class RequiresNotEmpty {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(NON_EXISTING_PATH));
      }

      @Test
      void whenDirectoryIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(EXISTING_DIRECTORY));
      }

      @Test
      void whenEmptyFileIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmpty(EMPTY_FILE));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenDirectoryIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(EXISTING_DIRECTORY, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyFileIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmpty(EMPTY_FILE, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenNonEmptyFileIsGiven() {
        assertThat(requiresNotEmpty(NON_EMPTY_FILE)).isEqualTo(NON_EMPTY_FILE);
      }
    }
  }

  @Nested
  class RequiresWritable {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresWritable(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresWritable(NON_EXISTING_PATH));
      }

      @Test
      void whenDirectoryIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresWritable(EXISTING_DIRECTORY));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresWritable(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresWritable(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenDirectoryIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresWritable(EXISTING_DIRECTORY, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenWritableFileIsGiven() {
        assertThat(requiresWritable(NON_EMPTY_FILE)).isEqualTo(NON_EMPTY_FILE);
      }
    }
  }

  @Nested
  class RequiresReadable {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresReadable(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresReadable(NON_EXISTING_PATH));
      }

      @Test
      void whenDirectoryIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresReadable(EXISTING_DIRECTORY));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresReadable(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresReadable(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenDirectoryIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresReadable(EXISTING_DIRECTORY, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenReadableFileIsGiven() {
        assertThat(requiresReadable(NON_EMPTY_FILE)).isEqualTo(NON_EMPTY_FILE);
      }
    }
  }

  @Nested
  class RequiresDirectory {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresDirectory(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresDirectory(NON_EXISTING_PATH));
      }

      @Test
      void whenFileIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresDirectory(EXISTING_FILE));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresDirectory(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresDirectory(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenFileIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresDirectory(EXISTING_FILE, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenExistingDirectoryIsGiven() {
        assertThat(requiresDirectory(EXISTING_DIRECTORY)).isEqualTo(EXISTING_DIRECTORY);
      }
    }
  }

  @Nested
  class RequiresFile {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresFile(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresFile(NON_EXISTING_PATH));
      }

      @Test
      void whenDirectoryIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresFile(EXISTING_DIRECTORY));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresFile(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresFile(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenDirectoryIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresFile(EXISTING_DIRECTORY, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenExistingFileIsGiven() {
        assertThat(requiresFile(EXISTING_FILE)).isEqualTo(EXISTING_FILE);
      }
    }
  }

  @Nested
  class RequiresExists {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresExists(NULL_PATH));
      }

      @Test
      void whenNonExistingPathIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresExists(NON_EXISTING_PATH));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresExists(NULL_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNonExistingPathIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresExists(NON_EXISTING_PATH, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenExistingFileIsGiven() {
        assertThat(requiresExists(EXISTING_FILE)).isEqualTo(EXISTING_FILE);
      }

      @Test
      void whenExistingDirectoryIsGiven() {
        assertThat(requiresExists(EXISTING_DIRECTORY)).isEqualTo(EXISTING_DIRECTORY);
      }
    }
  }

}
