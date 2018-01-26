package de.monticore.lang.monticar.emam2wasm;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import de.monticore.lang.monticar.util.FileUtils;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class EMAMStepTest {

  private static final Path EXPECTED_MODEL_PATH = Paths.get("a/b/c/Model.emam");
  private static final String NULL_STRING = null;
  private static final String PACKAGE_NAME = "a.b.c";
  private static final String MODEL_NAME = "Model";
  private static final String MODEL_CONTENT = "test";

  @Nested
  class Save {

    private ModelParser modelParser;

    @BeforeEach
    void setUp() {
      modelParser = mock(ModelParser.class);
      when(modelParser.parsePackage(anyString())).thenReturn(PACKAGE_NAME);
      when(modelParser.parseModelName(anyString())).thenReturn(MODEL_NAME);
    }

    @Nested
    class WhenModelStringIsNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldThrowPreconditionViolationException(Path basePath) {
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> emamStep.save(NULL_STRING));
      }
    }

    @Nested
    class WhenModelStringIsNotNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldSaveModelCorrespondingToFullName(Path basePath) {
        Path targetPath = basePath.resolve(EXPECTED_MODEL_PATH);
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        emamStep.save(MODEL_CONTENT);

        assertThat(targetPath).isRegularFile();
      }

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldReturnExpectedModelPath(Path basePath) {
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        Path savePath = emamStep.save(MODEL_CONTENT);

        assertThat(basePath.relativize(savePath)).isEqualTo(EXPECTED_MODEL_PATH);
      }

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldWriteProvidedContent(Path basePath) {
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        Path savePath = emamStep.save(MODEL_CONTENT);
        assertThat(savePath).hasContent(MODEL_CONTENT);
      }
    }
  }

  @Nested
  class GetFile {

    private ModelParser modelParser;

    @BeforeEach
    void setUp() {
      modelParser = mock(ModelParser.class);
      when(modelParser.parsePackage(anyString())).thenReturn(PACKAGE_NAME);
      when(modelParser.parseModelName(anyString())).thenReturn(MODEL_NAME);
    }

    @Nested
    class WhenModelStringIsNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldThrowPreconditionViolationException(Path basePath) {
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> emamStep.getFile(NULL_STRING));
      }
    }

    @Nested
    class WhenModelStringIsNotNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldReturnExpectedModelPath(Path basePath) {
        EMAMStep emamStep = new EMAMStep(basePath, modelParser);

        Path filePath = emamStep.getFile(MODEL_CONTENT);
        assertThat(basePath.relativize(filePath)).isEqualTo(EXPECTED_MODEL_PATH);
      }
    }
  }

}