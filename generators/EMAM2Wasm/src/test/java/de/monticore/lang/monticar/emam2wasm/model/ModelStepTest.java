/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.model;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class ModelStepTest {

  private static final Path EXPECTED_MODEL_PATH = Paths.get("a/b/c/Model.emam");
  private static final String NULL_STRING = null;
  private static final String PACKAGE_NAME = "a.b.c";
  private static final String MODEL_NAME = "Model";
  private static final String FILE_EXTENSION = "emam";
  private static final String MODEL_CONTENT = "test";

  private ModelNameProvider modelNameProviderMock() {
    ModelNameProvider nameProvider = mock(ModelNameProvider.class);
    when(nameProvider.getPackage(anyString())).thenReturn(PACKAGE_NAME);
    when(nameProvider.getName(anyString())).thenReturn(MODEL_NAME);
    when(nameProvider.getFileExtension()).thenReturn(FILE_EXTENSION);
    when(nameProvider.getFilePath(anyString())).thenCallRealMethod();
    return nameProvider;
  }

  @Nested
  class Save {

    private ModelNameProvider nameProvider;

    @BeforeEach
    void setUp() {
      nameProvider = modelNameProviderMock();
    }

    @Nested
    class WhenModelStringIsNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldThrowPreconditionViolationException(Path basePath) {
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

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
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

        emamStep.save(MODEL_CONTENT);

        assertThat(targetPath).isRegularFile();
      }

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldReturnExpectedModelPath(Path basePath) {
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

        Path savePath = emamStep.save(MODEL_CONTENT);

        assertThat(basePath.relativize(savePath)).isEqualTo(EXPECTED_MODEL_PATH);
      }

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldWriteProvidedContent(Path basePath) {
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

        Path savePath = emamStep.save(MODEL_CONTENT);
        assertThat(savePath).hasContent(MODEL_CONTENT);
      }
    }
  }

  @Nested
  class GetFile {

    private ModelNameProvider nameProvider;

    @BeforeEach
    void setUp() {
      nameProvider = modelNameProviderMock();
    }

    @Nested
    class WhenModelStringIsNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldThrowPreconditionViolationException(Path basePath) {
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

        assertThatExceptionOfType(PreconditionViolationException.class)
            .isThrownBy(() -> emamStep.getFile(NULL_STRING));
      }
    }

    @Nested
    class WhenModelStringIsNotNull {

      @Test
      @ExtendWith(TemporaryDirectoryExtension.class)
      void shouldReturnExpectedModelPath(Path basePath) {
        ModelStep emamStep = new ModelStep(basePath, nameProvider);

        Path filePath = emamStep.getFile(MODEL_CONTENT);
        assertThat(basePath.relativize(filePath)).isEqualTo(EXPECTED_MODEL_PATH);
      }
    }
  }

}
