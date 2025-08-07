/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.resolver;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class ResolverTest {

  private static final Path RESOLVE_PATH = Paths.get("src/test/resources/resolver");

  private static final String CORRECT_MODEL_CMP = "models.CorrectModel";
  private static final String NON_EXISTING_MODEL_CMP = "models.DoesNotExist";
  private static final String MODEL_NAME_NOT_EQUAL_TO_FILENAME_MODEL_CMP = "models.SomeOtherModelName";
  private static final String MODEL_NAME_LOWERCASE_CMP = "models.ModelNameLowercase";
  private static final String FILE_NAME_LOWERCASE_CMP = "models.FileNameLowercase";
  private static final String WRONG_EXTENSION_CMP = "models.WrongExtension";
  private static final String MISSING_PACKAGE_CMP = "models.MissingPackage";
  private static final String WRONG_PACKAGE_CMP = "wrong.WrongPackage";
  private static final String NOT_FULL_NAME_CMP = "CorrectModel";

  private static final String CORRECT_PORT = "models.CorrectModel.in1";
  private static final String NON_EXISTING_PORT = "models.CorrectModel.in3";

  private static final String CORRECT_COMPONENT_INST = "models.correctModel";
  private static final String NON_EXISTING_COMPONENT_INST = "models.doesNotExist";

  private static Resolver resolver;

  //  @BeforeAll
  @BeforeEach
  void setUp() {
    SymTabCreator symTabCreator = new SymTabCreator(RESOLVE_PATH);
    resolver = new Resolver(symTabCreator.createSymTab());
  }

  @Nested
  class GetComponentSymbol {

    @Test
    void shouldResolveComponent() {
      assertThat(resolver.getComponentSymbol(CORRECT_MODEL_CMP).getFullName()).isEqualTo(
          CORRECT_MODEL_CMP);
    }

    @Nested
    class ShouldThrowComponentNotFoundException {

      @Test
      void whenModelDoesNotExist() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(NON_EXISTING_MODEL_CMP));
      }

      @Test
      void whenModelNameIsNotEqualToFilename() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(
                () -> resolver.getComponentSymbol(MODEL_NAME_NOT_EQUAL_TO_FILENAME_MODEL_CMP));
      }

      @Test
      void whenModelNameIsLowercase() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(MODEL_NAME_LOWERCASE_CMP));
      }

      @Disabled
      @Test
      void whenFileNameIsLowercase() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(FILE_NAME_LOWERCASE_CMP));
      }

      @Test
      void whenFileNameHasWrongExtension() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(WRONG_EXTENSION_CMP));
      }

      @Test
      void whenPackageIsMissing() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(MISSING_PACKAGE_CMP));
      }

      @Test
      void whenPackageIsWrong() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(WRONG_PACKAGE_CMP));
      }

      @Disabled
      @Test
      void whenNotFullName() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getComponentSymbol(NOT_FULL_NAME_CMP));
      }
    }
  }

  @Nested
  class GetPortSymbol {

    @Test
    void shouldResolveComponent() {
      assertThat(resolver.getPortSymbol(CORRECT_PORT).getFullName()).isEqualTo(
          CORRECT_PORT);
    }

    @Nested
    class ShouldThrowComponentNotFoundException {

      @Test
      void whenPortDoesNotExist() {
        assertThatExceptionOfType(ComponentNotFoundException.class)
            .isThrownBy(() -> resolver.getPortSymbol(NON_EXISTING_PORT));
      }
    }
  }

  @Nested
  class GetExpandedComponentInstanceSymbol {

    @Test
    void shouldResolveComponent() {
      assertThat(resolver.getExpandedComponentInstanceSymbol(CORRECT_COMPONENT_INST).getFullName())
          .isEqualTo(CORRECT_COMPONENT_INST);
    }

    @Nested
    class ShouldThrowComponentNotFoundException {

      @Test
      void whenInstanceDoesNotExist() {
        assertThatExceptionOfType(ComponentNotFoundException.class).isThrownBy(
            () -> resolver.getExpandedComponentInstanceSymbol(NON_EXISTING_COMPONENT_INST));
      }
    }
  }
}
