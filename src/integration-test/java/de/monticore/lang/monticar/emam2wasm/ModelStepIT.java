/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm;

import static org.assertj.core.api.Assertions.assertThat;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.emam2wasm.model.EmamModelNameProvider;
import de.monticore.lang.monticar.emam2wasm.model.ModelNameProvider;
import de.monticore.lang.monticar.emam2wasm.model.ModelStep;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import de.monticore.lang.monticar.util.TextFile;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

public class ModelStepIT {

  private static final Path MODEL_FILE_PATH = Paths
      .get("src/integration-test/resources/modelstep/model.txt");

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldStoreModel(Path dir) {
    TextFile file = new TextFile(MODEL_FILE_PATH);
    ModelNameProvider modelNameProvider = new EmamModelNameProvider(
        new EmbeddedMontiArcMathParser());
    ModelStep modelStep = new ModelStep(dir, modelNameProvider);

    Path storedModelPath = modelStep.save(file.read());

    assertThat(storedModelPath).hasSameContentAs(MODEL_FILE_PATH);
  }
}
