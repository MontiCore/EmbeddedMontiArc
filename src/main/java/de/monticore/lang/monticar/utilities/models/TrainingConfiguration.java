package de.monticore.lang.monticar.utilities.models;

import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.utilities.utils.GeneratorEnum;

import java.io.File;

public class TrainingConfiguration {

  private GeneratorEnum generator;

  private Backend backend;

  private File pathToPython;

  private String modelToTrain;

  public GeneratorEnum getGenerator() {
    return generator;
  }

  public Backend getBackend() {
    if (backend == null) {
      backend = Backend.GLUON;
    }

    return backend;
  }

  public File getPathToPython() {
    if (pathToPython == null) {
      pathToPython = new File("/usr/bin/python");
    }

    return pathToPython;
  }

  public String getModelToTrain() {
    return modelToTrain;
  }
}
