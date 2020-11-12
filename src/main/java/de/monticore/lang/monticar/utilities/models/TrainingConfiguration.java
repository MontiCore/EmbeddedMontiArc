package de.monticore.lang.monticar.utilities.models;

import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.utilities.utils.GeneratorEnum;

import java.io.File;

public class TrainingConfiguration {

  private GeneratorEnum generator;

  private Backend backend;

  private File pathToPython;

  private File pathToProject;

  private File pathToTest;

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

  public File getPathToProject() {
    if (pathToProject == null) {
      pathToProject = new File("src/main/emadl");
    }

    return pathToProject;
  }

  public File getPathToTest() {
    if (pathToTest == null) {
      pathToTest = new File("src/main/emadl");
    }

    return pathToTest;
  }

  public String getModelToTrain() {
    return modelToTrain;
  }
}
