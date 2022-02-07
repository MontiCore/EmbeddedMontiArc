package de.monticore.lang.monticar.utilities.models;

import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.utilities.utils.GeneratorEnum;

import java.io.File;

public class TrainingConfiguration {

  private File pathToPython;

  private File customFilesPath;

  private File pathToProject;

  private File pathToTest;

  private String modelToTrain;

  private String useDgl;

  private Backend backend;

  private GeneratorEnum generator;

  public File getPathToPython() {
    return pathToPython;
  }

  public File getCustomFilesPath(){
    return customFilesPath;
  }

  public File getPathToProject() {
    return pathToProject;
  }

  public File getPathToTest() {
    return pathToTest;
  }

  public String getModelToTrain() {
    return modelToTrain;
  }

  public String getUseDgl(){
    return useDgl;
  }

  public Backend getBackend() {
    return backend;
  }

  public GeneratorEnum getGenerator() {
    return generator;
  }
}
