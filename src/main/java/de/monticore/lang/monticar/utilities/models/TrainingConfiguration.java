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

  private String forceRun;

  private String allowDecomposition;

  private String decomposeNetwork;

  public String getForceRun() {
    return forceRun;
  }

  public void setForceRun(String forceRun) {
    this.forceRun = forceRun;
  }

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

  public String getAllowDecomposition() {return allowDecomposition; }

  public String getDecomposeNetwork() {return decomposeNetwork; }

  public void setPathToPython(File pathToPython) {
    this.pathToPython = pathToPython;
  }

  public void setCustomFilesPath(File customFilesPath) {
    this.customFilesPath = customFilesPath;
  }

  public void setPathToProject(File pathToProject) {
    this.pathToProject = pathToProject;
  }

  public void setPathToTest(File pathToTest) {
    this.pathToTest = pathToTest;
  }

  public void setModelToTrain(String modelToTrain) {
    this.modelToTrain = modelToTrain;
  }

  public void setUseDgl(String useDgl) {
    this.useDgl = useDgl;
  }

  public void setBackend(Backend backend) {
    this.backend = backend;
  }

  public void setGenerator(GeneratorEnum generator) {
    this.generator = generator;
  }

  public void setAllowDecomposition(String allowDecomposition) {this.allowDecomposition = allowDecomposition; }

  public void setDecomposeNetworks(String decomposeNetworks) {this.decomposeNetwork= decomposeNetworks; }
}
