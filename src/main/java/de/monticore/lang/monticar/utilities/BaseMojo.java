package de.monticore.lang.monticar.utilities;

import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugins.annotations.Parameter;

public abstract class BaseMojo extends AbstractMojo {

  @Parameter
  private String pathToDataset;

  public String getPathToDataset() {
    return pathToDataset;
  }

  public void setPathToDataset(String pathToDataset) {
    this.pathToDataset = pathToDataset;
  }
}
