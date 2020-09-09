package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.models.DatasetToStore;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugins.annotations.Parameter;

public abstract class BaseMojo extends AbstractMojo {

  @Parameter
  private DatasetToStore datasetToStore;

  public DatasetToStore getDatasetToStore() {
    return datasetToStore;
  }

  public void setDatasetToStore(DatasetToStore datasetToStore) {
    this.datasetToStore = datasetToStore;
  }

}
