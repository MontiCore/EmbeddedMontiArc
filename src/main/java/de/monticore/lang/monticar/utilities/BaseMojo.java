package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.models.DatasetToStore;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.nio.file.Paths;

public abstract class BaseMojo extends AbstractMojo {

   protected static final String TEMP_FOLDER = "target/tmp";

  @Parameter
  private DatasetToStore datasetToStore;

  public DatasetToStore getDatasetToStore() {
    return datasetToStore;
  }

  public void setDatasetToStore(DatasetToStore datasetToStore) {
    this.datasetToStore = datasetToStore;
  }

  protected void mkdir(String path) {
    try {
      File tmpOut = Paths.get(path).toFile();
      if(!tmpOut.exists()){
        tmpOut.mkdirs();
      }
    }catch (Exception ex){
      ex.printStackTrace();
    }
  }

}
