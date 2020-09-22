package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.models.Repository;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.nio.file.Paths;

public abstract class BaseMojo extends AbstractMojo {

  @Parameter
  private Repository repository;

  protected static final String TEMP_FOLDER = "target/tmp";

  public void mkTmpDir() {
    this.mkdir(TEMP_FOLDER);
  }

  private void mkdir(String path) {
    try {
      File tmpOut = Paths.get(path).toFile();
      if(!tmpOut.exists()){
        tmpOut.mkdirs();
      }
    }catch (Exception ex){
      ex.printStackTrace();
    }
  }

  public Repository getRepository() {
    return repository;
  }
}
