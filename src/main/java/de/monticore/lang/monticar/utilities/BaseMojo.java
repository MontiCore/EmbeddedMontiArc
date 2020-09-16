package de.monticore.lang.monticar.utilities;

import org.apache.maven.plugin.AbstractMojo;

import java.io.File;
import java.nio.file.Paths;

public abstract class BaseMojo extends AbstractMojo {

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

}
