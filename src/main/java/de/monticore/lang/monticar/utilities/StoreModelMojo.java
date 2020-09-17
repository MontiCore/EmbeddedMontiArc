package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.ModelArtifactCreator;
import de.monticore.lang.monticar.utilities.models.DatasetToStore;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;

@Mojo(name = "storeModel")
public class StoreModelMojo extends BaseMojo {

  @Parameter
  private DatasetToStore modelToStore;


  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();

    File jarFile;
    try {
      ModelArtifactCreator.createArtifact(this.modelToStore);
    }
    catch (Exception e) {
      e.printStackTrace();
    }

  }

}
