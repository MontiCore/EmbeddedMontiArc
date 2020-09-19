package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.ModelArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.Arrays;

@Mojo(name = "storeModel")
public class StoreModelMojo extends BaseMojo {

  @Parameter
  private StorageInformation modelToStore;


  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for model %s", this.modelToStore.getPath()));
      jarFile = ModelArtifactCreator.createArtifact(this.modelToStore, TEMP_FOLDER);
      getLog().info("FINISHED creating Jar for model");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), modelToStore);
    }
    catch (Exception e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }

  }

}
