package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

@Mojo(name="deploy-dataset")
public class DeployDatasetMojo extends BaseMojo {

  @Parameter
  private StorageInformation datasetToStore;

  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    int newestVersion = this.getNewestVersion(datasetToStore);
    datasetToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.datasetToStore.getPath()));
      jarFile = DatasetArtifactCreator.createArtifact(this.datasetToStore, TEMP_FOLDER);
      getLog().info("FINISHED creating Jar for dataset");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.datasetToStore, this.getRepository());
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }


  }

}
