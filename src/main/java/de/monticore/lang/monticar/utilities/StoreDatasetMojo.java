package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.DatasetArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.DatasetToStore;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

@Mojo(name="storeDataset")
public class StoreDatasetMojo extends BaseMojo {

  @Parameter
  private DatasetToStore datasetToStore;

  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.datasetToStore.getPath()));
      jarFile = DatasetArtifactCreator.createArtifact(this.datasetToStore, TEMP_FOLDER);
      getLog().info("FINISHED creating Jar for dataset");

      DatasetArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.datasetToStore);
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }


  }

}
