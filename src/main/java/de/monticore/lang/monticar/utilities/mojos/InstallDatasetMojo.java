package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

@Mojo(name = "install-dataset")
public class InstallDatasetMojo extends BaseMojo {

  @Parameter
  private StorageInformation datasetToStore;

  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    int newestVersion = this.getNewestVersion(datasetToStore);
    datasetToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.datasetToStore.getPath()));
      jarFile = DatasetArtifactCreator.createArtifact(this.datasetToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for dataset");

      ArtifactDeployer.installArtifact(jarFile.getAbsolutePath(), this.datasetToStore, JarClassifierEnum.DATASET);
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }

  }

}
