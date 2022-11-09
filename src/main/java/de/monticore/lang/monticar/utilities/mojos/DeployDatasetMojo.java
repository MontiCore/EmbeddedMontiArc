package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.Dataset;
import de.monticore.lang.monticar.utilities.models.DatasetsConfiguration;
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
import java.util.stream.Collectors;

/**
 * Creates a JAR containing a dataset and deploys it to the remote repository.
 *
 */
@Mojo(name = "deploy-dataset")
public class DeployDatasetMojo extends BaseMojo {

  /**
   * Parameter to define the location of the files to store as well as the
   * coordinates that are used when creating the JAR.
   * <br>
   * The parameter contains the following sub-parameters: <br>
   * <ul>
   *   <li><b>path</b>: location of the files to be packaged and stored</li>
   *    <li><b>groupId</b>: the groupId used for the JAR.</li>
   *    <li><b>artifactId</b>: the artifactId used for the JAR.</li>
   *    <li><b>version</b>: the version used for the JAR.</li>
   *    <li><b>description</b>: optional description added to the manifest file of the JAR.</li>
   * </ul>
   * All sub-parameters are required, except the description. <br><br>
   */
  @Parameter
  private StorageInformation datasetToStore;

  @Parameter
  private DatasetsConfiguration datasetsToStore;

  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    File jarFile;

    if(datasetToStore != null){
      String newestVersion = this.getNewestVersion(datasetToStore);
      datasetToStore.setVersion(newestVersion);

      try {
        getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.datasetToStore.getPath()));
        jarFile = DatasetArtifactCreator.createArtifact(this.datasetToStore, this.getPathTmpOut());
        getLog().info("FINISHED creating Jar for dataset");

        ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.datasetToStore, getRepository(), JarClassifierEnum.DATASET,
                getMavenSession().getRequest().getUserSettingsFile());
      }
      catch (IOException | MavenInvocationException e) {
        throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
      }
    } else if (this.datasetsToStore != null) {
      String newestVersion = this.getNewestVersion(this.datasetsToStore);
      this.datasetsToStore.setVersion(newestVersion);

      try {
        getLog().info(String.format("STARTING creating Jar for the following datasets: %s", String.join(", ", datasetsToStore.getDatasets().stream().map(Dataset::getId).collect(Collectors.toList()))));
        jarFile = DatasetArtifactCreator.createArtifact(this.datasetsToStore, this.getPathTmpOut());
        getLog().info("FINISHED creating Jar for dataset(s)");

        ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.datasetsToStore, getRepository(), JarClassifierEnum.DATASET,
                getMavenSession().getRequest().getUserSettingsFile(), getDependencies());
      }
      catch (IOException | MavenInvocationException e) {
        throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
      }
    } else {
      throw new MojoFailureException("Neither datasetToStore nor datasetsToStore are defined.");
    }

  }

}
