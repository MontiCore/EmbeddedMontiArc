package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.TrainingEnvironmentArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

/**
 * Creates a JAR containing a training environment and
 * deploys it to the remote repository. A training environment consists
 * of EMADL components, training configurations and a dataset.
 *
 */
@Mojo(name = "deploy-environment")
public class DeployTrainingEnvironmentMojo extends BaseMojo {

  /**
   * Parameter to define the location of the dataset to store
   * <br>
   * The parameter needs the following sub-parameter: <br>
   * <ul>
   *   <li><b>path</b>: location of the files to be packaged and stored</li>
   * </ul>
   */
  @Parameter
  private StorageInformation datasetToStore;

  /**
   * Parameter to define the location of the EMADL components and training configurations to store
   * <br>
   * The parameter needs the following sub-parameter: <br>
   * <ul>
   *   <li><b>path</b>: location of the files to be packaged and stored</li>
   * </ul>
   */
  @Parameter
  private StorageInformation projectToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    MavenProject mavenProject = (MavenProject) this.getPluginContext().get("project");
    StorageInformation storageInformation = getStorageInformation(mavenProject);

    File jarFile;
    try {
      getLog().info("STARTING creating Jar for training environment");
      jarFile = TrainingEnvironmentArtifactCreator.createArtifact(storageInformation, this.datasetToStore, this.projectToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for training environment");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), storageInformation, this.getRepository(), JarClassifierEnum.EMPTY,
          getMavenSession().getRequest().getUserSettingsFile());
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }
  }

  private StorageInformation getStorageInformation(MavenProject mavenProject) throws MojoExecutionException {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId(mavenProject.getGroupId());
    storageInformation.setArtifactId(mavenProject.getArtifactId() + "-training-environment");
    storageInformation.setVersion(mavenProject.getVersion());
    storageInformation.setVersion(this.getNewestVersion(storageInformation));

    return storageInformation;
  }

}

