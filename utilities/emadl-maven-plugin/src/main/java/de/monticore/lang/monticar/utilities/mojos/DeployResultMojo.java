package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.ResultArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

/**
 * Creates a JAR containing a training result and
 * deploys it to the remote repository.
 *
 */
@Mojo(name = "deploy-result")
public class DeployResultMojo extends TrainingConfigMojo {

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    MavenProject mavenProject = (MavenProject) this.getPluginContext().get("project");
    StorageInformation storageInformation = getStorageInformation(mavenProject);

    File jarFile;
    try {
      getLog().info("STARTING creating Jar of the trained model.");
      jarFile = ResultArtifactCreator.createArtifact(storageInformation, getTrainingConfig(), getPathTmpOut(), getTaggingResolver());
      getLog().info("FINISHED creating Jar of the trained model.");

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
    storageInformation.setArtifactId(mavenProject.getArtifactId() + "-trained-model");
    storageInformation.setVersion(mavenProject.getVersion());
    storageInformation.setVersion(this.getNewestVersion(storageInformation));
    storageInformation.setPath(new File(getPathTmpOut()));

    return storageInformation;
  }
}
