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

@Mojo(name = "deploy-environment")
public class DeployTrainingEnvironmentMojo extends BaseMojo {

  @Parameter
  private StorageInformation datasetToStore;

  @Parameter
  private StorageInformation modelToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    MavenProject mavenProject = (MavenProject) this.getPluginContext().get("project");
    StorageInformation storageInformation = getStorageInformation(mavenProject);

    File jarFile;
    try {
      getLog().info("STARTING creating Jar for training environment");
      jarFile = TrainingEnvironmentArtifactCreator.createArtifact(storageInformation, this.datasetToStore, this.modelToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for training environment");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), storageInformation, this.getRepository(), JarClassifierEnum.EMPTY);
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }
  }

  private StorageInformation getStorageInformation(MavenProject mavenProject) {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId(mavenProject.getGroupId());
    storageInformation.setArtifactId(mavenProject.getArtifactId() + "-training-environment");
    storageInformation.setVersion(this.getNewestVersion(storageInformation));

    return storageInformation;
  }

}

