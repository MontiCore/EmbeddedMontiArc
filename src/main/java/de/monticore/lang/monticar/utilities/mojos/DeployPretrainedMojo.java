package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.PretrainedArtifactCreator;
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

/**
 * Creates a JAR containing a pre-trained neural network and
 * deploys it to the remote repository.
 *
 */
@Mojo(name = "deploy-pretrained")
public class DeployPretrainedMojo extends BaseMojo {

  @Parameter
  private StorageInformation pretrainedModelToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    int newestVersion = this.getNewestVersion(pretrainedModelToStore);
    pretrainedModelToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for pretrained network in directory %s", this.pretrainedModelToStore.getPath()));
      jarFile = PretrainedArtifactCreator.createArtifact(this.pretrainedModelToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for pretrained network");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.pretrainedModelToStore, this.getRepository(), JarClassifierEnum.PRETRAINED,
          getMavenSession().getRequest().getUserSettingsFile());
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }
  }
}
