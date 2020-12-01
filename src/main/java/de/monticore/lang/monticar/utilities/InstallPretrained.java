package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.PretrainedArtifactCreator;
import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifier;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

@Mojo(name = "install-pretrained")
public class InstallPretrained extends BaseMojo {

  @Parameter
  private StorageInformation pretrainedNetworkToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    int newestVersion = this.getNewestVersion(pretrainedNetworkToStore);
    pretrainedNetworkToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for pretrained network in directory %s", this.pretrainedNetworkToStore.getPath()));
      jarFile = PretrainedArtifactCreator.createArtifact(this.pretrainedNetworkToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for pretrained network");

      ArtifactDeployer.installArtifact(jarFile.getAbsolutePath(), this.pretrainedNetworkToStore, this.getRepository(), JarClassifier.PRETRAINED);
    } catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }

  }
}
