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
 * installs it into the local repository.
 *
 */
@Mojo(name = "install-pretrained")
public class InstallPretrainedMojo extends BaseMojo {

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
  private StorageInformation pretrainedModelToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    String newestVersion = this.getNewestVersion(pretrainedModelToStore);
    pretrainedModelToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for pretrained network in directory %s", this.pretrainedModelToStore.getPath()));
      jarFile = PretrainedArtifactCreator.createArtifact(this.pretrainedModelToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for pretrained network");

      ArtifactDeployer.installArtifact(jarFile.getAbsolutePath(), this.pretrainedModelToStore, JarClassifierEnum.PRETRAINED);
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }

  }
}
