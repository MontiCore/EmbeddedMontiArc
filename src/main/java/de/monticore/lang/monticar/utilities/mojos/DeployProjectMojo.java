package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactcreator.ModelArtifactCreator;
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

@Mojo(name = "deploy-project")
public class DeployProjectMojo extends BaseMojo {

  @Parameter
  private StorageInformation projectToStore;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkTmpDir();
    int newestVersion = this.getNewestVersion(projectToStore);
    projectToStore.setVersion(newestVersion);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for model %s", this.projectToStore.getPath()));
      jarFile = ModelArtifactCreator.createArtifact(this.projectToStore, this.getPathTmpOut());
      getLog().info("FINISHED creating Jar for model");

      ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), this.projectToStore, this.getRepository(), JarClassifierEnum.EMADL,
          getMavenSession().getRequest().getUserSettingsFile());
    }
    catch (IOException | MavenInvocationException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }

  }

}
