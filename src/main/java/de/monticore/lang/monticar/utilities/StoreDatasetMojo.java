package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import de.monticore.lang.monticar.utilities.utils.JarDeployer;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

@Mojo(name="storeDataset")
public class StoreDatasetMojo extends BaseMojo {

  public void execute() throws MojoExecutionException, MojoFailureException {
    this.mkdir(TEMP_FOLDER);

    File jarFile;
    try {
      getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.getDatasetToStore().getPath()));
      jarFile = DatasetArtifactCreator.createArtifact(this.getDatasetToStore(), TEMP_FOLDER);
      getLog().info("FINISHED creating Jar for dataset");

      JarDeployer.deployArtifact(jarFile.getAbsolutePath(), (MavenProject) getPluginContext().get("project"));
    }
    catch (MavenInvocationException | IOException e) {
      throw new MojoFailureException(Arrays.toString(e.getStackTrace()));
    }


  }

}
