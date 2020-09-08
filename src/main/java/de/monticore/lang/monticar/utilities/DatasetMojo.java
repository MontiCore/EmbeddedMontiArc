package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.jarcreator.DatasetArtifactCreator;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

@Mojo(name="storeDataset")
public class DatasetMojo extends BaseMojo {

  public void execute() throws MojoExecutionException, MojoFailureException {
    String pathToDataset = this.getPathToDataset();
    DatasetArtifactCreator creator = new DatasetArtifactCreator();
    getLog().info(String.format("STARTING creating Jar for dataset in directory %s", pathToDataset));
    creator.createArtifact(pathToDataset);
    getLog().info("FINISHED creating Jar for dataset");
  }

}
