package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

@Mojo(name="storeDataset")
public class StoreDatasetMojo extends BaseMojo {

  public void execute() throws MojoExecutionException, MojoFailureException {
    DatasetArtifactCreator creator = new DatasetArtifactCreator();
    getLog().info(String.format("STARTING creating Jar for dataset in directory %s", this.getDatasetToStore().getPath()));
    creator.createArtifact(this.getDatasetToStore());
    getLog().info("FINISHED creating Jar for dataset");
  }

}
