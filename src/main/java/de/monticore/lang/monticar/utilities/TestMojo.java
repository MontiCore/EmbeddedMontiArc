package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.jarcreator.DatasetArtifactCreator;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

@Mojo(name="ml-process-ex")
public class TestMojo extends AbstractMojo {
  public void execute() throws MojoExecutionException, MojoFailureException {
    DatasetArtifactCreator creator = new DatasetArtifactCreator();
    creator.createArtifact();
    getLog().info("TEST LOGGING");
  }
}
