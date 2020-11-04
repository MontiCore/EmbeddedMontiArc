/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;

import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import static org.twdata.maven.mojoexecutor.MojoExecutor.*;

/**
 * Runs CoCos test on all components in pathMain and pathTest
 * Generates c++ code for all components which have a stream test
 */
@Mojo(name = "train")
public class TrainingMojo1 extends BaseMojo {

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    executeMojo(
        plugin(
            groupId("de.monticore.lang.monticar.utilities"),
            artifactId("maven-streamtest")
        ),
        goal("streamtest-build"),
        configuration(
            element(name("pathMain"), this.getPathMain()),
            element(name("pathTest"), this.getPathTest()),
            element(name("pathTmpOut"), this.getPathTmpOut()),
            element(name("backend"), this.getTrainingConfig().getBackend().name()),
            element(name("rootModel"), this.getTrainingConfig().getModelToTrain()),
            element(name("pathToPython"), this.getTrainingConfig().getPathToPython().getAbsolutePath())
            ),
        executionEnvironment(
            this.getMavenProject(),
            this.getMavenSession(),
            this.getPluginManager()
        )
    );
  }
}
