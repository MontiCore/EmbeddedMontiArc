/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.monticar.utilities.utils.PropertyReader;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.util.LinkedList;
import java.util.List;

import static org.twdata.maven.mojoexecutor.MojoExecutor.*;

/**
 * Generates C++ code and trains an EMADL component with neural network implementation.
 */
@Mojo(name = "train")
public class TrainingMojo extends TrainingConfigMojo {

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    executeMojo(
        plugin(
            groupId("de.monticore.lang.monticar.utilities"),
            artifactId("maven-streamtest"),
            version("0.0.33-SNAPSHOT")
        ),
        goal("streamtest-generator"),
        configuration(getConfigElements().toArray(new Element[0])),
        executionEnvironment(
            this.getMavenProject(),
            this.getMavenSession(),
            this.getPluginManager()
        )
    );
  }

  private List<Element> getConfigElements() {
    List<Element> elements = new LinkedList<>();
    TrainingConfiguration trainingConfig = this.getTrainingConfig();

    elements.add(element(name("rootModel"), trainingConfig.getModelToTrain()));
    elements.add(element(name("trainingNeeded"), "true"));

    if (trainingConfig.getPathToProject() != null)
      elements.add(element(name("pathMain"), trainingConfig.getPathToProject().getAbsolutePath()));
    if (trainingConfig.getPathToTest() != null)
      elements.add(element(name("pathTest"), trainingConfig.getPathToTest().getAbsolutePath()));
    if (trainingConfig.getBackend() != null)
      elements.add(element(name("backend"), trainingConfig.getBackend().name()));
    if (trainingConfig.getPathToPython() != null)
      elements.add(element(name("pathToPython"), trainingConfig.getPathToPython().getAbsolutePath()));
    if (trainingConfig.getGenerator() != null)
      elements.add(element(name("generator"), trainingConfig.getGenerator().name()));
    if (trainingConfig.getCustomFilesPath() != null)
      elements.add(element(name("customFilesPath"), trainingConfig.getCustomFilesPath().getAbsolutePath()));
    if (trainingConfig.getUseDgl() != null)
      elements.add(element(name("useDgl"), trainingConfig.getUseDgl()));
    if (trainingConfig.getForceRun() != null)
      elements.add(element(name("forceRun"), trainingConfig.getForceRun()));
    return elements;
  }

}
