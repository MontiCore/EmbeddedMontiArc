/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.emadl.generator.AutoMLCli;
import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.utilities.artifactdeployer.ConfigurationArtifactDeployer;
import de.monticore.lang.monticar.utilities.utils.ConfigurationTracking;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.mlpipelines.util.configuration_tracking.ConfigurationTrackingManager;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.twdata.maven.mojoexecutor.MojoExecutor.*;

/**
 * Generates C++ code and trains an EMADL component with neural network implementation.
 */
@Mojo(name = "train")
public class TrainingMojo extends TrainingConfigMojo {
  @Parameter
  private String artifactTrackingConfiguration;
  @Parameter
  private String mlflowTrackingConfiguration;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    File settingsFile = getMavenSession().getRequest().getUserSettingsFile();
    if (artifactTrackingConfiguration != null) {
      if (artifactTrackingConfiguration.isEmpty()) {
        Log.error("Please provide the path to the configuration file required for artifact tracking.");
      }
      boolean success = ConfigurationTracking.initialize(artifactTrackingConfiguration, getMavenProject(), getMavenSession(), getTrainingConfig());
      if (!success) {
        Log.error("The chosen combination of group_id and experiment_name already exists. Please change at least one value");
        return;
      }
    }
    if (getTrainingConfig().getBackend().equals(Backend.PYTORCH)) {
      List<String> arguments = new ArrayList<>(Arrays.asList(
              "-m", getTrainingConfig().getPathToProject().getPath(),
              "-r", getTrainingConfig().getModelToTrain(),
              "-o", "target",
              "-b", getTrainingConfig().getBackend().name(),
              "-f", "n",
              "-c", "n"
      ));
      if (mlflowTrackingConfiguration != null) {
        arguments.add("-track");
        arguments.add(mlflowTrackingConfiguration);
      }
      AutoMLCli.main(arguments.toArray(new String[0]));
    } else {
      executeMojo(
              plugin(
                      groupId("de.monticore.lang.monticar.utilities"),
                      artifactId("maven-streamtest"),
                      version("0.0.35-SNAPSHOT")
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

    if (ConfigurationTrackingManager.shouldDeployArtifact()) {
      ConfigurationArtifactDeployer.deployArtifact(ConfigurationTracking.getStorageInformation(), ConfigurationTracking.getGitlabRepository(), settingsFile);
    }
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
    if (trainingConfig.getAllowDecomposition() != null)
      elements.add(element(name("allowDecomposition"), trainingConfig.getAllowDecomposition()));
    if (trainingConfig.getDecomposeNetwork() != null)
      elements.add(element(name("decomposeNetwork"),trainingConfig.getDecomposeNetwork()));
    return elements;
  }

}
