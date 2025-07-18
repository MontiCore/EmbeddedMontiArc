package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.monticar.utilities.mojos.DeployResultMojo;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import java.io.File;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.spy;
import static org.powermock.api.mockito.PowerMockito.mock;
import static org.powermock.api.mockito.PowerMockito.when;

public class ResultArtifactCreatorTest {

  @Test
  public void testCreateArtifactWithoutGroupId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setPath(new File("path"));

    assertThrowsException(new IllegalArgumentException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithoutArtifactId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId("groupId");
    storageInformation.setPath(new File("path"));

    assertThrowsException(new IllegalArgumentException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithoutPath() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");

    assertThrowsException(new NullPointerException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithNonExistingComponent() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");
    storageInformation.setVersion("1");
    storageInformation.setPath(new File("path"));

    TrainingConfiguration trainingConfiguration = mock(TrainingConfiguration.class);
    when(trainingConfiguration.getModelToTrain()).thenReturn("some.component");


    Exception exception = assertThrows(MojoFailureException.class, () ->
        ResultArtifactCreator.createArtifact(storageInformation, trainingConfiguration, "/tmp", getTaggingResolver())
    );

    assertEquals("Component with name some.component does not exist", exception.getMessage());
  }

  private TaggingResolver getTaggingResolver() {
    String resourcesEmadlUtilsProject = getClass().getClassLoader().getResource("emadl/utils").getFile();

    DeployResultMojo deployResultMojo = spy(DeployResultMojo.class);
    TrainingConfiguration trainingConfiguration = getTrainingConfiguration(resourcesEmadlUtilsProject);
    when(deployResultMojo.getTrainingConfig()).thenReturn(trainingConfiguration);

    return deployResultMojo.getTaggingResolver();
  }

  private TrainingConfiguration getTrainingConfiguration(String projectPath) {
    TrainingConfiguration configuration = mock(TrainingConfiguration.class);
    when(configuration.getPathToProject()).thenReturn(new File(projectPath));
    when(configuration.getPathToTest()).thenReturn(new File(projectPath));

    return configuration;
  }


  private static void assertThrowsException(Exception expectedException, StorageInformation storageInformation)
  {
    assertThrows(expectedException.getClass(), () ->
        ResultArtifactCreator.createArtifact(storageInformation, null, "/tmp", null)
    );
  }

}