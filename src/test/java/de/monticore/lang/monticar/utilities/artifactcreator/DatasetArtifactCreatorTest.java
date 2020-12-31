package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.junit.Test;

import java.io.File;

import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class DatasetArtifactCreatorTest {

  @Test
  public void testCreateArtifactWithoutGroupId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setPath(new File("path"));

    String expectedMessage = "Group ID of dataset artifact must be specified.";

    assertIllegalArgumentExceptionThrown(new IllegalArgumentException(), storageInformation, expectedMessage);
  }

  @Test
  public void testCreateArtifactWithoutArtifactId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId("groupId");
    storageInformation.setPath(new File("path"));

    String expectedMessage = "Artifact ID of dataset artifact must be specified.";

    assertIllegalArgumentExceptionThrown(new IllegalArgumentException(), storageInformation, expectedMessage);
  }

  @Test
  public void testCreateArtifactWithoutPath() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");

    String expectedMessage = "Path of dataset must be specified.";

    assertIllegalArgumentExceptionThrown(new NullPointerException(), storageInformation, expectedMessage);
  }


  private void assertIllegalArgumentExceptionThrown(Exception expectedException, StorageInformation storageInformation, String expectedMessage) {
    Exception exception = assertThrows(expectedException.getClass(), () ->
        DatasetArtifactCreator.createArtifact(storageInformation, "/tmp")
    );

    assertTrue(exception.getMessage().contains(expectedMessage));
  }
}