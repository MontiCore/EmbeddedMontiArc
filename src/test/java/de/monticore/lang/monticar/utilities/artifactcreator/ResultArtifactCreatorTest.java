package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.junit.Test;

import java.io.File;

import static org.junit.Assert.*;

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

  private static void assertThrowsException(Exception expectedException, StorageInformation storageInformation) {
    assertThrows(expectedException.getClass(), () ->
        ResultArtifactCreator.createArtifact(storageInformation, null, "/tmp", null)
    );
  }

}