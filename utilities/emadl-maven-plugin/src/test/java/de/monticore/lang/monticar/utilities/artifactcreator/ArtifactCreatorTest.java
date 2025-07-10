package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.junit.Test;

import java.io.File;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

import static org.junit.Assert.*;

public class ArtifactCreatorTest {

  private final String groupId = "com.awesome.company";
  private final String artifactId = "awesome-product";
  private final String version = "2";

  @Test
  public void testCreateManifestWithoutAdditionalAttributes() {
    Manifest manifest = ArtifactCreator.createManifest(groupId, artifactId, version);
    Attributes attributes = manifest.getMainAttributes();

    assertEquals(groupId, attributes.getValue("Group-ID"));
    assertEquals(artifactId, attributes.getValue("Artifact-ID"));
    assertEquals(String.valueOf(version), attributes.getValue("Version"));
    assertEquals(4, attributes.size());
  }

  @Test
  public void testCreateManifestWithAdditionalAttributes() {
    Attributes.Name contributorAttribute = new Attributes.Name("Contributor");

    Attributes additionalAttributes = new Attributes();
    additionalAttributes.put(contributorAttribute, "Test guy");

    Manifest manifest = ArtifactCreator.createManifest(groupId, artifactId, version, additionalAttributes);
    Attributes attributes = manifest.getMainAttributes();

    assertEquals(groupId, attributes.getValue("Group-ID"));
    assertEquals(artifactId, attributes.getValue("Artifact-ID"));
    assertEquals(String.valueOf(version), attributes.getValue("Version"));
    assertEquals("Test guy", additionalAttributes.getValue(contributorAttribute));
    assertEquals(5, attributes.size());
  }

  @Test
  public void testCreateJarFileName() {
    String tempDirectory = "target";
    String jarName = "dataset";

    String jarFileName = ArtifactCreator.createJarFileName(tempDirectory, jarName).replace('\\', '/');

    String expected = System.getProperty("user.dir").replace('\\', '/') + "/target/dataset.jar";
    assertEquals(expected, jarFileName);
  }

  @Test
  public void testCheckStorageInformationWithMissingGroupId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setPath(new File("path"));

    String expectedMessage = "Group ID of dataset artifact must be specified.";

    assertThrowsExceptionWithMessage(new IllegalArgumentException(), storageInformation, "dataset", expectedMessage);
  }

  @Test
  public void testCheckStorageInformationWithMissingArtifactId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId("groupId");
    storageInformation.setPath(new File("path"));

    String expectedMessage = "Artifact ID of EMADL project artifact must be specified.";

    assertThrowsExceptionWithMessage(new IllegalArgumentException(), storageInformation, "EMADL project", expectedMessage);
  }

  @Test
  public void testCheckStorageInformationWithMissingPath() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");

    String expectedMessage = "Path of pretrained network must be specified.";

    assertThrowsExceptionWithMessage(new NullPointerException(), storageInformation, "pretrained network", expectedMessage);
  }

  @Test
  public void testCheckStorageInformationWithValidInformation() {
    // Test passes if not exception is thrown
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");
    storageInformation.setPath(new File(""));

    ArtifactCreator.checkStorageInformation(storageInformation, "dataset");
  }

  public static void assertThrowsExceptionWithMessage(Exception expectedException, StorageInformation storageInformation,String storingObject,
      String expectedMessage)
  {
    Exception exception = assertThrows(expectedException.getClass(), () ->
        ArtifactCreator.checkStorageInformation(storageInformation, storingObject)
    );

    assertTrue(exception.getMessage().contains(expectedMessage));
  }

}