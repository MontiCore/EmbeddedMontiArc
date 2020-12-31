package de.monticore.lang.monticar.utilities.artifactcreator;

import org.junit.Test;

import java.util.jar.Attributes;
import java.util.jar.Manifest;

import static org.junit.Assert.assertEquals;

public class ArtifactCreatorTest {

  private final String groupId = "com.awesome.company";
  private final String artifactId = "awesome-product";
  private final int version = 2;

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

    String jarFileName = ArtifactCreator.createJarFileName(tempDirectory, jarName);

    String expected = System.getProperty("user.dir") + "/target/dataset.jar";
    assertEquals(expected, jarFileName);

  }

}