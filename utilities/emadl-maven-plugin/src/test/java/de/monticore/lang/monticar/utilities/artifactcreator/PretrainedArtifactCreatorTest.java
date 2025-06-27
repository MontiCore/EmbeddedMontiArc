package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.maven.plugin.MojoExecutionException;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.jar.Manifest;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.*;
import static org.powermock.api.mockito.PowerMockito.*;

@RunWith(PowerMockRunner.class)
@PrepareForTest(JarCreator.class)
public class PretrainedArtifactCreatorTest {

  @Test
  public void testCreateArtifactWithoutGroupId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setPath(new File("path"));

    assertThrowsExceptionForCreateArtifact(new IllegalArgumentException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithoutArtifactId() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setGroupId("groupId");
    storageInformation.setPath(new File("path"));

    assertThrowsExceptionForCreateArtifact(new IllegalArgumentException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithoutPath() {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");

    assertThrowsExceptionForCreateArtifact(new NullPointerException(), storageInformation);
  }

  @Test
  public void testCreateArtifactWithValidInformation() throws IOException, MojoExecutionException {
    File pretrainedPath = mock(File.class);
    File jsonFile = new File("json/file/path/simple_embedding-1.json");
    File paramsFile = new File("json/file/path/simple_embedding-1.params");
    when(pretrainedPath.listFiles()).thenReturn(new File[] {jsonFile, paramsFile});

    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");
    storageInformation.setVersion("1");
    storageInformation.setPath(pretrainedPath);

    File jar = new File("");
    mockStatic(JarCreator.class);
    when(JarCreator.createArtifact(anyString(), any(Manifest.class), anyList())).thenReturn(jar);

    File artifact = PretrainedArtifactCreator.createArtifact(storageInformation, "tmp");
    assertEquals(jar, artifact);
  }

  @Test
  public void testGetPretrainedNetworkLocationsForPathWithoutFiles() {
    File pretrainedPath = mock(File.class);
    when(pretrainedPath.listFiles()).thenReturn(new File[0]);

    assertThrowsExceptionForGetLocations(pretrainedPath);
  }

  @Test
  public void testGetPretrainedNetworkLocationsForPathWithoutParamsFile() {
    File pretrainedPath = mock(File.class);
    File jsonFile = new File("json/file/path/simple_embedding-1.json");
    when(pretrainedPath.listFiles()).thenReturn(new File[] {jsonFile});

    assertThrowsExceptionForGetLocations(pretrainedPath);
  }

  @Test
  public void testGetPretrainedNetworkLocationsForPathWithoutJSONFile() {
    File pretrainedPath = mock(File.class);
    File paramsFile = new File("json/file/path/simple_embedding-1.params");
    when(pretrainedPath.listFiles()).thenReturn(new File[] {paramsFile});

    assertThrowsExceptionForGetLocations(pretrainedPath);
  }

  @Test
  public void testGetPretrainedNetworkLocationsForPathWithWrongFiles() {
    File pretrainedPath = mock(File.class);
    File jsonFile = new File("json/file/path/test.xml");
    File paramsFile = new File("json/file/path/wrong.java");
    when(pretrainedPath.listFiles()).thenReturn(new File[] {jsonFile, paramsFile});

    assertThrowsExceptionForGetLocations(pretrainedPath);
  }


  @Test
  public void testGetPretrainedNetworkLocationsForPathWithValidInformation() throws MojoExecutionException {
    File pretrainedPath = mock(File.class);
    File jsonFile = new File("json/file/path/simple_embedding-1.json");
    File paramsFile = new File("json/file/path/simple_embedding-1.params");
    File directory = new File(getClass().getClassLoader().getResource("").getFile());
    when(pretrainedPath.listFiles()).thenReturn(new File[] {paramsFile, jsonFile, directory});

    List<FileLocation> fileLocations = PretrainedArtifactCreator.getPretrainedNetworkLocations(pretrainedPath);

    assertEquals(2, fileLocations.size());
    assertEquals(paramsFile.getName(), fileLocations.get(0).getJarLocation());
    assertEquals(paramsFile.getAbsolutePath(), fileLocations.get(0).getSourceLocation());
    assertEquals(jsonFile.getName(), fileLocations.get(1).getJarLocation());
    assertEquals(jsonFile.getAbsolutePath(), fileLocations.get(1).getSourceLocation());
  }

  private static void assertThrowsExceptionForGetLocations(File pretrainedPath) {
    Exception exception = assertThrows(MojoExecutionException.class, () ->
        PretrainedArtifactCreator.getPretrainedNetworkLocations(pretrainedPath)
    );

    assertEquals(
        "Directory must contain {...}.param and {...}.json files, or {...}.onnx file.",
        exception.getMessage()
    );
  }

  private static void assertThrowsExceptionForCreateArtifact(Exception expectedException, StorageInformation storageInformation) {
    assertThrows(expectedException.getClass(), () ->
        PretrainedArtifactCreator.createArtifact(storageInformation, "/tmp")
    );
  }

}