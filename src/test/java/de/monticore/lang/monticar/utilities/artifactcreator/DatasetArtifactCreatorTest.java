package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.jar.JarFile;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.anyString;
import static org.powermock.api.mockito.PowerMockito.spy;
import static org.powermock.api.mockito.PowerMockito.when;

@RunWith(PowerMockRunner.class)
@PrepareForTest(ArtifactCreator.class)
public class DatasetArtifactCreatorTest {

  @Rule
  public TemporaryFolder tmpFolder = new TemporaryFolder();

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
  public void testCreateArtifactWithValidInformation() throws IOException {
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");
    storageInformation.setVersion(1);
    storageInformation.setPath(
        new File(getClass().getClassLoader().getResource("dataset/episodicMemorySimple").getFile())
    );

    spy(ArtifactCreator.class);
    when(ArtifactCreator.createJarFileName(anyString(), anyString())).thenReturn(tmpFolder.getRoot().getAbsolutePath() + "/dataset.jar");

    File artifact = DatasetArtifactCreator.createArtifact(storageInformation, tmpFolder.getRoot().getAbsolutePath());
    JarFile jar = new JarFile(artifact);

    assertTrue(artifact.exists());
    assertEquals(3, jar.size());
  }

  @Test
  public void testGetDatasetLocations() {
    File datasetPath = new File(getClass().getClassLoader().getResource("dataset/episodicMemorySimple").getFile());
    List<FileLocation> datasetLocations = DatasetArtifactCreator.getDatasetLocations(datasetPath);

    assertEquals(2, datasetLocations.size());
    assertEquals("training_data/train.h5", datasetLocations.get(0).getJarLocation());
    assertEquals("training_data/test.h5", datasetLocations.get(1).getJarLocation());
  }


  private static void assertThrowsException(Exception expectedException, StorageInformation storageInformation) {
    assertThrows(expectedException.getClass(), () ->
        DatasetArtifactCreator.createArtifact(storageInformation, "/tmp")
    );
  }
}