package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.plugin.MojoExecutionException;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.JarFile;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.anyString;
import static org.powermock.api.mockito.PowerMockito.*;

@RunWith(PowerMockRunner.class)
@PrepareForTest(ArtifactCreator.class)
public class ModelArtifactCreatorTest {

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
  public void testCreateArtifactWithoutValidInformation() throws IOException, MojoExecutionException {
    File modelPath =  new File(getClass().getClassLoader().getResource("emadl/classifier").getFile());
    StorageInformation storageInformation = new StorageInformation();
    storageInformation.setArtifactId("artifactId");
    storageInformation.setGroupId("groupId");
    storageInformation.setVersion("1");
    storageInformation.setPath(modelPath);

    spy(ArtifactCreator.class);
    when(ArtifactCreator.createJarFileName(anyString(), anyString())).thenReturn(tmpFolder.getRoot().getAbsolutePath() + "/model.jar");

    File artifact = ModelArtifactCreator.createArtifact(storageInformation, "");
    JarFile jar = new JarFile(artifact);

    assertTrue(artifact.exists());
    assertEquals(4, jar.size());

  }

  @Test
  public void testGetFileLocationsWithoutCNNTFile() throws IOException {
    File modelPath =  new File(getClass().getClassLoader().getResource("emadl/utils/").getFile());
    EMADLParser emadlParser = new EMADLParser();

    List<FileLocation> fileLocations = ModelArtifactCreator.getFileLocations(modelPath, emadlParser);

    assertEquals(1, fileLocations.size());
    assertEquals(modelPath.getAbsolutePath() + "/ArgMax.emadl", fileLocations.get(0).getSourceLocation());
    assertEquals("utils/ArgMax.emadl", fileLocations.get(0).getJarLocation());
  }

  @Test
  public void testGetFileLocationsWithCNNTFileAndSubpackage() throws IOException {
    File modelPath =  mock(File.class);
    EMADLParser emadlParser = new EMADLParser();

    File utilsDirectory = new File(getClass().getClassLoader().getResource("emadl/classifier/utils").getFile());
    File networkEMADL = new File(getClass().getClassLoader().getResource("emadl/classifier/Network.emadl").getFile());
    File networkCNNT = new File(getClass().getClassLoader().getResource("emadl/classifier/Network.cnnt").getFile());
    File[] files = new File[] {utilsDirectory, networkEMADL, networkCNNT};

    when(modelPath.listFiles()).thenReturn(files);
    List<FileLocation> fileLocations = ModelArtifactCreator.getFileLocations(modelPath, emadlParser);

    assertEquals(3, fileLocations.size());
    assertEquals(utilsDirectory.getAbsolutePath() + "/ArgMax.emadl", fileLocations.get(0).getSourceLocation());
    assertEquals("classifier/utils/ArgMax.emadl", fileLocations.get(0).getJarLocation());
    assertEquals(networkEMADL.getAbsolutePath(), fileLocations.get(1).getSourceLocation());
    assertEquals("classifier/Network.emadl", fileLocations.get(1).getJarLocation());
    assertEquals(networkCNNT.getAbsolutePath(), fileLocations.get(2).getSourceLocation());
    assertEquals("classifier/Network.cnnt", fileLocations.get(2).getJarLocation());
  }

  @Test
  public void testGetPackagePathWithEmptyPackageList() {
    String packagePath = ModelArtifactCreator.getPackagePath(Collections.emptyList());
    assertEquals("", packagePath);
  }

  @Test
  public void testGetPackagePathWithNonEmptyPackageList() {
    List<String> packageList = new LinkedList<>();
    packageList.add("classifier");
    packageList.add("utils");

    String packagePath = ModelArtifactCreator.getPackagePath(packageList);
    assertEquals("classifier/utils/", packagePath);
  }

  private static void assertThrowsException(Exception expectedException, StorageInformation storageInformation) {
    assertThrows(expectedException.getClass(), () ->
        ModelArtifactCreator.createArtifact(storageInformation, "/tmp")
    );
  }


}