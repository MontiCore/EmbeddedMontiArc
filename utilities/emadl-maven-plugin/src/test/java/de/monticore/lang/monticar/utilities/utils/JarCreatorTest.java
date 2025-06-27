package de.monticore.lang.monticar.utilities.utils;

import de.monticore.lang.monticar.utilities.models.FileLocation;
import org.apache.commons.io.IOUtils;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.Enumeration;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.jar.Manifest;

import static org.junit.Assert.*;

public class JarCreatorTest {

  @Rule
  public TemporaryFolder tmpFolder = new TemporaryFolder();

  @Test
  public void testCreateArtifact() throws IOException {
    File jarFile =  tmpFolder.newFile("test.jar");
    File argMaxEmadlFile = new File(getClass().getClassLoader().getResource("emadl/utils/ArgMax.emadl").getFile());
    String metadataFile = Files.createTempFile("hello!", "").toAbsolutePath().toString();

    Manifest manifest = new Manifest();

    JarCreator.createArtifact(jarFile.getAbsolutePath(), manifest, getFileLocation(argMaxEmadlFile, "utils/ArgMax.emadl", metadataFile, "utils/ArgMax.emadl.json"));

    JarFile jar = new JarFile(jarFile);
    InputStream jarEntryStream = jar.getInputStream(jar.getEntry("utils/ArgMax.emadl"));
    InputStream argMaxFileStream = new FileInputStream(argMaxEmadlFile);

    assertEquals(3, jar.size());
    assertTrue(IOUtils.contentEquals(jarEntryStream, argMaxFileStream));
  }

  private List<FileLocation> getFileLocation(File file, String jarLocation, String propertiesLocation, String metadataLocation) {
    List<FileLocation> fileLocations = new LinkedList<>();

    FileLocation fileLocation = new FileLocation();
    fileLocation.setSourceLocation(file.getAbsolutePath());
    fileLocation.setJarLocation(jarLocation);
    fileLocation.setPropertiesLocation(propertiesLocation);
    fileLocation.setMetadataJarLocation(metadataLocation);

    fileLocations.add(fileLocation);
    return fileLocations;
  }

}