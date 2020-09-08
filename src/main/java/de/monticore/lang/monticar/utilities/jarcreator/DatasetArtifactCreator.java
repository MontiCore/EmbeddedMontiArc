package de.monticore.lang.monticar.utilities.jarcreator;

import java.io.File;
import java.util.LinkedList;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class DatasetArtifactCreator {

  public void createArtifact(String pathToDataset) {
    Manifest manifest = createManifest();
    String jarFileName = System.getProperty("user.dir") + "/dataset.jar";

    FileLocation trainDataLocation = new FileLocation();
    trainDataLocation.setJarLocation("dataset/train.h5");
    File trainDataFile = new File(pathToDataset, "train.h5");
    trainDataLocation.setSourceLocation(trainDataFile.getAbsolutePath());

    FileLocation testDataLocation = new FileLocation();
    testDataLocation.setJarLocation("dataset/test.h5");
    File testDataFile = new File(pathToDataset, "test.h5");
    testDataLocation.setSourceLocation(testDataFile.getAbsolutePath());

    JarCreator.create(jarFileName, manifest, new LinkedList<FileLocation>() { {add(trainDataLocation); add(testDataLocation);} });
  }

  private Manifest createManifest() {
    String version = "1.0.0";
    String author = "Abdallah Atouani";
    Manifest manifest = new Manifest();
    Attributes attributes = manifest.getMainAttributes();
    attributes.put(Attributes.Name.MANIFEST_VERSION, version);
    attributes.put(new Attributes.Name("Created-by"), author);

    return manifest;
  }

}
