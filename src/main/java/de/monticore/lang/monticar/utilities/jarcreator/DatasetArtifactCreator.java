package de.monticore.lang.monticar.utilities.jarcreator;

import java.util.LinkedList;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class DatasetArtifactCreator {

  public void createArtifact() {
    Manifest manifest = createManifest();
    String jarFileName = System.getProperty("user.dir") + "/dataset.jar";

    String logFile = "/home/abdallah/Documents/RWTH/master_thesis/ma-atouani/"
        + "03.Workspace/digitclassifier/train.log";
    FileLocation fileLocation = new FileLocation();
    fileLocation.setJarLocation("dataset/test.log");
    fileLocation.setSourceLocation(logFile);
    JarCreator.create(jarFileName, manifest, new LinkedList<FileLocation>() { {add(fileLocation);} });
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
