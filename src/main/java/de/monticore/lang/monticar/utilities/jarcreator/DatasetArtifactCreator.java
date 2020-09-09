package de.monticore.lang.monticar.utilities.jarcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.utilities.models.DatasetToStore;

import java.io.File;
import java.util.LinkedList;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class DatasetArtifactCreator {

  public void createArtifact(DatasetToStore datasetToStore) {
    String datasetName = datasetToStore.getName();
    String datasetPath = datasetToStore.getPath();
    Preconditions.checkNotNull(datasetName);
    Preconditions.checkNotNull(datasetPath);

    Manifest manifest = createManifest(datasetName);
    String jarFileName = createJarFileName();

    FileLocation trainDataLocation = new FileLocation();
    trainDataLocation.setJarLocation(String.format("dataset%strain.h5", File.separator));
    File trainDataFile = new File(datasetPath, "train.h5");
    trainDataLocation.setSourceLocation(trainDataFile.getAbsolutePath());

    FileLocation testDataLocation = new FileLocation();
    testDataLocation.setJarLocation(String.format("dataset%stest.h5", File.separator));
    File testDataFile = new File(datasetPath, "test.h5");
    testDataLocation.setSourceLocation(testDataFile.getAbsolutePath());

    JarCreator.create(jarFileName, manifest, new LinkedList<FileLocation>() { {add(trainDataLocation); add(testDataLocation);} });
  }

  private Manifest createManifest(String name) {
    String version = "1.0.0";
    Manifest manifest = new Manifest();
    Attributes attributes = manifest.getMainAttributes();
    attributes.put(Attributes.Name.MANIFEST_VERSION, version);
    attributes.put(new Attributes.Name("name"), name);
    attributes.put(new Attributes.Name("version"), version);

    return manifest;
  }

  private String createJarFileName() {
    return String.format("%s%sdataset.jar", System.getProperty("user.dir"), File.separator);
  }

}
