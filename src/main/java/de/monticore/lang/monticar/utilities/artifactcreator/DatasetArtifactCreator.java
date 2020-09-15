package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.utilities.models.DatasetToStore;

import java.io.File;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class DatasetArtifactCreator {

  public void createArtifact(DatasetToStore datasetToStore, String tempDirectory) {
    String datasetName = datasetToStore.getName();
    String datasetPath = datasetToStore.getPath();
    Preconditions.checkNotNull(datasetName);
    Preconditions.checkNotNull(datasetPath);

    Manifest manifest = createManifest(datasetName);
    String jarFileName = createJarFileName(tempDirectory);
    List<FileLocation> datasetLocations = getDatasetLocations(datasetPath);

    JarCreator.create(jarFileName, manifest, datasetLocations);
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

  private String createJarFileName(String tempDirectory) {
    return String.format("%s%s%s%sdataset.jar", System.getProperty("user.dir"), File.separator, tempDirectory, File.separator);
  }

  private List<FileLocation> getDatasetLocations(String datasetPath) {
    File directory = new File(datasetPath);
    List<FileLocation> datasetLocations = new LinkedList<>();

    for (File dataset: directory.listFiles()) {
      String datasetName = dataset.getName();

      // IF dataset is a h5 file
      if (datasetName.matches(".*\\.h5$")) {
        FileLocation fileLocation = new FileLocation();
        fileLocation.setJarLocation(String.format("dataset%s%s", File.separator, dataset.getName()));
        fileLocation.setSourceLocation((new File(datasetPath, dataset.getName()).getAbsolutePath()));

        datasetLocations.add(fileLocation);
      }
    }

    return datasetLocations;
  }

}
