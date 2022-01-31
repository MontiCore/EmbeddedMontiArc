package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class DatasetArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation datasetToStore, String tempDirectory) throws IOException {
    checkStorageInformation(datasetToStore, "dataset");
    String datasetGroupId = datasetToStore.getGroupId();
    String datasetArtifactId = datasetToStore.getArtifactId();
    File datasetPath = datasetToStore.getPath();

    Manifest manifest = createManifest(datasetGroupId, datasetArtifactId, datasetToStore.getVersion(), getAdditionalAttributes());
    String jarFileName = createJarFileName(tempDirectory, "dataset");
    List<FileLocation> datasetLocations = getDatasetLocations(datasetPath);

    return JarCreator.createArtifact(jarFileName, manifest, datasetLocations);
  }

  protected static List<FileLocation> getDatasetLocations(File datasetPath) {
    List<FileLocation> datasetLocations = new LinkedList<>();

    for (File dataset : Objects.requireNonNull(datasetPath.listFiles())) {
      String datasetName = dataset.getName();

      // IF dataset is a h5 file
      if (datasetName.matches(".*\\.h5$") || datasetName.matches("(.*)graph")) {
        FileLocation fileLocation = new FileLocation();
        fileLocation.setJarLocation(String.format("training_data%s%s", File.separator, dataset.getName()));
        fileLocation.setSourceLocation((new File(datasetPath, dataset.getName()).getAbsolutePath()));

        datasetLocations.add(fileLocation);
      }
    }

    return datasetLocations;
  }

  private static Attributes getAdditionalAttributes() {
    Attributes attributes = new Attributes();
    attributes.put(new Attributes.Name("File-Type"), "HDF5");

    return attributes;
  }

}
