package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.Dataset;
import de.monticore.lang.monticar.utilities.models.DatasetsConfiguration;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import de.monticore.lang.monticar.utilities.utils.JsonCreator;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Files;
import java.util.*;
import java.util.jar.Attributes;
import java.util.jar.Manifest;
import java.util.stream.Collectors;

public class DatasetArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation datasetToStore, String tempDirectory) throws IOException {
    checkStorageInformation(datasetToStore, "dataset");
    String datasetGroupId = datasetToStore.getGroupId();
    String datasetArtifactId = datasetToStore.getArtifactId();
    File datasetPath = datasetToStore.getPath();

    Manifest manifest = createManifest(datasetGroupId, datasetArtifactId, datasetToStore.getVersion(), getAdditionalAttributes());
    String jarFileName = createJarFileName(tempDirectory, "dataset");
    Map<String, FileLocation> datasetLocations = getDatasetLocations(datasetPath);

    if(datasetLocations.isEmpty()) {
      throw new RuntimeException("No dataset found. You have to provide at least one dataset.");
    }

    return JarCreator.createArtifact(jarFileName, manifest, datasetLocations);
  }

  public static File createArtifact(DatasetsConfiguration datasetsToStore, String tempDirectory) throws IOException {
    checkStorageInformationWithoutPath(datasetsToStore, "dataset");
    String datasetGroupId = datasetsToStore.getGroupId();
    String datasetArtifactId = datasetsToStore.getArtifactId();

    Manifest manifest = createManifest(datasetGroupId, datasetArtifactId, datasetsToStore.getVersion(), getAdditionalAttributes());
    String jarFileName = createJarFileName(tempDirectory, "dataset");
    Map<String, FileLocation> datasetLocations = getDatasetLocations(datasetsToStore.getDatasets());
    JsonCreator.createJSONMetadata(datasetLocations, datasetsToStore.getDatasets());

    return JarCreator.createArtifact(jarFileName, manifest, datasetLocations);
  }


  protected static Map<String, FileLocation> getDatasetLocations(File datasetPath) {
    HashMap<String, FileLocation> datasetLocations = new HashMap<>();

    for (File dataset : Objects.requireNonNull(datasetPath.listFiles())) {
      String datasetName = dataset.getName();

      // IF dataset is a h5 file
      if (datasetName.matches(".*\\.h5$") || datasetName.matches("(.*)graph")) {
        FileLocation fileLocation = new FileLocation();
        fileLocation.setJarLocation(String.format("training_data%s%s", File.separator, dataset.getName()));
        fileLocation.setSourceLocation((new File(datasetPath, dataset.getName()).getAbsolutePath()));

        try {
          fileLocation.setPropertiesLocation(Files.createTempFile("metadata", "").toAbsolutePath().toString());
        } catch (IOException e) {
          e.printStackTrace();
        }

        fileLocation.setMetadataJarLocation(String.format(String.format("data%s%s", File.separator,  datasetName + ".json")));

        datasetLocations.put(dataset.getName(), fileLocation);
      }
    }

    return datasetLocations;
  }

  protected static Map<String, FileLocation> getDatasetLocations(List<Dataset> datasets) throws IOException {
    return datasets.stream().collect(Collectors.toMap(Dataset::getId, (Dataset dataset) -> {
      FileLocation fileLocation = new FileLocation();
      fileLocation.setJarLocation(String.format("data%s%s", File.separator, dataset.getId() + getFileExtension(dataset.getPath())));
      fileLocation.setSourceLocation((new File(dataset.getPath()).getAbsolutePath()));
      try {
        fileLocation.setPropertiesLocation(Files.createTempFile("metadata", "").toAbsolutePath().toString());
      } catch (IOException e) {
        e.printStackTrace();
      }

      fileLocation.setMetadataJarLocation(String.format(String.format("data%s%s", File.separator, dataset.getId() + ".json")));
      return fileLocation;
    }));
  }

  public static String getFileExtension(String filePath){
    if(!filePath.contains(".")){
      return "";
    }

    return "." + filePath.substring(filePath.lastIndexOf(".") + 1);
  }


  private static Attributes getAdditionalAttributes() {
    Attributes attributes = new Attributes();
    attributes.put(new Attributes.Name("File-Type"), "HDF5");

    return attributes;
  }

}
