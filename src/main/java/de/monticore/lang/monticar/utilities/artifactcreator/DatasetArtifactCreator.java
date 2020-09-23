package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.lang3.StringUtils;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.jar.Manifest;

public class DatasetArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation datasetToStore, String tempDirectory) throws IOException {
    String datasetGroupId = datasetToStore.getGroupId();
    String datasetArtifactId = datasetToStore.getArtifactId();
    File datasetPath = datasetToStore.getPath();
    Preconditions.checkArgument(!StringUtils.isEmpty(datasetGroupId), "Group ID of dataset artifact must be specified.");
    Preconditions.checkArgument(!StringUtils.isEmpty(datasetArtifactId), "Artifact ID of dataset artifact must be specified.");
    Preconditions.checkNotNull(datasetPath, "Path of dataset must be specified.");

    Manifest manifest = createManifest(datasetGroupId, datasetArtifactId, datasetToStore.getVersion());
    String jarFileName = createJarFileName(tempDirectory, "dataset");
    List<FileLocation> datasetLocations = getDatasetLocations(datasetPath);

    return JarCreator.createArtifact(jarFileName, manifest, datasetLocations);
  }

  private static List<FileLocation> getDatasetLocations(File datasetPath) {
    List<FileLocation> datasetLocations = new LinkedList<>();

    for (File dataset: Objects.requireNonNull(datasetPath.listFiles())) {
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
