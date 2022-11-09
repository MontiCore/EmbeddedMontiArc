package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.Manifest;

public class TrainingEnvironmentArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation storageInformation, StorageInformation datasetToStore, StorageInformation modelToStore, String tmpDirectory)
      throws IOException {
    List<FileLocation> fileLocations = new LinkedList<>();
    if (datasetToStore != null)
      fileLocations.addAll(DatasetArtifactCreator.getDatasetLocations(datasetToStore.getPath()).values());
    if (modelToStore != null)
      fileLocations.addAll(ModelArtifactCreator.getFileLocations(modelToStore.getPath(), new EMADLParser()));

    Manifest manifest = createManifest(storageInformation.getGroupId(), storageInformation.getArtifactId(), storageInformation.getVersion());
    String jarFileName = createJarFileName(tmpDirectory, "training-env");

    return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
  }

}
