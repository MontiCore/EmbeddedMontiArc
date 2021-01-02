package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.jar.Manifest;

public class PretrainedArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation pretrainedToStore, String tempDirectory) throws MojoExecutionException, IOException {
    checkStorageInformation(pretrainedToStore, "pretrained network");
    String pretrainedGroupId = pretrainedToStore.getGroupId();
    String pretrainedArtifactId = pretrainedToStore.getArtifactId();
    File pretrainedPath = pretrainedToStore.getPath();

    getPretrainedNetworkLocations(pretrainedPath);
    Manifest manifest = createManifest(pretrainedGroupId, pretrainedArtifactId, pretrainedToStore.getVersion());
    String jarFileName = createJarFileName(tempDirectory, "pretrained");
    List<FileLocation> pretrainedNetworkLocations = getPretrainedNetworkLocations(pretrainedPath);

    return JarCreator.createArtifact(jarFileName, manifest, pretrainedNetworkLocations);
  }

  private static List<FileLocation> getPretrainedNetworkLocations(File pretrainedPath) throws MojoExecutionException {
    List<FileLocation> pretrainedLocations = new LinkedList<>();

    boolean paramFileFound = false;
    boolean jsonFileFound = false;
    for (File file: Objects.requireNonNull(pretrainedPath.listFiles())) {
      if (file.isDirectory()) {
        continue;
      }

      if (file.getName().startsWith("simple_embedding-") && file.getName().endsWith(".params")) {
        FileLocation fileLocation = new FileLocation();
        fileLocation.setSourceLocation(file.getAbsolutePath());
        fileLocation.setJarLocation(file.getName());

        pretrainedLocations.add(fileLocation);
        paramFileFound = true;
      }
      else if (file.getName().startsWith("simple_embedding-") && file.getName().endsWith(".json")) {
        FileLocation fileLocation = new FileLocation();
        fileLocation.setSourceLocation(file.getAbsolutePath());
        fileLocation.setJarLocation(file.getName());

        pretrainedLocations.add(fileLocation);
        jsonFileFound = true;
      }
    }

    if (!paramFileFound || !jsonFileFound) {
      throw new MojoExecutionException("Directory must contain simple_embedding{...}.param and simple_embedding{...}.json file.");
    }

    return pretrainedLocations;
  }

}
