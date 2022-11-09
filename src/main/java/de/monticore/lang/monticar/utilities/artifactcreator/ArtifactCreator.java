package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.monticar.utilities.models.DatasetsConfiguration;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.commons.lang3.StringUtils;

import java.io.File;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

abstract class ArtifactCreator {

  private static final Attributes.Name GROUP_ID = new Attributes.Name("Group-ID");
  private static final Attributes.Name ARTIFACT_ID = new Attributes.Name("Artifact-ID");
  private static final Attributes.Name VERSION = new Attributes.Name("Version");

  public static Manifest createManifest(String groupId, String artifactId, String version) {
    return createManifest(groupId, artifactId, version, new Attributes());
  }

  public static Manifest createManifest(String groupId, String artifactId, String version, Attributes additionalAttributes) {
    Manifest manifest = new Manifest();
    Attributes attributes = manifest.getMainAttributes();
    attributes.put(Attributes.Name.MANIFEST_VERSION, "1.0.0");
    attributes.put(GROUP_ID, groupId);
    attributes.put(ARTIFACT_ID, artifactId);
    attributes.put(VERSION, version);
    attributes.putAll(additionalAttributes);

    return manifest;
  }

  public static String createJarFileName(String tempDirectory, String jarName) {
    return String.format("%s%s%s%s%s.jar", System.getProperty("user.dir"), File.separator, tempDirectory, File.separator, jarName);
  }

  public static void checkStorageInformationWithoutPath(StorageInformation storageInformation, String storingObject) {
    Preconditions.checkArgument(!StringUtils.isEmpty(storageInformation.getGroupId()), "Group ID of " + storingObject + " artifact must be specified.");
    Preconditions.checkArgument(!StringUtils.isEmpty(storageInformation.getArtifactId()), "Artifact ID of " + storingObject + " artifact must be specified.");
  }

  public static void checkStorageInformation(StorageInformation storageInformation, String storingObject){
    checkStorageInformationWithoutPath(storageInformation, storingObject);
    Preconditions.checkNotNull(storageInformation.getPath(), "Path of " + storingObject + " must be specified.");
  }

}
