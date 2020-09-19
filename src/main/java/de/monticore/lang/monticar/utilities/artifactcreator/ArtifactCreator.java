package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.Constants;

import java.io.File;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

abstract class ArtifactCreator {

  private static final Attributes.Name GROUP_ID = new Attributes.Name("Group-ID");
  private static final Attributes.Name ARTIFACT_ID = new Attributes.Name("Artifact-ID");
  private static final Attributes.Name VERSION = new Attributes.Name("Version");

  public static Manifest createManifest(String groupId, String artifactId) {
    Manifest manifest = new Manifest();
    Attributes attributes = manifest.getMainAttributes();
    attributes.put(Attributes.Name.MANIFEST_VERSION, "1.0.0");
    attributes.put(GROUP_ID, groupId);
    attributes.put(ARTIFACT_ID, artifactId);
    attributes.put(VERSION, String.valueOf(Constants.INITIAL_VERSION));

    return manifest;
  }

  public static String createJarFileName(String tempDirectory, String jarName) {
    return String.format("%s%s%s%s%s.jar", System.getProperty("user.dir"), File.separator, tempDirectory, File.separator, jarName);
  }

}
