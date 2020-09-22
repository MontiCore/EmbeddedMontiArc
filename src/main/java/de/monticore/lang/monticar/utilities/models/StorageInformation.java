package de.monticore.lang.monticar.utilities.models;

import java.io.File;

public class StorageInformation {

  private String groupId;

  private String artifactId;

  private File path;

  private String description;

  public String getGroupId() {
    return groupId;
  }

  public String getArtifactId() {
    return artifactId;
  }

  public File getPath() {
    return path;
  }

  public String getDescription() {
    return description;
  }
}
