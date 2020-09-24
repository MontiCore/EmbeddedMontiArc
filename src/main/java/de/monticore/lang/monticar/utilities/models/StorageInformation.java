package de.monticore.lang.monticar.utilities.models;

import java.io.File;

public class StorageInformation {

  private String groupId;

  private String artifactId;

  private File path;

  private Integer version;

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

  public Integer getVersion() {
    return version;
  }

  public void setVersion(Integer version) {
    this.version = version;
  }

  public String getDescription() {
    return description;
  }

  public void setGroupId(String groupId) {
    this.groupId = groupId;
  }

  public void setArtifactId(String artifactId) {
    this.artifactId = artifactId;
  }

  public void setPath(File path) {
    this.path = path;
  }

  public void setDescription(String description) {
    this.description = description;
  }
}
