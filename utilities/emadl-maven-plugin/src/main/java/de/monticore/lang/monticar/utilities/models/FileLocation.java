package de.monticore.lang.monticar.utilities.models;

import java.util.Map;

public class FileLocation {

  private String jarLocation;

  private String sourceLocation;

  private String propertiesLocation;

  private String metadataJarLocation;

  public String getMetadataJarLocation() {
    return metadataJarLocation;
  }

  public void setMetadataJarLocation(String metadataJarLocation) {
    this.metadataJarLocation = metadataJarLocation;
  }

  public String getPropertiesLocation() {
    return propertiesLocation;
  }

  public void setPropertiesLocation(String propertiesLocation) {
    this.propertiesLocation = propertiesLocation;
  }

  public String getJarLocation() {
    return jarLocation;
  }

  public void setJarLocation(String jarLocation) {
    this.jarLocation = jarLocation;
  }

  public String getSourceLocation() {
    return sourceLocation;
  }

  public void setSourceLocation(String sourceLocation) {
    this.sourceLocation = sourceLocation;
  }
}
