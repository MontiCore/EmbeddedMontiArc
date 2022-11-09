package de.monticore.lang.monticar.utilities.models;

import java.io.File;
import java.util.List;

public class DatasetsConfiguration extends StorageInformation{
    private String groupId;

    private String artifactId;

    private String version;

    private String description;

    private List<Dataset> datasets;

    public String getGroupId() {
        return groupId;
    }

    public String getArtifactId() {
        return artifactId;
    }

    public String getVersion() {
        return version;
    }

    public void setVersion(String version) {
        this.version = version;
    }

    public String getDescription() {
        return description;
    }

    public void setGroupId(String groupId) {
        this.groupId = groupId;
    }

    public List<Dataset> getDatasets() {
        return datasets;
    }

    public void setDatasets(List<Dataset> datasets) {
        this.datasets = datasets;
    }

    public void setArtifactId(String artifactId) {
        this.artifactId = artifactId;
    }

    public void setDescription(String description) {
        this.description = description;
    }
}
